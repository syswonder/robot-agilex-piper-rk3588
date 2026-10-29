#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Microphone audio primitive — Capability-based driver (mic only, no speaker).

Owns `robonix/primitive/audio/{mic,driver,list_devices,select_device}`:
  primitive/audio/mic            server-stream `Stream() returns (stream AudioChunk)`
  primitive/audio/driver         gRPC lifecycle (Capability built-in)
  primitive/audio/list_devices   enumerate ALSA capture devices
  primitive/audio/select_device  repick capture device at runtime

driver-init: start.sh spawns this; `Driver(CMD_INIT, config_json)` scans ALSA,
instantiates the MicDriver, returns ready. The mic handler picks the
module-level driver up the first time a client connects.

This is the provider for liaison's MIC_CONTRACT — without it the handsfree
voice loop can't start (fails wake-word wait with "no mic provider").

Config keys (from the manifest's per-package config: block):
  mic_device       ALSA device id (e.g. "hw:2,0"); empty = auto-detect (prefers USB)
  mic_sample_rate  Hz (default 16000)
  mic_channels     default 1
  mic_bits         default 16
  mic_chunk_ms     default 100
Env overrides (AUDIO_MIC_DEVICE / _SAMPLE_RATE / _CHANNELS / _BITS / _CHUNK_MS)
remain honored for parity with older manifests.
"""
from __future__ import annotations

import logging
import os
import threading

from robonix_api import Primitive, Ok, Err

audio_mic = Primitive(id="audio_mic", namespace="robonix/primitive/audio")
log = logging.getLogger("audio-mic")

import audio_pb2          # type: ignore  # noqa: E402  (codegen)

from audio_mic.alsa_utils import scan_alsa_inputs, find_default_mic  # noqa: E402
from audio_mic.mic_driver import MicDriver, probe_mic_sample_rate  # noqa: E402

mic_driver: MicDriver | None = None
# Currently-selected + deploy-configured device ids (mirror of what mic_driver
# points at). Empty string = OS/ALSA default. Set by select_device / init;
# read back by list_devices.
current_input_id: str = ""
configured_input_id: str = ""
_mic_stream_lock = threading.Lock()


# ── streaming handler ──────────────────────────────────────────────────────
@audio_mic.grpc("robonix/primitive/audio/mic")
def mic_stream(request, context):
    """Server-streaming mic capture with exclusive ALSA ownership.

    One MicDriver owns one arecord process. Reject overlapping clients
    explicitly instead of leaking the previous recorder.
    """
    grpc = __import__("grpc")
    if not _mic_stream_lock.acquire(blocking=False):
        context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED,
                      "microphone is already in use by another stream")
        return
    driver = None
    try:
        driver = mic_driver
        if driver is None:
            context.abort(grpc.StatusCode.UNAVAILABLE,
                          "mic driver not initialized — Driver(CMD_INIT) failed or never ran")
            return
        log.info("mic stream client connected")
        driver.start()
        while context.is_active():
            chunk = driver.read_chunk()
            if chunk is None:
                break
            yield audio_pb2.AudioChunk(
                timestamp_ns=chunk["timestamp_ns"],
                data=chunk["data"],
                sequence=chunk["sequence"],
                duration_s=chunk["duration_s"],
            )
    finally:
        try:
            if driver is not None:
                driver.stop()
                log.info("mic stream client disconnected")
        finally:
            _mic_stream_lock.release()


# ── device list / select ───────────────────────────────────────────────────
def _scan_devices_proto():
    """ALSA scan → AudioDevice protos (capture only)."""
    devs = []
    default_mic = find_default_mic(scan_alsa_inputs())
    default_mic_id = default_mic.device_id if default_mic else ""
    for d in scan_alsa_inputs():
        devs.append(audio_pb2.AudioDevice(
            id=d.device_id,
            name=d.name,
            kind="input",
            is_default=(d.device_id == default_mic_id),
            channels=1,           # arecord -l doesn't report; conservative default
            note="",
        ))
    ids = {d.id for d in devs}
    # Expose the active deployment selection as a first-class choice even if it
    # isn't a plain hw:X,Y card that arecord -l lists (e.g. a plug device).
    for device_id in (current_input_id, configured_input_id,
                      os.environ.get("AUDIO_MIC_DEVICE", "").strip()):
        if device_id and device_id not in ids:
            devs.append(audio_pb2.AudioDevice(
                id=device_id, name="Configured ALSA device", kind="input",
                is_default=False, channels=1, note="active deployment selection"))
            ids.add(device_id)
    return devs


@audio_mic.grpc("robonix/primitive/audio/list_devices")
def list_devices(request, context):
    return audio_pb2.ListAudioDevices_Response(
        devices=_scan_devices_proto(),
        current_input_id=current_input_id,
        current_output_id="",
    )


@audio_mic.grpc("robonix/primitive/audio/select_device")
def select_device(request, context):
    global mic_driver, current_input_id
    kind = (request.kind or "").lower()
    if kind != "input":
        return audio_pb2.SelectAudioDevice_Response(
            ok=False, error=f"mic primitive only supports kind='input', got '{kind}'")

    requested = request.id
    if requested:
        valid = {d.device_id for d in scan_alsa_inputs()}
        configured = os.environ.get("AUDIO_MIC_DEVICE", "").strip()
        if configured:
            valid.add(configured)
        if current_input_id:
            valid.add(current_input_id)
        if configured_input_id:
            valid.add(configured_input_id)
        if requested not in valid:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error=f"unknown input id '{requested}'")
        new_id = requested
    else:
        info = find_default_mic(scan_alsa_inputs())
        if info is None:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error="no default input device")
        new_id = info.device_id

    if not _mic_stream_lock.acquire(blocking=False):
        return audio_pb2.SelectAudioDevice_Response(
            ok=False, error="cannot change input device while microphone is streaming")
    try:
        previous = mic_driver
        if previous is not None:
            try:
                previous.stop()
            except Exception:  # noqa: BLE001
                pass
        mic_rate = (
            int(os.environ["AUDIO_MIC_SAMPLE_RATE"])
            if "AUDIO_MIC_SAMPLE_RATE" in os.environ
            else previous.sample_rate if previous is not None
            else probe_mic_sample_rate(new_id)
        )
        mic_driver = MicDriver(
            device_id=new_id,
            sample_rate=mic_rate,
            channels=int(os.environ.get(
                "AUDIO_MIC_CHANNELS",
                str(previous.channels if previous is not None else 1))),
            bits_per_sample=int(os.environ.get(
                "AUDIO_MIC_BITS",
                str(previous.bits_per_sample if previous is not None else 16))),
            chunk_duration_s=int(os.environ.get(
                "AUDIO_MIC_CHUNK_MS",
                str(round((previous.chunk_duration_s if previous is not None else 0.1) * 1000)))) / 1000.0,
        )
        current_input_id = new_id
    finally:
        _mic_stream_lock.release()
    return audio_pb2.SelectAudioDevice_Response(ok=True, error="")


# ── driver-init lifecycle ──────────────────────────────────────────────────
@audio_mic.on_init
def init(cfg):
    """Configure the ALSA capture device from config, then env, then auto-detect."""
    global mic_driver, current_input_id, configured_input_id

    devices = scan_alsa_inputs()
    for d in devices:
        log.info("ALSA capture: %s (%s)", d.device_id, d.name)

    mic_id = str(cfg.get("mic_device") or os.environ.get("AUDIO_MIC_DEVICE", "")).strip()
    if mic_id:
        log.info("mic override: %s", mic_id)
        mic_dev_id: str | None = mic_id
    else:
        mic_info = find_default_mic(devices)
        if mic_info:
            log.info("mic auto-detected: %s (%s)", mic_info.device_id, mic_info.name)
            mic_dev_id = mic_info.device_id
        else:
            log.warning("no microphone found")
            mic_dev_id = None

    if mic_dev_id is None:
        return Err("no ALSA capture device available")

    configured_mic_rate = cfg.get("mic_sample_rate") or os.environ.get("AUDIO_MIC_SAMPLE_RATE")
    mic_rate = int(configured_mic_rate) if configured_mic_rate else probe_mic_sample_rate(mic_dev_id)
    mic_driver = MicDriver(
        device_id=mic_dev_id,
        sample_rate=mic_rate,
        channels=int(cfg.get("mic_channels") or os.environ.get("AUDIO_MIC_CHANNELS", "1")),
        bits_per_sample=int(cfg.get("mic_bits") or os.environ.get("AUDIO_MIC_BITS", "16")),
        chunk_duration_s=int(cfg.get("mic_chunk_ms") or os.environ.get("AUDIO_MIC_CHUNK_MS", "100")) / 1000.0,
    )
    current_input_id = mic_dev_id
    configured_input_id = mic_dev_id
    log.info("mic ready: device=%s rate=%d ch=%d bits=%d chunk=%.3fs",
             mic_driver.device_id, mic_driver.sample_rate, mic_driver.channels,
             mic_driver.bits_per_sample, mic_driver.chunk_duration_s)
    return Ok()


@audio_mic.on_shutdown
def shutdown():
    global mic_driver
    if mic_driver is not None:
        try:
            mic_driver.stop()
        except Exception:
            pass
        mic_driver = None
    return Ok()


def main() -> int:
    audio_mic.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
