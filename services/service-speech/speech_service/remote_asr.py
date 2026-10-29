# SPDX-License-Identifier: MulanPSL-2.0
"""Remote streaming-ASR backend: a thin WebSocket client.

Why this exists
---------------
The `local` backend loads FunASR (torch + CUDA) *inside* this service process.
In this deploy the robonix container is deliberately DL-free, so the heavy
FunASR model runs on the HOST (docker/host-services/funasr_asr_server.py,
ws://127.0.0.1:8771) and this class just forwards audio to it and relays the
recognition events back.

Contract (identical to FunASRStreamingBackend.recognize_stream):
    recognize_stream(pcm_chunks: Iterable[bytes]) -> Iterator[dict]
    each event dict: {event_type:int, text:str, confidence:float, is_final:bool}
The host reproduces FunASRStreamingBackend VERBATIM, so recognition behaviour
is identical — only a localhost network hop is inserted. The FunASR streaming
*cache* stays entirely on the host (one place), so nothing here is stateful.

Wire protocol (one WS connection per utterance / Stream RPC):
    client → host : binary frames = raw 16 kHz mono pcm_s16le (already adapted
                    by SpeechAsrStreamServicer before it reaches this backend)
    client → host : text frame '{"type":"eos"}'  = end of audio, flush final
    host  → client: text frames = JSON event dicts (as they are produced)
    host  → client: text frame '{"type":"done"}' = stream finished
"""
from __future__ import annotations

import json
import logging
import os
import threading
from typing import Iterable, Iterator

log = logging.getLogger("speech_service.remote_asr")

_DEFAULT_URL = "ws://127.0.0.1:8771"


class RemoteFunASRStreamingBackend:
    """Streaming ASR over WebSocket to the host-side FunASR server."""

    def __init__(self) -> None:
        # Lazy connect: the URL is resolved here but the socket is opened per
        # recognize_stream() call. We do NOT fail construction if the host is
        # down — that keeps `rbnx boot` decoupled from host-server liveness
        # (the servicer still registers ACTIVE; a dead host surfaces as a
        # per-utterance error event, same as any network backend).
        self.url = os.environ.get("FUNASR_REMOTE_URL", _DEFAULT_URL).strip() or _DEFAULT_URL
        # Import here so a missing 'websockets' wheel fails this backend only
        # (via _try_backend) instead of crashing the whole service import.
        from websockets.sync.client import connect as _connect  # noqa: F401
        log.info("RemoteFunASRStreamingBackend target: %s", self.url)

    def recognize_stream(self, pcm_chunks: Iterable[bytes]) -> Iterator[dict]:
        from websockets.sync.client import connect
        from websockets.exceptions import ConnectionClosed

        # max_size=None: recognition JSON is tiny, but disable the frame cap so
        # a burst of audio can never trip the default 1 MiB inbound limit.
        with connect(self.url, max_size=None, open_timeout=10) as ws:
            send_err: list[BaseException] = []

            def _pump() -> None:
                try:
                    for chunk in pcm_chunks:
                        if chunk:
                            ws.send(chunk)  # binary frame
                    ws.send(json.dumps({"type": "eos"}))  # text frame
                except BaseException as exc:  # noqa: BLE001
                    send_err.append(exc)
                    try:
                        ws.close()
                    except Exception:
                        pass

            sender = threading.Thread(target=_pump, name="remote-asr-send", daemon=True)
            sender.start()

            saw_final = False
            try:
                while True:
                    msg = ws.recv()
                    if isinstance(msg, (bytes, bytearray)):
                        continue  # host only ever sends text; ignore stray binary
                    ev = json.loads(msg)
                    if ev.get("type") == "done":
                        break
                    if ev.get("type") == "error":
                        raise RuntimeError(ev.get("message", "remote ASR error"))
                    if ev.get("is_final"):
                        saw_final = True
                    yield {
                        "event_type": int(ev.get("event_type", 1 if ev.get("is_final") else 0)),
                        "text": ev.get("text", ""),
                        "confidence": float(ev.get("confidence", 0.0)),
                        "is_final": bool(ev.get("is_final", False)),
                    }
            except ConnectionClosed:
                if not saw_final:
                    raise
            finally:
                sender.join(timeout=2.0)

            if send_err:
                raise send_err[0]
