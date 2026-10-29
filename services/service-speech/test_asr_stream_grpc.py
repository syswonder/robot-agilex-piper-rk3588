#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Simulated-voice test for the CONTAINER speech gRPC asr_stream servicer.

Exercises the FULL container path without a microphone:
    this client → RobonixServiceSpeechAsrStream.RecognizeStream (127.0.0.1:PORT)
                → speech_service (SPEECH_BACKEND=custom, RemoteFunASRStreamingBackend)
                → host FunASR ws://127.0.0.1:8771 → text back up the chain.

Run INSIDE the container with the built speech venv + codegen on PYTHONPATH:
    cd /root/workspace/services/service-speech
    export PYTHONPATH="$PWD/rbnx-build/codegen/proto_gen:$PWD/rbnx-build/codegen/robonix_mcp_types:$PYTHONPATH"
    rbnx-build/venv/bin/python test_asr_stream_grpc.py --port 43089 [WAV]

The port is the speech service's "Lifecycle gRPC serving on 0.0.0.0:PORT" from
rbnx-boot/logs/speech.log (ephemeral — re-read it each boot).
"""
from __future__ import annotations

import argparse
import sys
import time
import wave

DEFAULT_WAV = (
    "/home/czn/.cache/modelscope/hub/models/iic/"
    "speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-online/"
    "example/asr_example.wav"
)


def read_pcm(path: str) -> bytes:
    with wave.open(path, "rb") as w:
        assert w.getnchannels() == 1 and w.getframerate() == 16000 and w.getsampwidth() == 2, (
            f"want 16kHz mono s16le, got {w.getnchannels()}ch "
            f"{w.getframerate()}Hz {w.getsampwidth()*8}bit"
        )
        return w.readframes(w.getnframes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("wav", nargs="?", default=DEFAULT_WAV)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--chunk-ms", type=int, default=600)
    args = ap.parse_args()

    import grpc
    import asr_pb2 as pb              # AsrAudioChunk, RecognizeStreamEvent
    import robonix_contracts_pb2_grpc as pbg
    # AudioChunk lives in the audio_pb2 module (robonix.audio namespace).
    import audio_pb2

    pcm = read_pcm(args.wav)
    step = int(16000 * 2 * args.chunk_ms / 1000)
    target = f"{args.host}:{args.port}"
    print(f"[test] wav={args.wav}")
    print(f"[test] {len(pcm)} bytes ({len(pcm)/32000:.2f}s), {args.chunk_ms}ms chunks, target={target}")

    ch = grpc.insecure_channel(target)
    grpc.channel_ready_future(ch).result(timeout=10)
    stub = pbg.RobonixServiceSpeechAsrStreamStub(ch)

    def _gen():
        seq = 0
        for i in range(0, len(pcm), step):
            data = pcm[i:i + step]
            chunk = audio_pb2.AudioChunk(
                timestamp_ns=time.time_ns(),
                data=data,
                sequence=seq,
                duration_s=len(data) / 32000.0,
            )
            seq += 1
            yield pb.AsrAudioChunk(chunk=chunk)
            time.sleep(0.02)

    finals: list[str] = []
    partials = 0
    last_partial = ""
    try:
        for ev in stub.RecognizeStream(_gen(), timeout=60):
            if ev.error:
                print(f"[test]  ERROR event: {ev.error}")
                continue
            if ev.is_final or ev.event_type == 1:
                finals.append(ev.text)
                print(f"[test]  FINAL: {ev.text!r} (conf={ev.confidence:.2f})")
            else:
                partials += 1
                last_partial = ev.text
                print(f"[test]  partial: {ev.text!r}")
    except grpc.RpcError as e:
        print(f"[test] RPC FAILED: {e.code()} {e.details()}")
        return 1

    print(f"\n[test] done. {partials} partial(s), {len(finals)} final(s).")
    joined = "".join(finals) if finals else last_partial
    print(f"[test] transcript: {joined!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
