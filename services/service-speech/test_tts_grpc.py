#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Simulated TTS test for the CONTAINER speech gRPC tts servicer.

Exercises the TTS path without a speaker:
    this client → RobonixServiceSpeechTts.Synthesize (127.0.0.1:PORT)
                → speech_service (custom mode, speech_tts_backend_class)

With `speech_tts_backend_class: "speech_service.service:MockTTSBackend"` the
rpc must SUCCEED and return 100 ms of silence — i.e. liaison's
"tts skipped: tts rpc failed: ... TTS backend not available" must NOT fire.

Run INSIDE the container with the built speech venv + codegen on PYTHONPATH:
    cd /root/workspace/services/service-speech
    export PYTHONPATH="$PWD/rbnx-build/codegen/proto_gen:$PWD/rbnx-build/codegen/robonix_mcp_types:$PYTHONPATH"
    rbnx-build/venv/bin/python test_tts_grpc.py --port 40631

The port is the speech service's "Lifecycle gRPC serving on 0.0.0.0:PORT" from
rbnx-boot/logs/speech.log (ephemeral — re-read it each boot).
"""
from __future__ import annotations

import argparse
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--text", default="测试一下语音合成链路")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()

    import grpc
    import robonix_contracts_pb2_grpc as pbg
    import tts_pb2

    target = f"{args.host}:{args.port}"
    print(f"[test] target={target} text={args.text!r}")
    t0 = time.monotonic()
    with grpc.insecure_channel(target) as ch:
        stub = pbg.RobonixServiceSpeechTtsStub(ch)
        resp = stub.Synthesize(
            tts_pb2.Synthesize_Request(text=args.text), timeout=args.timeout)
    dt = time.monotonic() - t0

    audio = bytes(resp.audio_data)
    all_silence = audio == b"\x00" * len(audio)   # py3.10: 反斜杠不能写进 f-string
    print(f"[test] rpc ok in {dt:.2f}s")
    print(f"[test]   encoding={resp.encoding!r} sample_rate_hz={resp.sample_rate_hz}")
    print(f"[test]   audio_bytes={len(audio)} ({len(audio)/2/16000*1000:.0f}ms @16k s16le)")
    print(f"[test]   all_silence={all_silence}")
    print(f"[test]   error={resp.error!r}")
    ok = bool(audio) and not resp.error
    print("[test] RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
