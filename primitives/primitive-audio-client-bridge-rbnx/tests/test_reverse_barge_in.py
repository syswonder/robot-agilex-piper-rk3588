from __future__ import annotations

import asyncio
import json
import queue
from types import SimpleNamespace
import unittest
from unittest import mock

from audio_client_bridge.reverse import ReverseAudioBridge, _ClientSession


class ActiveContext:
    @staticmethod
    def is_active() -> bool:
        return True


class ReverseAudioBridgeTest(unittest.TestCase):
    def test_barge_in_supersedes_inflight_speaker_generation(self) -> None:
        bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
        sent: list[str | bytes] = []
        bridge._require_client = lambda _context: True  # type: ignore[method-assign]
        bridge._send = lambda payload: (sent.append(payload), True)[1]  # type: ignore[method-assign]

        def chunks():
            yield SimpleNamespace(data=b"old-audio-1")
            bridge._interrupt_speaker()
            yield SimpleNamespace(data=b"old-audio-2")

        bridge.speaker_stream(chunks(), ActiveContext(), lambda: object())

        self.assertIn(b"old-audio-1", sent)
        self.assertNotIn(b"old-audio-2", sent)
        controls = [json.loads(item) for item in sent if isinstance(item, str)]
        self.assertEqual({control["type"] for control in controls}, {"speaker_stop"})

    def test_send_forwards_payload_to_connected_client(self) -> None:
        bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
        delivered: list[str | bytes] = []
        marker_loop = object()

        class FakeWebSocket:
            async def send(self, payload: str | bytes) -> None:
                delivered.append(payload)

        class FakeFuture:
            def result(self, timeout: float) -> None:
                self_outer.assertEqual(timeout, 3.0)

        self_outer = self

        def run_coroutine(coro, loop):
            self.assertIs(loop, marker_loop)
            asyncio.run(coro)
            return FakeFuture()

        bridge._session = _ClientSession(ws=FakeWebSocket(), loop=marker_loop)
        with mock.patch(
            "audio_client_bridge.reverse.asyncio.run_coroutine_threadsafe",
            side_effect=run_coroutine,
        ):
            self.assertTrue(bridge._send('{"type":"mic_start"}'))
        self.assertEqual(delivered, ['{"type":"mic_start"}'])

    def test_send_reports_disconnected_client(self) -> None:
        bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
        self.assertFalse(bridge._send('{"type":"mic_start"}'))

    def test_stale_mic_end_does_not_terminate_new_stream(self) -> None:
        bridge = ReverseAudioBridge("127.0.0.1", 0, 3200)
        old_frames: queue.Queue[bytes | None] = queue.Queue()
        new_frames: queue.Queue[bytes | None] = queue.Queue()
        bridge._mic_streams = {"old": old_frames, "new": new_frames}
        bridge._active_mic_id = "new"

        bridge._handle_control('{"type":"mic_end","stream_id":"old"}')
        bridge._put_active_mic(b"new-pcm")

        self.assertIsNone(old_frames.get_nowait())
        self.assertEqual(new_frames.get_nowait(), b"new-pcm")
        self.assertTrue(new_frames.empty())


if __name__ == "__main__":
    unittest.main()
