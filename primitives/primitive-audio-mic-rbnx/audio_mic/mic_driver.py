"""Microphone capture driver using arecord subprocess.

Wraps the ALSA `arecord` CLI to capture raw PCM from a mic device, producing
audio chunks (dicts) for the gRPC mic streaming handler.

    ALSA device → arecord subprocess → MicDriver.read_chunk() → gRPC AudioChunk

Why subprocess (not PyAudio/sounddevice): arecord is universally available on
ALSA systems, needs no C-extension compilation (no portaudio), and is simple
and reliable for fixed-format capture.
"""
import re
import subprocess
import time
import logging

log = logging.getLogger(__name__)

# bits_per_sample → arecord -f argument.
_ALSA_FORMATS = {
    16: "S16_LE",   # 16-bit signed little-endian (standard for ASR)
    24: "S24_LE",
    32: "S32_LE",
    8: "S8",
}


def probe_mic_sample_rate(device_id: str) -> int:
    """Query ALSA hw params to find a supported sample rate, preferring 16000.

    Runs `arecord --dump-hw-params` and parses the RATE line. Falls back to
    16000 on any parse/subprocess failure. Never raises.
    """
    try:
        proc = subprocess.run(
            ["arecord", "-D", device_id, "--dump-hw-params",
             "-f", "S16_LE", "-d", "1", "/dev/null"],
            capture_output=True, text=True, timeout=5,
        )
        output = proc.stdout + "\n" + proc.stderr
    except (FileNotFoundError, subprocess.TimeoutExpired):
        log.warning("probe mic rate for '%s' failed, falling back to 16000 Hz", device_id)
        return 16000

    m = re.search(r"RATE:\s*(.+)", output)
    if not m:
        log.warning("probe mic rate for '%s': no RATE line, using 16000 Hz", device_id)
        return 16000
    rates_str = m.group(1).strip()

    rates: list[int] = []
    range_m = re.match(r"\[(\d+)\s+(\d+)\]", rates_str)
    if range_m:
        rates = [int(range_m.group(1)), int(range_m.group(2))]
    else:
        rates = [int(x) for x in rates_str.split() if x.isdigit()]

    if not rates:
        log.warning("probe mic rate for '%s': empty RATE, using 16000 Hz", device_id)
        return 16000

    PREFERRED = 16000
    if len(rates) == 2 and rates[0] <= PREFERRED <= rates[1]:
        log.info("mic hw RATE [%d .. %d] → using %d Hz", rates[0], rates[1], PREFERRED)
        return PREFERRED
    if PREFERRED in rates:
        log.info("mic hw rates %s → using %d Hz (preferred)", rates, PREFERRED)
        return PREFERRED
    chosen = min(rates)
    log.info("mic hw rates %s → using %d Hz (lowest)", rates, chosen)
    return chosen


class MicDriver:
    """Captures audio from an ALSA device via an arecord subprocess.

    Launches `arecord -D <device> -f <fmt> -r <rate> -c <ch> -t raw` and reads
    fixed-size chunks from its stdout, each wrapped with timestamp/sequence/
    duration metadata.

    NOT thread-safe — one client per MicDriver, driven from a single gRPC
    servicer thread.
    """

    def __init__(
        self,
        device_id: str,
        sample_rate: int = 16000,
        channels: int = 1,
        bits_per_sample: int = 16,
        chunk_duration_s: float = 0.1,
    ):
        self.device_id = device_id
        self.sample_rate = sample_rate
        self.channels = channels
        self.bits_per_sample = bits_per_sample
        self.chunk_duration_s = chunk_duration_s

        self._bytes_per_sample = bits_per_sample // 8
        self._chunk_bytes = (
            int(sample_rate * chunk_duration_s) * channels * self._bytes_per_sample
        )
        self._process: subprocess.Popen | None = None
        self._sequence = 0

    def start(self) -> None:
        """Launch the arecord subprocess. Idempotent-ish: safe to call when
        already running (re-launches a fresh recorder)."""
        fmt = _ALSA_FORMATS.get(self.bits_per_sample)
        if fmt is None:
            raise ValueError(f"Unsupported bits_per_sample={self.bits_per_sample}")
        if self._process is not None and self._process.poll() is None:
            return
        cmd = [
            "arecord", "-D", self.device_id,
            "-f", fmt, "-r", str(self.sample_rate),
            "-c", str(self.channels), "-t", "raw",
        ]
        log.info("Starting mic capture: %s", " ".join(cmd))
        self._process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._sequence = 0

    def read_chunk(self) -> dict | None:
        """Read exactly one chunk (blocking). Returns None on EOF/partial read
        (subprocess ended) — surfaces arecord stderr in the log to explain why."""
        if self._process is None:
            return None
        data = self._process.stdout.read(self._chunk_bytes)
        if not data or len(data) < self._chunk_bytes:
            rc = self._process.poll()
            if rc is not None and rc != 0:
                err = b""
                try:
                    err = self._process.stderr.read() or b""
                except Exception:
                    pass
                log.error("arecord exited rc=%s: %s", rc, err.decode(errors="replace").strip())
            return None
        chunk = {
            "timestamp_ns": int(time.time() * 1e9),
            "data": data,
            "sequence": self._sequence,
            "duration_s": self.chunk_duration_s,
        }
        self._sequence += 1
        return chunk

    def stop(self) -> None:
        """SIGTERM the arecord subprocess, SIGKILL after 5s. Safe to repeat."""
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
            log.info("Mic capture stopped")

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None
