"""ALSA capture-device discovery for the mic primitive.

Parses `arecord -l` to find input devices (USB mics included), so init can
auto-pick a default and list_devices can report choices. Speaker/output
scanning is omitted — this primitive is mic-only (no TTS in this deploy).
"""
import re
import subprocess
import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)


@dataclass
class AlsaDeviceInfo:
    """A single ALSA capture device from `arecord -l`.

    device_id is the "plughw:card,device" string passed to arecord -D. We use
    the `plughw:` plugin (not raw `hw:`) on purpose: USB mics commonly capture
    at 2ch/48kHz natively, and raw hw: refuses format conversion, so
    `arecord -D hw:2,0 -c 1 -r 16000` fails with "Channels count non
    available". `plughw:` inserts ALSA's software rate/channel conversion so we
    always get the 16kHz mono s16le the ASR path wants. Verified on the UGREEN
    USB MIC-CM564 (card 2): hw:2,0 fails, plughw:2,0 records clean 16k mono.
    """
    card: int
    device: int
    name: str          # e.g. "UGREEN USB MIC-CM564"
    device_id: str     # e.g. "plughw:2,0"
    is_input: bool = True


def _parse_alsa_inputs(output: str) -> list[AlsaDeviceInfo]:
    """Parse `arecord -l` lines like:
        card 2: MICCM564 [UGREEN USB MIC-CM564], device 0: USB Audio ..."""
    devices = []
    pattern = re.compile(r"card\s+(\d+):\s+(.+?)\s*\[(.+?)\],\s+device\s+(\d+):")
    for match in pattern.finditer(output):
        card = int(match.group(1))
        name = match.group(3).strip()
        device = int(match.group(4))
        devices.append(AlsaDeviceInfo(
            card=card, device=device, name=name,
            device_id=f"plughw:{card},{device}", is_input=True,
        ))
    return devices


def scan_alsa_inputs() -> list[AlsaDeviceInfo]:
    """Scan ALSA capture devices via `arecord -l`. Empty if none / arecord
    missing."""
    try:
        result = subprocess.run(
            ["arecord", "-l"], capture_output=True, text=True, timeout=5)
        output = result.stdout + result.stderr
    except FileNotFoundError:
        log.warning("arecord not found — cannot scan capture devices")
        return []
    except subprocess.TimeoutExpired:
        log.warning("arecord -l timed out")
        return []
    devices = _parse_alsa_inputs(output)
    for d in devices:
        log.debug("ALSA capture: %s (%s)", d.device_id, d.name)
    return devices


def find_default_mic(devices: list[AlsaDeviceInfo] | None = None) -> AlsaDeviceInfo | None:
    """Best default mic: prefer a real USB microphone, else first device.

    Match on the name (contains "usb" or "mic"), NOT on card>=1: on this Jetson
    the SoC audio (APE) is card 1 with 32 routing endpoints that are not real
    microphones, so a bare card>=1 heuristic would wrongly pick APE over the
    actual USB mic (card 2, "UGREEN USB MIC-CM564"). A deploy can always pin the
    device explicitly via the manifest's `mic_device` (e.g. plughw:2,0).
    """
    if devices is None:
        devices = scan_alsa_inputs()
    if not devices:
        return None
    usb = [d for d in devices
           if "usb" in d.name.lower() or "mic" in d.name.lower()]
    return usb[0] if usb else devices[0]
