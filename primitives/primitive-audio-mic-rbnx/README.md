# Piper ALSA microphone

Microphone-only ALSA primitive for the Piper deployment. Provides gRPC
`robonix/primitive/audio/{driver,mic,list_devices,select_device}` contracts.
It does not provide speaker playback; use the audio client bridge for that.

Build with `bash scripts/build.sh`, start with `bash scripts/start.sh`, or use
the robot's root `robonix_manifest.yaml`. Requires Linux ALSA, `arecord`
(`alsa-utils`), Robonix and access to `/dev/snd`. The container recipe supplies
these dependencies.

Configure `mic_device` (empty for discovery, or `plughw:CARD,DEVICE`),
`mic_sample_rate` (16000), `mic_channels` (1), `mic_bits` (16), and
`mic_chunk_ms` (100) in the deployment's primitive config. When the audio
client bridge is also enabled, explicitly select the input provider to avoid
ambiguous microphone discovery.

Derived from the Robonix ALSA audio driver; provenance is in
`docs/UPSTREAMS.md` at the robot repository root. Software license:
MulanPSL-2.0.
