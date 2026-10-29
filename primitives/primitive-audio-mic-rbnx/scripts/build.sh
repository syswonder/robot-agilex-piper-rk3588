#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Microphone primitive build phase.
#
# This primitive is venv-less: it runs on the container's system python3 and
# only needs the `robonix_api` source (added to PYTHONPATH at start) plus the
# codegen'd audio_pb2 stubs. Build = run codegen and verify the audio proto
# imports. The `arecord` CLI it shells out to is installed into the image by
# docker/Dockerfile (alsa-utils), not here — build.sh has no apt/root step.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

echo "[build] rbnx codegen (audio contracts → audio_pb2)"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Fail fast at build time if the generated audio stubs or robonix_api aren't
# importable, rather than degrading into a 60s Atlas registration timeout at
# boot. Mirror the runtime PYTHONPATH that start.sh sets.
CODEGEN="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types"
PYTHONPATH="$(rbnx path robonix-api):$CODEGEN:${PYTHONPATH:-}" python3 - <<'PY'
import audio_pb2
from robonix_api import Primitive
print("[build] audio_pb2 + robonix_api imports OK")
PY

# arecord is the capture backend. Warn (don't fail) if missing so a bare
# `rbnx build` on a host without alsa-utils still succeeds; the image provides
# it, and start.sh / init will surface a clear error if it's truly absent.
if ! command -v arecord >/dev/null 2>&1; then
    echo "[build] WARNING: 'arecord' not found on PATH — install alsa-utils (Dockerfile handles this in the container)." >&2
fi

echo "[build] done."
