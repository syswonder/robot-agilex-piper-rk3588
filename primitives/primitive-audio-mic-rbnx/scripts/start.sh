#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Microphone primitive runtime. Uses `arecord` against the container's ALSA
# stack (host /proc/asound + /dev/snd bind-mounted by docker/run.sh) to expose
# robonix/primitive/audio/{mic,driver,list_devices,select_device} on atlas.
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# robonix_api ships as a source dir; expose it on PYTHONPATH so
# `from robonix_api import Primitive` resolves. The Primitive constructor
# then walks up to add rbnx-build/codegen/{proto_gen,robonix_mcp_types}.
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

# Don't redirect stdout/stderr — `rbnx boot` captures both into
# rbnx-boot/logs/*.log. A second redirect here would split the log.
exec python3 -m audio_mic.main
