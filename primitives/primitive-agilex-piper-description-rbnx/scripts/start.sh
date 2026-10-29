#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase. The script itself is now trivial — atlas registration,
# lifecycle Driver gRPC, heartbeat and signal forwarding are all owned
# by robonix_api.Primitive (see scripts/atlas_register_and_launch.py).
# Same pattern as primitive-robot-description-rbnx.
#
# What we still do here:
#   1. source ROS + the colcon overlay so `package://piper_description/...`
#      mesh URIs resolve at RViz / MoveIt time.
#   2. make robonix_api + generated atlas/contract stubs importable.
#   3. exec the python entrypoint (which calls cap.run()).
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Source the colcon-built overlay so `package://piper_description/...`
# mesh references in the URDF resolve correctly (RSP works without
# this — TF would still publish — but RViz / MoveIt won't render the
# arm).
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
fi

# Codegen output (atlas_pb2 + atlas_pb2_grpc + contracts stubs).
CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
CODEGEN_MCP="$PKG/rbnx-build/codegen/robonix_mcp_types"
if [[ ! -d "$CODEGEN_PROTO" ]]; then
    echo "[piper_description/start] ERR: codegen output missing at $CODEGEN_PROTO" >&2
    echo "[piper_description/start]      Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi

# robonix_api itself — resolved via `rbnx path robonix-api` (same as
# primitive-robot-description-rbnx). Fall back to just the codegen dirs
# if the CLI is missing so a standalone run still errors loudly on the
# import instead of on a shell command that can't be found.
if ROBONIX_API_PATH="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API_PATH:$CODEGEN_PROTO:$CODEGEN_MCP:${PYTHONPATH:-}"
else
    export PYTHONPATH="$CODEGEN_PROTO:$CODEGEN_MCP:${PYTHONPATH:-}"
fi

exec python3 -u "$PKG/scripts/atlas_register_and_launch.py"
