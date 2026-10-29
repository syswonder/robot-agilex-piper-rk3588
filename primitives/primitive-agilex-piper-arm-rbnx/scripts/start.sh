#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# rbnx-cli does NOT auto-source package build outputs anymore — each
# package is responsible for sourcing its own colcon overlay here.
# Source the vendored piper / piper_msgs / graspnet_msgs we built in
# scripts/build.sh so `ros2 launch piper start_single_piper.launch.py`
# in on_activate can find them.
if [[ -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u
else
    echo "[piper_ctl/start] ERR: colcon overlay missing at $PKG/rbnx-build/ws/install/" >&2
    echo "[piper_ctl/start]      Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi
# robonix_api auto-bootstraps codegen paths from the caller frame, but
# be explicit so a bare `python3 -m piper_ctl.main` still finds atlas_pb2.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

exec python3 -m piper_ctl.main
