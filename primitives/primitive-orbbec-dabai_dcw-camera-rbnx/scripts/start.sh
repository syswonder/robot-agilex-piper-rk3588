#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ─── debug: source state at entry (we are RESPONSIBLE for sourcing our
# own colcon overlay — rbnx-cli no longer auto-sources package build
# outputs. AMENT_PREFIX_PATH should be empty/distro-only at entry; we
# add /opt/ros/humble + our rbnx-build/ws/install/ ourselves below).
echo "[orbbec_camera/start] PKG=$PKG" >&2
echo "[orbbec_camera/start] entry AMENT_PREFIX_PATH heads:" >&2
printf '  %s\n' ${AMENT_PREFIX_PATH//:/ } 2>&1 | sed -n '1,5p' >&2
echo "[orbbec_camera/start] entry: ros2 pkg prefix orbbec_camera => $(ros2 pkg prefix orbbec_camera 2>&1 || echo MISSING)" >&2
echo "[orbbec_camera/start] setup.bash exists? $(test -f "$PKG/rbnx-build/ws/install/setup.bash" && echo YES || echo NO)" >&2

# Source order matters:
#   1. /opt/ros/<distro>/setup.bash     — base ROS 2 distro
#   2. <pkg>/rbnx-build/ws/install/...  — our colcon-built overlay
#                                         (orbbec_camera + msgs).
# Each `source` appends to AMENT_PREFIX_PATH; later overlays win on
# package-name conflict, which is what we want (our vendored
# orbbec_camera must shadow any distro-shipped one).
ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
if [[ ! -f "$PKG/rbnx-build/ws/install/setup.bash" ]]; then
    echo "[orbbec_camera/start] FATAL: rbnx-build/ws/install/setup.bash missing" >&2
    echo "[orbbec_camera/start]        Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi
# Verify the actual subpackages exist in install/ — colcon's setup.bash
# records absolute build-time paths to per-package local_setup.bash and
# silently warns 'not found' if those subdirs are missing (e.g. partial
# rsync, or scripts/ was copied to a runtime cache without rbnx-build/).
# Make that failure explicit and fatal.
for sub in orbbec_camera_msgs orbbec_camera orbbec_description; do
    if [[ ! -f "$PKG/rbnx-build/ws/install/$sub/share/$sub/local_setup.bash" ]]; then
        echo "[orbbec_camera/start] FATAL: missing install for ROS package '$sub'" >&2
        echo "[orbbec_camera/start]        Expected: $PKG/rbnx-build/ws/install/$sub/share/$sub/local_setup.bash" >&2
        echo "[orbbec_camera/start]        rbnx-build/ exists but is incomplete — most likely you" >&2
        echo "[orbbec_camera/start]        rsynced source-only and skipped the build artifacts, OR" >&2
        echo "[orbbec_camera/start]        build.sh failed half-way. Re-run \`bash scripts/build.sh\`" >&2
        echo "[orbbec_camera/start]        in a CLEAN shell (unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH)." >&2
        exit 2
    fi
done

# Also scrub COLCON_PREFIX_PATH of obviously-stale entries (paths that
# don't exist on this host) before sourcing — otherwise the inner
# colcon setup.bash will spam 'not found' warnings that look like our
# bug. We keep entries that exist on disk.
if [[ -n "${COLCON_PREFIX_PATH:-}" ]]; then
    clean_cpp=""
    IFS=':' read -ra _cpp_parts <<< "$COLCON_PREFIX_PATH"
    for p in "${_cpp_parts[@]}"; do
        if [[ -n "$p" && -d "$p" ]]; then
            clean_cpp="${clean_cpp:+$clean_cpp:}$p"
        else
            echo "[orbbec_camera/start] scrubbing dead COLCON_PREFIX_PATH entry: $p" >&2
        fi
    done
    export COLCON_PREFIX_PATH="$clean_cpp"
fi

# shellcheck disable=SC1091
set +u; source "$PKG/rbnx-build/ws/install/setup.bash"; set -u

echo "[orbbec_camera/start] post-source: ros2 pkg prefix orbbec_camera => $(ros2 pkg prefix orbbec_camera 2>&1 || echo MISSING)" >&2

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
fi
# robonix_api auto-bootstraps codegen paths from the caller frame, but
# be explicit so a bare `python3 -m orbbec_camera.main` (without
# robonix_api's stack-walk fallback) still finds atlas_pb2.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

exec python3 -m orbbec_camera.main
