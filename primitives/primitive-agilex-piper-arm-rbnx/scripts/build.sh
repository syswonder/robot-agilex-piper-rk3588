#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: colcon-build the vendored piper / piper_msgs / graspnet_msgs
# packages, then run rbnx codegen so atlas_bridge can import atlas_pb2 +
# lifecycle_pb2.
#
# Why graspnet_msgs is vendored alongside piper:
#   piper/package.xml lists `<depend>graspnet_msgs</depend>` because
#   graspnet_to_poscmd_node.py imports `graspnet_msgs/msg/GraspPose`.
#   Even though we don't run that node from this package (Stage 5
#   piper_moveit_rbnx will), colcon's build-time dep resolution still
#   needs the msgs available, otherwise `piper` won't build. graspnet_msgs
#   is tiny (one .msg) so no real cost.
#
# piper_humble (the MoveIt config in the upstream piper_ros workspace) is
# DELIBERATELY NOT vendored here — it's MoveIt-specific and belongs to
# Stage 5 (piper_moveit_rbnx). Keeping this package focused on hardware
# driver + msgs only.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[piper_ctl/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink the three vendored ROS source trees into rbnx-build/ws/src/
# so colcon picks them up. Symlink (not copy) keeps edits to src/ live
# without a rebuild dance — matches realsense / ranger_chassis pattern.
ln -snf "$PKG/src/piper"          "$PKG/rbnx-build/ws/src/piper"
ln -snf "$PKG/src/piper_msgs"     "$PKG/rbnx-build/ws/src/piper_msgs"
ln -snf "$PKG/src/graspnet_msgs"  "$PKG/rbnx-build/ws/src/graspnet_msgs"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[piper_ctl/build] colcon build (piper + piper_msgs + graspnet_msgs)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install \
    --packages-select piper piper_msgs graspnet_msgs \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[piper_ctl/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[piper_ctl/build] done."
