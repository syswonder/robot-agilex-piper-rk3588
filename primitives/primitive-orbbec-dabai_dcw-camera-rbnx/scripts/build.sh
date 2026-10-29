#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build phase: colcon-build the vendored OrbbecSDK_ROS2 packages, then
# run rbnx codegen so atlas_bridge can import atlas_pb2 + lifecycle_pb2.
#
# Vendored at src/OrbbecSDK_ROS2 — orbbec/OrbbecSDK_ROS2 upstream
# (last sync used in /Users/howenliu/lab/grasp/driver/OrbbecSDK_ROS2,
# Dabai DCW SDK shipped under orbbec_camera/SDK/). If anything diverges
# from upstream, drop a *.patch alongside src/ documenting the diff.
#
# Output goes into rbnx-build/{ws/install,codegen}/. start.sh sources
# rbnx-build/ws/install/setup.bash before launching atlas_bridge.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[orbbec_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink the vendored ROS source tree into rbnx-build/ws/src/ so colcon
# picks it up. Symlink (not copy) keeps edits to src/ live without a
# rebuild dance — matches realsense_camera_rbnx's pattern.
ln -snf "$PKG/src/OrbbecSDK_ROS2" "$PKG/rbnx-build/ws/src/OrbbecSDK_ROS2"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[orbbec_camera/build] colcon build (orbbec_camera + msgs + description)"
cd "$PKG/rbnx-build/ws"
# Match the upstream build.sh package selection. orbbec_description
# only ships URDF + meshes (no rclcpp_components_register_node), build
# is fast — keep it for completeness even though the static TFs are
# emitted by piper_description_rbnx in this deploy.
colcon build --symlink-install \
    --packages-select orbbec_camera_msgs orbbec_camera orbbec_description \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[orbbec_camera/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[orbbec_camera/build] done."
