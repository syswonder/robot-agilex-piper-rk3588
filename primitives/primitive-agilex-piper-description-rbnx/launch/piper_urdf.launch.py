# SPDX-License-Identifier: MulanPSL-2.0
"""Piper arm URDF + robot_state_publisher launch.

Minimal viable replacement for the upstream piper_description
display launches:

    * loads urdf/piper_with_gripper/piper_description.urdf via
      `cat` (NOT xacro — the URDF is already pre-expanded by upstream)
    * remaps the RSP `joint_states` subscription to
      `/arm/joint_states_single` (what piper_ctl_rbnx actually
      publishes; see piper_ctrl_single_node.py:43 + the `/arm`
      namespace baked into start_single_piper.launch.py)
    * skips joint_state_publisher / joint_state_publisher_gui
      (the upstream display launch starts both for RViz sliders;
      they would clash with the real driver's joint feedback)
    * skips RViz (rbnx boot is headless; bring up RViz separately
      if you want to look at it)

Frame contract published by this launch:

    base_link ← arm mechanical root (NOT prefixed; see manifest
                 header re: future namespace collision with mobile
                 platforms' chassis base_link).
    link1..link6
                ← driven by /arm/joint_states_single positions
                  (joint1..joint6).
    gripper_base_link, link7, link8
                ← gripper geometry. The "gripper" name in JointState
                  is ignored by RSP (URDF uses joint7/joint8); finger
                  TFs stay at zero. grasp pipeline only uses link6,
                  so this is fine.

Camera hand-eye is NOT published here. The current vertical-grasp
deploy does not run easy_handeye2; yolo_grasp uses a calibrated 2D
homography file to map image pixels directly into arm/base_link XY.

Launch arguments:
    urdf_path           absolute path to piper_description.urdf.
                        Defaults to env PIPER_URDF_PATH if set
                        (atlas_register_and_launch.py exports it),
                        else the conventional vendored location.
    joint_states_topic  topic RSP subscribes to, default
                        `/arm/joint_states_single`.
"""
from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_urdf_path() -> str:
    """Resolve the vendored URDF path.

    Priority:
      1. PIPER_URDF_PATH env (set by atlas_register_and_launch.py
         based on cfg / RBNX_PACKAGE_ROOT)
      2. <pkg_root>/src/piper_description/urdf/piper_description.urdf
         (with-gripper variant, joint1..joint8).
    """
    env = os.environ.get("PIPER_URDF_PATH", "").strip()
    if env:
        return env
    pkg_root = Path(__file__).resolve().parent.parent
    return str(
        pkg_root
        / "src"
        / "piper_description"
        / "urdf"
        / "piper_description.urdf"
    )


def generate_launch_description() -> LaunchDescription:
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=_default_urdf_path(),
        description="Absolute path to piper_description.urdf",
    )
    js_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/arm/joint_states_single",
        description="Topic robot_state_publisher subscribes to",
    )

    # `Command(['cat', ...])` reads the URDF file at launch time; same
    # idiom upstream piper_description uses with `Command(['xacro', ...])`
    # except we skip the xacro pass (the vendored .urdf is already
    # pre-expanded from .xacro).
    robot_description = ParameterValue(
        Command(["cat ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="piper_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # Remap the RSP subscription from its default `joint_states`
        # to whatever piper_ctl_rbnx publishes. Without this, RSP
        # would never receive joint angles and TF would freeze at
        # zero.
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )

    return LaunchDescription([urdf_arg, js_arg, rsp_node])
