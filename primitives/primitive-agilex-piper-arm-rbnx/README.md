# primitive-agilex-piper-arm-rbnx

Robonix package wrapping the **AgileX Piper 6-DoF arm** hardware driver. Owns the `primitive/arm/*` namespace.

Catalog name: `robonix.primitive.agilex.piper.arm`.

> Naming note: internal directory / binary name is `piper_ctl` (`_ctl` = low-level control), while the contract namespace is `arm`. Piper is an arm, so calling this `piper_chassis_rbnx` would mislead.

## Capability surface

| Contract                                | Mode       | Transport | Source / handler                                     |
| --------------------------------------- | ---------- | --------- | ---------------------------------------------------- |
| `robonix/primitive/arm/driver`          | rpc        | gRPC      | `Driver(CMD_INIT, config_json)` — lifecycle gate     |
| `robonix/primitive/arm/joint_states`    | topic_out  | ROS 2     | `/<ns>/joint_states_single` (sensor_msgs/JointState) |
| `robonix/primitive/arm/joint_command`   | topic_in   | ROS 2     | `/<ns>/joint_command` (sensor_msgs/JointState) — driver subscribes; consumers publish |
| `robonix/primitive/arm/end_pose`        | topic_out  | ROS 2     | `/<ns>/end_pose` (geometry_msgs/Pose)                |
| `robonix/primitive/arm/pos_command`     | topic_in   | ROS 2     | `/<ns>/pos_command` (geometry_msgs/Pose) — driver subscribes; consumers publish |
| `robonix/primitive/arm/arm_status`      | topic_out  | ROS 2     | `/<ns>/arm_status` (piper_msgs/PiperStatusMsg) — vendor extension |

The FOUR **standard** contracts (driver / joint_states / joint_command / end_pose / pos_command) mirror robonix's global `capabilities/primitive/arm/*.v1.toml`: same id, version, IDL, and mode. Keep them byte-compatible with the global tree so a future consolidation is a no-op.

`arm_status` is a Piper-specific **vendor extension** and remains package-local. Its IDL `piper_msgs/PiperStatusMsg.msg` is shipped at package level under `capabilities/lib/piper_msgs/msg/PiperStatusMsg.msg`.

> **Single-source-of-truth** for the `PiperStatusMsg` IDL is the vendored `src/piper_msgs/msg/` directory. The mirrored copy under `capabilities/lib/piper_msgs/msg/` is what `rbnx codegen` / atlas's contract registry actually scan. Keep the two in sync — plain file copies are used rather than symlinks to keep `git diff` legible.

### Joint-space vs Cartesian control paths

Two independent command channels; each one bypasses the other:

- **Joint-space (default in the vertical-grasp deploy)**: consumers publish `sensor_msgs/JointState` on `arm/joint_command`. `roboarm_ik` already solves IK, so the driver just forwards the target angles to the Piper SDK's `JointCtrl` + `GripperCtrl`. The `gripper` entry in the JointState carries finger opening.
- **Cartesian**: consumers publish `geometry_msgs/Pose` on `arm/pos_command`. The driver converts the quaternion to xyz-euler and forwards to the Piper SDK's `EndPoseCtrl` (SDK-side / firmware IK). **Gripper is NOT part of this contract** — command it separately through `joint_command`.

## Boot ordering

Boot this **before** any consumer of `primitive/arm/*`. In the vertical-grasp pipeline:

- `primitive-agilex-piper-description-rbnx` consumes `arm/joint_states` to drive `robot_state_publisher`;
- `service-roboarm-ik-rbnx` consumes `arm/joint_states` and publishes to `arm/joint_command` (default path);
- `service-piper-moveit-rbnx` consumes `arm/arm_status` and publishes to `arm/pos_command` (Cartesian path);
- `skill-pick-vertical-grasp-rbnx` polls `arm/arm_status` between grasps and monitors `/arm/joint_states_single` to verify that the gripper is holding an object.

rbnx-cli has no defer/retry, so providers MUST come first in YAML declaration order.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge — no ROS spawn. The bridge opens a gRPC server, registers the provider, declares only `primitive/arm/driver` (auto-emitted by the framework when codegen produces a `Driver` Servicer), then blocks on `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The lifecycle then runs:

1. `CMD_INIT`: validate cfg (CAN port, bitrate, gripper flags, sentinel timeout);
2. `CMD_ACTIVATE`: optionally run `scripts/can_activate.sh` when `auto_can_setup=true`;
3. spawn `ros2 launch piper start_single_piper.launch.py …`;
4. wait for the first `sensor_msgs/JointState` on `/<ns>/joint_states_single` as proof the CAN link came up;
5. declare `arm/joint_states`, `arm/joint_command`, `arm/end_pose`, `arm/pos_command`, and `arm/arm_status` on atlas, then return ok.

`CMD_DEACTIVATE` / `CMD_SHUTDOWN` kill the piper subprocess. Idempotent.

## CAN bring-up — TWO paths

The Piper arm uses USB-CAN (MCP251xFD or similar). The CAN interface must be **up** and named `can_piper` (or whatever you set `can_port:` to) before `piper_ctrl_single_node` can talk to it. The CAN interface name is config-driven — the package never hardcodes a device name.

If the configured USB address or interface name looks wrong, inspect
the current host mapping first:

```bash
bash scripts/find_all_can_port.sh
```

Use the reported USB bus path, for example `1-4.2:1.0`, as
`can_usb_address`. Do not guess when multiple CAN interfaces are
present; Jetson onboard `*.mttcan` interfaces and USB-CAN adapters can
coexist.

### Path A — recommended for production (default)

Bring `can_piper` up **outside `rbnx boot`**, ahead of time. On a Jetson with udev rules, this is once-per-boot:

```bash
# Run once per host boot, BEFORE `rbnx boot`:
bash scripts/can_activate.sh can_piper 1000000 "1-4.2:1.0"
# Replace "1-4.2:1.0" with the USB bus path of your Piper.
# To list candidates:
#   lsusb -t
#   sudo ethtool -i can0 | grep bus-info
```

`auto_can_setup` stays `false` in the manifest. This path keeps the deploy free of sudo coupling — the operator approves the sudo prompt once, interactively, then `rbnx boot` runs unattended.

### Path B — convenience for dev laptops

Set `auto_can_setup: true` in the manifest config block. `on_activate` will run `scripts/can_activate.sh` itself before spawning the driver. Requires **passwordless sudo** for the operator (otherwise the script blocks on the prompt and `CMD_ACTIVATE` times out).

```yaml
- name: piper_ctl
  path: ../packages/piper_ctl_rbnx
  config:
    can_port:        can_piper
    can_bitrate:     1000000
    can_usb_address: "1-4.2:1.0"
    auto_can_setup:  true
```

Path B is more convenient when the USB-CAN's bus path shifts between sessions, but couples the deploy to sudoers state — not ideal for production.

## Layout

```
primitive-agilex-piper-arm-rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── primitive/arm/
│   │   ├── driver.v1.toml
│   │   ├── joint_states.v1.toml
│   │   ├── joint_command.v1.toml
│   │   ├── end_pose.v1.toml
│   │   ├── pos_command.v1.toml
│   │   └── arm_status.v1.toml               # Piper vendor extension
│   └── lib/piper_msgs/msg/                  # IDL for codegen
│       └── PiperStatusMsg.msg               # mirror of src/piper_msgs/msg/
├── piper_ctl/
│   ├── __init__.py
│   └── main.py                              # lifecycle + sentinel
├── scripts/
│   ├── build.sh                             # colcon + rbnx codegen
│   ├── start.sh                             # source ROS, exec main
│   ├── find_all_can_port.sh                 # print CAN interface ↔ USB bus mapping
│   └── can_activate.sh                      # vendored from upstream piper_ros
└── src/                                     # vendored (no .git, no build/install)
    ├── piper/                               # main ROS 2 driver (rclpy)
    ├── piper_msgs/                          # PiperStatusMsg + Enable.srv (PosCmd kept only for the legacy graspnet_to_poscmd_node)
    └── graspnet_msgs/                       # GraspPose.msg — required by piper/package.xml
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```yaml
can_port:            can_piper        # CAN interface name
can_bitrate:         1000000
can_usb_address:     ""               # pin via USB bus path; "" = first detected
auto_can_setup:      false            # see "CAN bring-up" above
auto_enable:         true             # forwarded to start_single_piper auto_enable
gripper_exist:       true
gripper_val_mutiple: 2                # upstream typo preserved
arm_namespace:       /arm             # see note in main.py docstring
sentinel_timeout_s:  30.0             # max wait for first JointState in on_activate
# Topic-name overrides (rarely needed; default derives from arm_namespace):
# joint_states_topic:   /arm/joint_states_single
# joint_command_topic:  /arm/joint_command
# arm_status_topic:     /arm/arm_status
# end_pose_topic:       /arm/end_pose
# pos_command_topic:    /arm/pos_command
```

## Build / run standalone

```bash
bash scripts/build.sh                                  # colcon + rbnx codegen
ROBONIX_ATLAS=127.0.0.1:50051 \
    bash scripts/start.sh                              # registers, awaits Init
```

To drive the lifecycle manually (without `rbnx boot`), call the arm's `Driver` service with `CMD_INIT` and a JSON config blob, then call `CMD_ACTIVATE`. Init only validates configuration; Activate returns after the first JointState is observed and the five data topics are declared.

## Verification

```bash
rbnx caps | grep arm
# Expected: piper_ctl provider with
#   robonix/primitive/arm/{driver, joint_states, joint_command,
#                          end_pose, pos_command, arm_status}

ros2 topic hz /arm/joint_states_single             # ~200 Hz
ros2 topic echo /arm/arm_status --once             # PiperStatusMsg fields
ros2 topic echo /arm/end_pose --once               # geometry_msgs/Pose

# Cartesian cmd path (CAREFUL — moves the arm; clear the workspace first):
ros2 topic pub --once /arm/pos_command geometry_msgs/msg/Pose \
    "{position: {x: 0.30, y: 0.0, z: 0.25},
      orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}"

# Joint-space cmd path (also moves the arm — same warning):
ros2 topic pub --once /arm/joint_command sensor_msgs/msg/JointState \
    "{name: [joint1, joint2, joint3, joint4, joint5, joint6, gripper],
      position: [0.0, 0.5, -0.7, 0.0, 0.6, 0.0, 0.04],
      velocity: [], effort: []}"
```

## Vendor / upstream

`src/piper/`, `src/piper_msgs/`, `src/graspnet_msgs/` are verbatim copies from [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros). The other packages in the upstream workspace are deliberately **not** vendored here and land in their own robonix packages:

- `piper_description` / `piper_with_gripper_moveit` → `primitive-agilex-piper-description-rbnx`.
- `piper_humble` / `piper_moveit_control` → `service-piper-moveit-rbnx`.
- `piper_gazebo` / `piper_mujoco` / `piper_sim` → not migrated (sim-only).

If anything diverges from upstream, drop a `*.patch` alongside `src/` documenting the diff.

## License

This package: Apache-2.0. Vendored piper_ros / piper_msgs / graspnet_msgs: see their respective LICENSE files.
