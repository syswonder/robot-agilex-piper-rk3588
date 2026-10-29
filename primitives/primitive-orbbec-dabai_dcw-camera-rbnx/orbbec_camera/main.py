#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""orbbec_camera_rbnx — Orbbec Dabai DCW RGBD primitive.

Owns `robonix/primitive/camera/*` for the piper_grasp deploy. Wraps
the upstream `orbbec_camera` ROS 2 driver (vendored under
src/OrbbecSDK_ROS2) and atlas-routes its rgb / depth / camera_info
streams.

Module-name disambiguation:
  - This Python module is `orbbec_camera` (lives at
    `<pkg_root>/orbbec_camera/`).
  - The vendored ROS 2 package is also called `orbbec_camera` but
    lives at `<pkg_root>/src/OrbbecSDK_ROS2/orbbec_camera/` and is
    only ever invoked via the `ros2 launch` subprocess we spawn —
    it is NOT on PYTHONPATH at the rbnx process level. So
    `python3 -m orbbec_camera.main` resolves to this file, not the
    ROS package. The collision is unfortunate but harmless.

Lifecycle (per Robonix developer guide §5):
    on_init       — light: validate cfg, derive topic names.
    on_activate   — heavy: spawn dabai_dcw.launch.py, wait for the
                    first RGB frame, declare rgb + depth +
                    camera_info ROS 2 topic_out on atlas. Failure
                    cleanly tears down the orbbec subprocess.
    on_deactivate — symmetric: kill orbbec subprocess.
    on_shutdown   — last-chance kill (idempotent w/ on_deactivate).

Multi-instance support:
    The atlas `provider_id` is read from the `RBNX_INSTANCE_NAME`
    environment variable that rbnx-cli sets per-instance (see
    developer guide §14.1). The same package can therefore be
    deployed multiple times with different manifest `name`s, e.g.
    `front_camera` + `arm_camera`, and each instance will register
    on atlas under its own provider_id without id collisions. When
    running multiple instances you MUST also give each one a
    distinct `camera_name` in its `config` block, otherwise the
    ROS 2 topics themselves (/<camera_name>/color/image_raw, ...)
    will collide inside the ROS graph — atlas isolation alone won't
    save you from a ROS-level topic clash.

    When `RBNX_INSTANCE_NAME` is unset (bare `python3 -m
    orbbec_camera.main` or older rbnx-cli versions), the
    provider_id falls back to the literal string "orbbec_camera",
    preserving the previous single-instance behaviour.

Config (from manifest's primitive[].config block, delivered via
Driver(CMD_INIT, config_json)):
    camera_name         default "camera"
    rgb_topic           default "/<camera_name>/color/image_raw"
    depth_topic         default "/<camera_name>/depth/image_raw"
    camera_info_topic   default "/<camera_name>/color/camera_info"
    depth_registration  default true   (HW-align depth to color frame)
    enable_d2c_viewer   default false  (debug RViz; off in deploy)
    color_width         default 640
    color_height        default 480
    color_fps           default 10
    depth_width         default 640
    depth_height        default 400
    depth_fps           default 10
    serial_number       default ""
    usb_port            default ""
    sentinel_timeout_s  default 30.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("ORBBEC_LOG_LEVEL", "INFO"),
    format="[orbbec] %(message)s",
)
log = logging.getLogger("orbbec")

# Provider id: read from RBNX_INSTANCE_NAME so the same package can be
# deployed as multiple instances (e.g. `front_camera` + `arm_camera`)
# without any code duplication. rbnx-cli injects this env var per
# instance right before spawning our start.sh (developer guide §14.1
# — Reusable packages should read RBNX_INSTANCE_NAME rather than
# hard-coding an id).
#
# Fallback "orbbec_camera" preserves the previous single-instance
# behaviour when the env var is not set (bare `python3 -m
# orbbec_camera.main` or older rbnx-cli versions). Manifests that
# only declare one instance can either use `- name: orbbec_camera`
# (matches the fallback) or any other name (env var wins).
_INSTANCE_ID = os.environ.get("RBNX_INSTANCE_NAME", "orbbec_camera")
log.info("provider_id resolved to %r (RBNX_INSTANCE_NAME=%r)",
         _INSTANCE_ID, os.environ.get("RBNX_INSTANCE_NAME"))
orbbec_camera = Primitive(
    id=_INSTANCE_ID,
    namespace="robonix/primitive/camera",
)

_pkg_root: Path = Path(__file__).resolve().parent.parent

# Subprocess + cached cfg. Allocated in on_activate, released in
# on_deactivate / on_shutdown. Module-level so the kill helper is
# reachable from every lifecycle callback.
_orbbec_proc: Optional[subprocess.Popen] = None
_resolved_cfg: Optional[dict[str, Any]] = None


def _bool_arg(v: Any) -> str:
    """Coerce truthy Python values into the lowercase 'true'/'false'
    strings that ros2 launch consumes via DeclareLaunchArgument."""
    return "true" if bool(v) else "false"


def _spawn_orbbec(cfg: dict) -> None:
    """Launch ros2 launch orbbec_camera dabai_dcw.launch.py with config args.

    Builds the argv from cfg; only forwards the keys we expose in the
    package manifest (everything else takes the launch file's own
    defaults). start_new_session=True so the whole process group can
    be torn down by signalling its PGID — matters because the launch
    spawns a ComposableNodeContainer, which spawns the actual driver,
    and a flat SIGTERM only kills the parent.
    """
    global _orbbec_proc
    cam = cfg.get("camera_name", "camera")
    # Launch file is config-selectable so one wrapper drives any Orbbec
    # model. Default keeps the original Dabai DCW; this deploy overrides
    # it to gemini_330_series.launch.py for the Gemini 336L.
    launch_file = str(cfg.get("launch_file", "dabai_dcw.launch.py"))
    args = [
        "ros2", "launch", "orbbec_camera", launch_file,
        f"camera_name:={cam}",
        f"enable_depth:={_bool_arg(cfg.get('enable_depth', True))}",
        f"depth_registration:={_bool_arg(cfg.get('depth_registration', True))}",
        f"enable_d2c_viewer:={_bool_arg(cfg.get('enable_d2c_viewer', False))}",
        f"color_width:={int(cfg.get('color_width', 640))}",
        f"color_height:={int(cfg.get('color_height', 480))}",
        f"color_fps:={int(cfg.get('color_fps', 10))}",
        f"color_format:={str(cfg.get('color_format', 'MJPG'))}",
        f"depth_width:={int(cfg.get('depth_width', 640))}",
        f"depth_height:={int(cfg.get('depth_height', 400))}",
        f"depth_fps:={int(cfg.get('depth_fps', 10))}",
    ]
    # Optional pin-by-USB args — only forward when the user gave a
    # non-empty value, otherwise let dabai_dcw.launch.py keep its
    # defaults (empty == "any device").
    if cfg.get("serial_number"):
        args.append(f"serial_number:={cfg['serial_number']}")
    if cfg.get("usb_port"):
        args.append(f"usb_port:={cfg['usb_port']}")

    log_path = _pkg_root / "rbnx-build" / "data" / "orbbec.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    log.info("spawning orbbec (cam=%s) → %s", cam, log_path)
    log.debug("launch args: %s", " ".join(args))
    _orbbec_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_orbbec() -> None:
    """Tear down the launched ros2 process group. Idempotent — safe
    to call from on_deactivate followed by on_shutdown without
    raising on the second call."""
    global _orbbec_proc
    p = _orbbec_proc
    if p is None or p.poll() is not None:
        _orbbec_proc = None
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        _orbbec_proc = None
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    _orbbec_proc = None


def _wait_for_image(topic: str, timeout_s: float) -> bool:
    """Spin up a one-shot rclpy node, subscribe to `topic`, return
    True when the first sensor_msgs/Image arrives within timeout.

    Why best_effort + volatile: that's what the upstream orbbec_camera
    driver publishes Image at; using reliable here would just silently
    never receive frames. Mirrors mid360_imu/main.py::_wait_for_imu's
    pattern."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Image
    except ImportError as e:
        log.warning("rclpy unavailable (%s); skipping sentinel wait", e)
        return True
    rclpy.init(args=None)
    node = Node("orbbec_atlas_sentinel")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    seen = threading.Event()
    node.create_subscription(Image, topic, lambda _m: seen.set(), qos)
    log.info("waiting for first frame on %s — up to %.1fs", topic, timeout_s)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if seen.is_set():
                break
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    return seen.is_set()


# ── lifecycle handlers ───────────────────────────────────────────────────
@orbbec_camera.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Validate cfg + cache for activate.

    Light only — DO NOT spawn ros2, DO NOT declare on atlas. Heavy
    work belongs in on_activate so a CMD_DEACTIVATE → CMD_ACTIVATE
    re-cycle works without a half-baked init side effect."""
    global _resolved_cfg
    cfg = cfg or {}
    try:
        sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
        if sentinel_timeout <= 0:
            return Err(f"sentinel_timeout_s must be > 0, got {sentinel_timeout}")
    except (TypeError, ValueError) as e:
        return Err(f"sentinel_timeout_s not numeric: {e}")
    _resolved_cfg = dict(cfg)
    log.info("CMD_INIT ok (camera_name=%s, depth_registration=%s)",
             cfg.get("camera_name", "camera"),
             cfg.get("depth_registration", True))
    return Ok()


@orbbec_camera.on_activate
def activate():
    """INACTIVE → ACTIVE. Spawn dabai_dcw.launch.py, wait for the
    first RGB frame as proof the pipeline is live, then atlas-declare
    the three topic_out streams.

    On any failure between spawn and declare, the orbbec subprocess
    is torn down before returning Err so the next CMD_ACTIVATE starts
    from a clean state."""
    cfg = _resolved_cfg or {}
    cam = str(cfg.get("camera_name", "camera"))
    rgb_topic = str(cfg.get("rgb_topic", f"/{cam}/color/image_raw"))
    depth_topic = str(cfg.get("depth_topic", f"/{cam}/depth/image_raw"))
    camera_info_topic = str(cfg.get(
        "camera_info_topic", f"/{cam}/color/camera_info",
    ))
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))

    try:
        _spawn_orbbec(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn orbbec failed: {e}")

    if not _wait_for_image(rgb_topic, sentinel_timeout):
        _kill_orbbec()
        return Err(
            f"no Image on {rgb_topic} within {sentinel_timeout:.1f}s "
            f"(check rbnx-build/data/orbbec.log; USB attached? udev?)"
        )

    # Frame-name in the description is the value `depth_registration:=true`
    # makes effective: with HW alignment on, depth pixels share the
    # color stream's optical frame, which downstream geometry (yolo_grasp's
    # back-projection, scene 3D fusion) depends on.
    color_frame = f"{cam}_color_optical_frame"
    try:
        orbbec_camera.declare_ros2_topic(
            "robonix/primitive/camera/rgb",
            topic=rgb_topic,
            qos="best_effort",
            description=(
                f"Orbbec Dabai DCW color stream (sensor_msgs/Image, frame: "
                f"{color_frame})."
            ),
        )
        orbbec_camera.declare_ros2_topic(
            "robonix/primitive/camera/depth",
            topic=depth_topic,
            qos="best_effort",
            description=(
                f"Orbbec Dabai DCW depth stream HW-aligned to color "
                f"(sensor_msgs/Image, frame: {color_frame}, "
                f"depth_registration=true). Pixel-corresponds 1:1 with "
                f"the rgb stream — yolo_grasp uses this for 3D back-projection."
            ),
        )
        orbbec_camera.declare_ros2_topic(
            "robonix/primitive/camera/camera_info",
            topic=camera_info_topic,
            qos="best_effort",
            description=(
                f"Color stream intrinsics (sensor_msgs/CameraInfo, frame: "
                f"{color_frame}). fx/fy/cx/cy + distortion model — drive "
                f"the back-projection in yolo_grasp from 2D bbox center to "
                f"3D point in the camera frame."
            ),
        )
    except Exception as e:  # noqa: BLE001
        _kill_orbbec()
        return Err(f"declare_ros2_topic failed: {e}")

    log.info("CMD_ACTIVATE ok: rgb=%s depth=%s camera_info=%s",
             rgb_topic, depth_topic, camera_info_topic)
    return Ok()


@orbbec_camera.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Kill the orbbec subprocess. Idempotent."""
    _kill_orbbec()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


@orbbec_camera.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance kill. Idempotent w/ on_deactivate."""
    _kill_orbbec()
    return Ok()


if __name__ == "__main__":
    orbbec_camera.run()
