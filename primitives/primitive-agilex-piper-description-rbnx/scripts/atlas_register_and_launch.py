#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""piper_description_rbnx — robonix_api Primitive wrapper for the Piper URDF + robot_state_publisher.

Same pattern as `primitive-robot-description-rbnx` (which uses the new
`robonix_api.Primitive` + `@cap.on_init` / `@cap.on_shutdown` lifecycle)
— we differ only in the URDF source. This package publishes the AgileX
Piper arm URDF vendored under `src/piper_description/urdf/` via
`ros2 launch piper_urdf.launch.py`; we do NOT fetch a URDF from soma.

The framework (robonix_api) takes care of:
  * atlas RegisterPrimitive + heartbeat
  * Driver(CMD_INIT / CMD_ACTIVATE / CMD_DEACTIVATE / CMD_SHUTDOWN) gRPC servicer
  * SIGTERM / SIGINT signal handling
  * Lifecycle state pushed back to atlas / rbnx-boot's status view

This script only owns the two things that are Piper-specific:
  1. locate the URDF file (`_resolve_urdf_path`)
  2. spawn / stop the `ros2 launch` subprocess

Required PYTHONPATH (set by start.sh):
    rbnx-build/codegen/proto_gen              — atlas_pb2 / atlas_pb2_grpc
    rbnx-build/codegen/robonix_mcp_types      — contracts stubs
    <robonix-api site-packages>               — robonix_api itself
    /opt/ros/humble/lib/python*/...           — implicit via ros2 launch's own env
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
from pathlib import Path

from robonix_api import Err, Ok, Primitive


logging.basicConfig(level=logging.INFO, format="[piper_description] %(message)s")
log = logging.getLogger("piper_description")


PROVIDER_ID = os.environ.get("RBNX_INSTANCE_NAME", "piper_description")
NAMESPACE = "robonix/primitive/robot_description"

cap = Primitive(id=PROVIDER_ID, namespace=NAMESPACE)

# Track the ros2 launch process across on_init / on_shutdown. Kept at
# module scope (not on `cap`) because the underlying subprocess handle
# is a plain Popen — same pattern as robot_description/main.py.
_launch_proc: subprocess.Popen[bytes] | None = None


def _package_root() -> Path:
    """Resolve the package root. RBNX sets RBNX_PACKAGE_ROOT for us; the
    fallback keeps `python3 -m` invocations from a shell working too."""
    return Path(
        os.environ.get(
            "RBNX_PACKAGE_ROOT",
            str(Path(__file__).resolve().parent.parent),
        )
    )


def _resolve_urdf_path(pkg_root: Path) -> Path:
    """Resolve which Piper URDF to load.

    Default: vendored `urdf/piper_description.urdf` (with-gripper
    variant — upstream's pre-expanded URDF, joints joint1..joint8
    where joint7/joint8 are the two gripper fingers).

    Override with `PIPER_URDF_PATH` env to switch to
    `urdf/piper_no_gripper_description.urdf` (joint1..joint6 only)
    or to a forked URDF for moving-platform deploys that need a
    tf_prefix.
    """
    explicit = os.environ.get("PIPER_URDF_PATH", "").strip()
    if explicit:
        return Path(explicit)
    return pkg_root / "src" / "piper_description" / "urdf" / "piper_description.urdf"


def _resolve_launch_file(pkg_root: Path) -> Path:
    return pkg_root / "launch" / "piper_urdf.launch.py"


def _spawn_launch(launch_file: Path, urdf_path: Path) -> subprocess.Popen[bytes]:
    """Spawn `ros2 launch <launch_file>` in its own process group so we
    can SIGTERM the whole tree (launch + robot_state_publisher) on
    shutdown.

    Pass urdf_path through env (PIPER_URDF_PATH) so the .launch.py
    picks it up via DeclareLaunchArgument's default. Avoids quoting
    headaches that would arise from an `urdf_path:=...` cmdline arg
    holding an absolute path with shell-special chars.
    """
    log.info("spawning ros2 launch %s", launch_file)
    log.info("PIPER_URDF_PATH=%s", urdf_path)
    env = os.environ.copy()
    env["PIPER_URDF_PATH"] = str(urdf_path)
    return subprocess.Popen(
        ["ros2", "launch", str(launch_file)],
        env=env,
        # New session → killpg(getpgid(child)) reaches every descendant.
        start_new_session=True,
    )


def _stop_launch(proc: subprocess.Popen[bytes] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        log.warning("ros2 launch did not exit within 10s; sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


@cap.on_init
def init(_cfg: dict):
    """Bring up robot_state_publisher via ros2 launch.

    The per-package `config:` block in the deploy manifest is
    intentionally ignored: this package has no CMD_INIT-time knobs;
    configuration is env-based (`PIPER_URDF_PATH`). See README.
    """
    global _launch_proc
    if _launch_proc is not None and _launch_proc.poll() is None:
        return Ok()

    pkg_root = _package_root()
    launch_file = _resolve_launch_file(pkg_root)
    if not launch_file.is_file():
        return Err(f"launch file missing: {launch_file}")

    urdf_path = _resolve_urdf_path(pkg_root)
    if not urdf_path.is_file():
        return Err(
            "URDF missing: "
            f"{urdf_path} "
            "(set PIPER_URDF_PATH or place piper_description.urdf under "
            "src/piper_description/urdf/)"
        )

    try:
        _launch_proc = _spawn_launch(launch_file, urdf_path)
    except FileNotFoundError as exc:
        return Err(f"ros2 launch not on PATH: {exc}")
    except Exception as exc:  # noqa: BLE001
        return Err(f"failed to spawn ros2 launch: {exc}")

    # Quick liveness check — matches robot_description/main.py's pattern.
    # Don't wait long: robot_state_publisher publishes /tf_static almost
    # immediately, and stalling on_init just delays soma stage 1.
    try:
        rc = _launch_proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        return Ok()
    else:
        _launch_proc = None
        return Err(f"ros2 launch exited immediately with rc={rc}")


@cap.on_shutdown
def shutdown():
    global _launch_proc
    _stop_launch(_launch_proc)
    _launch_proc = None
    return Ok()


if __name__ == "__main__":
    cap.run()
