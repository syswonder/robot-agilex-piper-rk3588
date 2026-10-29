# SPDX-License-Identifier: MulanPSL-2.0
"""运动：技能能动臂、能动能夹爪的全部动作，都在 `Arm` 这一个对象上。

和上游服务端的分层一一对应（服务端内部也正好是这么分的）：

    arm.move_pose(pose, gripper_width, timeout)   笛卡尔位姿 → IK → 移动 + 夹爪
                                                  （走 execute_grasp 契约）
    arm.go_observe()                              观察位（关节角在服务端配置里）
    arm.go_teach_safe(hold_gripper=False)         安全位（同上，可选保持握持）
    arm.open_gripper_in_place()                   只动夹爪、关节一个不动

前三个走 gRPC 契约，最后一个直接发 `/arm/joint_command` 话题——因为它要保证
"零臂运动"：用契约表达"移动到当前位姿"必须先知道当前末端位姿（技能侧没有正向
运动学），而且 IK 解回来的关节角万一和当前差一点，臂就会微动。

**为什么是类**：`Arm` 一个实例持有整套状态——ROS 节点、joint_command 发布器、
关节/夹爪反馈缓存、上游端点、夹爪配置。技能在 init 里造它、activate 里 `start()`、
deactivate 里 `shutdown()`：状态和生命周期绑在一起，不会有"上一次运行的残留"，
也不用靠调用顺序维持一堆模块级全局量的不变式。测试时注入假的 `Clients` 就能跑，
不需要 ROS。

**分层纪律**：编排层（atlas_bridge）只调 `Arm` 的公开方法；`Arm` 只调 `Clients`
与 ROS 话题。谁都不许跳层。

**错误约定**：失败一律抛 `MotionError`。
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

from .config import num, text
from .grpc_clients import Clients, GrpcCallError

log = logging.getLogger("pick_skill.motion")


class MotionError(RuntimeError):
    """一次臂/夹爪动作失败（上游拒绝、夹爪没夹住、反馈缺失）。"""


@dataclass
class GripperConfig:
    """夹爪与反馈的配置（由编排层在 init 时填好）。"""

    open_width: float = 0.08          # 张开宽度（米）
    close_width: float = 0.0          # 闭合宽度（米）
    grasp_threshold_width: float = 0.005   # 低于它 = 夹空了
    feedback_timeout_s: float = 2.0
    val_multiple: float = 2.0         # piper_ctl 会把 joint_command 再乘回去
    command_effort: float = 3.0
    # 话题名不写死：正常由 atlas 解析 `robonix/primitive/arm/{joint_states,
    # joint_command}` 得到；要显式指定就配 gripper_*_topic。都没有则激活失败。
    joint_states_topic: str = ""
    joint_command_topic: str = ""

    @classmethod
    def from_cfg(cls, cfg: dict) -> "GripperConfig":
        """按 manifest 的 config 块建配置；缺项用默认值，坏值告警后回落。"""
        return cls(
            open_width=num(cfg, "gripper_open_width", 0.08, lo=0.0),
            close_width=num(cfg, "gripper_close_width", 0.0, lo=0.0),
            grasp_threshold_width=num(cfg, "gripper_grasp_threshold_width", 0.005, lo=0.0),
            feedback_timeout_s=num(cfg, "gripper_feedback_timeout_s", 2.0, lo=0.1),
            # 话题默认空：正常由 atlas 解析，只有显式配置才写进这里（见上）
            joint_states_topic=text(cfg, "gripper_joint_states_topic"),
        )


# 一次移动的每段最小预算（秒）：宁可多给一点，也不让服务端拿 0 秒去跑余量算法。
MIN_SEGMENT_S = 2.0


def qos_from_profile(profile: str):
    """把通道声明里的 QoS 名转成 rclpy QoSProfile。

    相机这类高频话题对端多声明 best_effort；用错方向会直接收不到数据
    （reliable 订阅者接不上 best_effort 发布者），所以按声明来、不猜。
    """
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    best_effort = str(profile or "").strip().lower() in ("best_effort", "besteffort")
    return QoSProfile(
        depth=5,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT if best_effort
        else ReliabilityPolicy.RELIABLE,
    )


def _tag(stage: str) -> str:
    """日志前缀：有步骤名就显示 `[步骤名] `。"""
    return f"[{stage}] " if stage else ""


def _fail(stage: str, message: str) -> MotionError:
    """拼失败文案：有步骤名则 `‹步骤名› failed: ‹原因›`，否则就是原因本身。"""
    return MotionError(f"{stage} failed: {message}" if stage else message)


def segment_budget(total_s: float, segments: int = 3) -> float:
    """把一段动作的总预算切成 n 段，每段不低于 MIN_SEGMENT_S。

    调用方负责传入"剩余时间"（自己算 deadline），本函数只管均分与下限。
    """
    return max(MIN_SEGMENT_S, float(total_s) / max(1, int(segments)))


def _joint_state_gripper_width(msg: Any) -> Optional[float]:
    """从 JointState 取夹爪实际开度（米）。

    piper_ctl 发 /arm/joint_states_single，名字是 joint1..joint6 + gripper，
    position[6] 就是夹爪开度。joint7 是 MoveIt 时代的叫法，一并接受。
    """
    names = list(getattr(msg, "name", []) or [])
    positions = list(getattr(msg, "position", []) or [])
    if not positions:
        return None
    idx: Optional[int] = None
    for candidate in ("gripper", "joint7"):
        if candidate in names:
            idx = names.index(candidate)
            break
    if idx is None and len(positions) >= 7:
        idx = 6
    if idx is None or idx >= len(positions):
        return None
    try:
        width = float(positions[idx])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(width):
        return None
    return max(0.0, width)


def _joint_state_arm_joints(msg: Any) -> Optional[list[float]]:
    """从同一个 JointState 取 6 关节向量（弧度）。

    六个具名关节不齐或出现非有限值就返回 None——绝不猜角度：open_gripper_in_place
    会原样回放这些值，猜错就是把臂驱动起来。
    """
    names = list(getattr(msg, "name", []) or [])
    positions = list(getattr(msg, "position", []) or [])
    out: list[float] = []
    for j in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
        if j not in names:
            return None
        idx = names.index(j)
        if idx >= len(positions):
            return None
        try:
            v = float(positions[idx])
        except (TypeError, ValueError):
            return None
        if not math.isfinite(v):
            return None
        out.append(v)
    return out


class Arm:
    """臂与夹爪。一个实例 = 一套 ROS 连接、反馈缓存和上游端点。"""

    def __init__(self, clients: Clients, gripper: Optional[GripperConfig] = None,
                 endpoints: Optional[dict[str, str]] = None,
                 camera: Any = None) -> None:
        """clients 由调用方提供（同一个传输层也服务感知调用）；camera 非空时
        `start()` 会在 ROS 节点上挂上它的图像订阅。"""
        self.clients = clients
        self.gripper = gripper or GripperConfig()
        self._endpoints: dict[str, str] = {str(k): str(v)
                                           for k, v in (endpoints or {}).items()}
        self._camera = camera
        self._node: Any = None
        self._pub: Any = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._width: Optional[float] = None
        self._width_mono: Optional[float] = None
        self._joints: Optional[list[float]] = None
        self._joints_mono: Optional[float] = None
        # 原语话题的 QoS（由编排层经 atlas 解析后 set_topics 填；默认 best_effort）
        self._js_qos: str = "best_effort"
        self._jc_qos: str = "best_effort"

    # ── 配置与生命周期 ──────────────────────────────────────────────────
    def set_endpoints(self, endpoints: Optional[dict[str, str]]) -> None:
        """设置/清空上游端点（技能 activate / deactivate 时调）。"""
        self._endpoints = {str(k): str(v) for k, v in (endpoints or {}).items()}

    def set_topics(self, joint_states: tuple, joint_command: tuple) -> None:
        """设置（经 atlas 解析出来的）原语话题与 QoS。

        每个都是 (topic, qos_profile)；编排层解析 `robonix/primitive/arm/*` 契约
        后传进来，解析不到就传配置兜底值。必须在 start() 之前调。
        """
        (self.gripper.joint_states_topic, self._js_qos) = joint_states
        (self.gripper.joint_command_topic, self._jc_qos) = joint_command

    def start(self) -> None:
        """起 ROS 节点：订阅关节反馈、建 joint_command 发布器。

        camera 非空时同时把图像订阅挂上（`camera.attach(node)`）。ROS 不可用时
        只告警不抛——动作会在需要反馈时失败。
        """
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
        except Exception as e:  # noqa: BLE001
            log.warning("ROS bridge unavailable: %s", e)
            return

        self._stop.clear()
        arm = self

        class _RosBridge(Node):
            def __init__(self) -> None:
                super().__init__("pick_skill_ros_bridge")
                # 话题名与 QoS 都来自 atlas 解析的 `robonix/primitive/arm/*` 契约
                # （见 set_topics）；解析不到且没配兜底时**不订阅/不发布**——宁可
                # 动作时报错，也不去猜一个话题静默失联。
                if arm.gripper.joint_states_topic:
                    self.create_subscription(
                        JointState, arm.gripper.joint_states_topic,
                        self._on_joint_state, qos_from_profile(arm._js_qos))
                else:
                    log.error("arm joint_states topic unresolved (atlas) and not "
                              "configured — gripper feedback will be unavailable")
                if arm.gripper.joint_command_topic:
                    arm._pub = self.create_publisher(
                        JointState, arm.gripper.joint_command_topic,
                        qos_from_profile(arm._jc_qos))
                else:
                    log.error("arm joint_command topic unresolved (atlas) and not "
                              "configured — in-place gripper moves will be refused")
                self.camera_ok = bool(arm._camera.attach(self)) \
                    if arm._camera is not None else False

            def _on_joint_state(self, msg: Any) -> None:
                arm._record_joint_state(msg)

        def _runner() -> None:
            try:
                if not rclpy.ok():
                    rclpy.init(args=None)
                node = _RosBridge()
                self._node = node
                log.info("ROS bridge up: joint feedback %s, command %s (camera=%s)",
                         self.gripper.joint_states_topic,
                         self.gripper.joint_command_topic,
                         "attached" if getattr(node, "camera_ok", False) else "none")
                while rclpy.ok() and not self._stop.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)
                node.destroy_node()
            except Exception as e:  # noqa: BLE001
                log.warning("ROS bridge stopped: %s", e)
            finally:
                self._pub = None
                self._node = None

        self._thread = threading.Thread(
            target=_runner, name="pick-skill-ros", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """停 ROS 节点（deactivate 时调）。"""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None

    def shutdown(self) -> None:
        """停 ROS 节点 + 关掉上游连接（deactivate 时调）。"""
        self.stop()
        self.clients.close()

    def _record_joint_state(self, msg: Any) -> None:
        """ROS 回调：记录最新夹爪开度与 6 关节向量。"""
        now = time.monotonic()
        width = _joint_state_gripper_width(msg)
        joints = _joint_state_arm_joints(msg)
        with self._lock:
            if width is not None:
                self._width = width
                self._width_mono = now
            if joints is not None:
                self._joints = joints
                self._joints_mono = now

    # ── 反馈 ────────────────────────────────────────────────────────────
    def gripper_width_now(self) -> Optional[float]:
        """最近一次夹爪开度（米）；从没收到过反馈时 None。"""
        with self._lock:
            return self._width

    def wait_gripper_width_after(self, start_mono: float,
                                 timeout_s: float) -> Optional[float]:
        """等一条"不早于 start_mono"的夹爪反馈，超时返回 None。"""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            with self._lock:
                width, seen = self._width, self._width_mono
            if width is not None and seen is not None and seen >= start_mono:
                return width
            time.sleep(0.05)
        return None

    def verify_holding(self, close_started_mono: float) -> tuple[bool, str, float]:
        """合爪之后判定"是否真的夹住了东西"。返回 (是否夹住, 说明, 实测宽度)。

        判据：反馈宽度 < grasp_threshold_width 视为夹空（手指合到底了）。
        """
        timeout_s = self.gripper.feedback_timeout_s
        threshold = self.gripper.grasp_threshold_width
        width = self.wait_gripper_width_after(close_started_mono, timeout_s)
        if width is None:
            return (False,
                    f"no gripper feedback on {self.gripper.joint_states_topic} "
                    f"within {timeout_s:.1f}s",
                    0.0)
        if width < threshold:
            return (False,
                    f"gripper closed to {width:.4f}m < "
                    f"{threshold:.4f}m threshold; likely empty",
                    width)
        return (True,
                f"gripper held at {width:.4f}m >= {threshold:.4f}m threshold",
                width)

    def latest_arm_joints_fresh(self,
                                max_age_s: float = 2.0) -> Optional[list[float]]:
        """最近一条 6 关节反馈，超过 max_age_s 视为不新鲜（返回 None）。

        陈旧守卫是必须的：这些值会被原样回放成命令。
        """
        with self._lock:
            joints, seen = self._joints, self._joints_mono
        if joints is None or seen is None:
            return None
        if (time.monotonic() - seen) > max_age_s:
            return None
        return list(joints)

    # ── 动作 ────────────────────────────────────────────────────────────
    def move_pose(self, target_pose: dict, gripper_width: float,
                  timeout_s: float, stage: str = "") -> dict:
        """笛卡尔移动：臂到 target_pose（arm/base_link），夹爪设成 gripper_width。

        服务端自己 IK、自己插值、自己安排合爪时机（要合爪就先到位再合）。
        失败抛 MotionError（连不上、超时、上游返回 success=false）。

        stage 是可选步骤名（如 "place carry"）：进日志，也进失败文案
        （"place carry failed: ‹原因›"）。编排层靠它把"哪一步失败了"写清楚，
        而不必为每一步写一遍 try/except。
        """
        endpoint = self._endpoints.get("execute_grasp")
        if not endpoint:
            raise _fail(stage, "execute_grasp not resolved on atlas")
        log.info("%smove_pose: gripper_width=%.3f timeout=%.1fs",
                 _tag(stage), gripper_width, timeout_s)
        try:
            resp = self.clients.execute_grasp(endpoint, target_pose,
                                              gripper_width, timeout_s)
        except GrpcCallError as e:
            raise _fail(stage, str(e)) from e
        log.info("%smove_pose result: success=%s msg=%r elapsed=%.2fs",
                 _tag(stage), resp.get("success"),
                 str(resp.get("message", ""))[:60],
                 float(resp.get("elapsed_s", 0.0)))
        if not resp.get("success"):
            raise _fail(stage, str(resp.get("message") or "execute_grasp failed"))
        return resp

    def go_observe(self, stage: str = "") -> None:
        """回观察位并张爪（服务端的 reset 契约，关节角在服务端配置里）。

        这个能力是可选依赖：没有就只告警——与整理前的行为一致。
        stage 见 move_pose()。
        """
        endpoint = self._endpoints.get("reset")
        if not endpoint:
            log.warning("go_observe SKIPPED — manipulation/reset not on atlas")
            return
        log.info("%sgo_observe: park at observation pose, open gripper", _tag(stage))
        try:
            resp = self.clients.reset(endpoint)
        except GrpcCallError as e:
            raise _fail(stage, str(e)) from e
        log.info("%sgo_observe result: success=%s msg=%r elapsed=%.2fs",
                 _tag(stage), resp.get("success"),
                 str(resp.get("message", ""))[:60],
                 float(resp.get("elapsed_s", 0.0)))
        if not resp.get("success"):
            raise _fail(stage, str(resp.get("message") or "reset failed"))

    def go_teach_safe(self, hold_gripper: bool = False, stage: str = "") -> None:
        """回安全位（服务端的 teach_safe 契约，关节角在服务端配置里）。

        hold_gripper=True 保持当前握持（抓完东西回位）；False 先张爪再回。
        stage 见 move_pose()。
        """
        endpoint = self._endpoints.get("teach_safe")
        if not endpoint:
            raise _fail(stage, "teach_safe capability missing")
        mode = "hold gripper" if hold_gripper else "release gripper"
        log.info("%sgo_teach_safe (%s)", _tag(stage), mode)
        try:
            resp = self.clients.teach_safe(endpoint, hold_gripper=hold_gripper)
        except GrpcCallError as e:
            raise _fail(stage, str(e)) from e
        log.info("%sgo_teach_safe result: success=%s msg=%r elapsed=%.2fs",
                 _tag(stage), resp.get("success"),
                 str(resp.get("message", ""))[:60],
                 float(resp.get("elapsed_s", 0.0)))
        if not resp.get("success"):
            raise _fail(stage, str(resp.get("message") or "teach_safe failed"))

    def open_gripper_in_place(self, stage: str = "") -> str:
        """就地张爪：读出当前 6 关节，原样回放到 /arm/joint_command，只把夹爪张开。

        零臂运动——因为回放的就是当前关节值。拿不到新鲜关节反馈或发布器时**拒绝
        执行**（宁可失败，也不猜一个会驱动臂的姿态）。返回给用户看的说明文字。

        故意不用操作服务自带的 fixed-pose put_down：它会把臂驱到一个固定释放
        姿态，可能撞关节限位。松手 = 就在原地张开手指。
        stage 见 move_pose()。
        """
        from sensor_msgs.msg import JointState

        t0 = time.monotonic()
        pub, node = self._pub, self._node
        if pub is None or node is None:
            raise _fail(stage, "joint_command publisher not ready "
                               "(ROS bridge / arm primitive down)")
        joints = self.latest_arm_joints_fresh(max_age_s=2.0)
        if joints is None:
            msg = (f"no fresh arm joint feedback on {self.gripper.joint_states_topic}; "
                   f"refusing to command (would risk moving the arm)")
            raise _fail(stage, msg)

        open_width = float(self.gripper.open_width)
        multiplier = max(1.0, float(self.gripper.val_multiple))
        gripper_joint = float(min(max(open_width, 0.0), 0.08)) / multiplier
        repeats = 10
        period_s = 0.04
        log.info("%sopen_gripper_in_place: replay joints=%s gripper %.3fm "
                 "(NO arm motion)", _tag(stage),
                 [round(v, 3) for v in joints], open_width)
        for _ in range(repeats):
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = ["joint1", "joint2", "joint3", "joint4",
                        "joint5", "joint6", "gripper"]
            msg.position = [float(v) for v in joints] + [gripper_joint]
            msg.velocity = [0.0] * 6 + [30.0]
            msg.effort = [0.0] * 6 + [float(self.gripper.command_effort)]
            pub.publish(msg)
            time.sleep(period_s)

        width = self.wait_gripper_width_after(
            t0, max(1.0, self.gripper.feedback_timeout_s))
        if width is not None and width >= self.gripper.grasp_threshold_width:
            message = f"gripper opened in place to {width:.4f}m (no arm motion)"
        else:
            seen = "unknown" if width is None else f"{width:.4f}m"
            message = (f"gripper open command sent (no arm motion); "
                       f"feedback width={seen}")
        log.info("%sopen_gripper_in_place result: %s", _tag(stage), message)
        return message
