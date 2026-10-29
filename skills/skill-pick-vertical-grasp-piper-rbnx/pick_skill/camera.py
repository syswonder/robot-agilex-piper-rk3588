# SPDX-License-Identifier: MulanPSL-2.0
"""相机取图：帧缓存 + 新鲜度判断 + 去畸变。

职责边界（本模块只做"拿到一张可用的图"）：
  - 缓存 ROS 回调送来的帧，记录收帧时刻；
  - 判断帧是否够新；
  - 按标定内参去畸变。

不做检测、不认物体、不碰臂——检测在 `detect.py`，编排在 `atlas_bridge.py`。

**为什么去畸变放在这里**：手眼 homography 是在"去畸变图"上采点的
（不去畸变时角落像素可偏 ~30px ≈ 3cm）。把"去畸变"和"取图"绑成一步，
任何检测路径都不可能漏做。
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .config import floats, matrix, text
from .motion import qos_from_profile

log = logging.getLogger("pick_skill.camera")

@dataclass
class CameraConfig:
    """相机相关配置：话题、标定内参（去畸变用）。

    话题名**不写死在代码里**：由 atlas 解析 `robonix/primitive/camera/rgb` 得到；
    解析不到时可以用配置键 camera_topic 明确指定（那是操作员的决定），两者都没有
    就不订阅——宁可检测直接失败，也不猜一个话题然后静默收不到帧。
    帧必须与手眼标定时的分辨率一致（各部署都是 1280x720）。
    """

    topic: str = ""
    camera_matrix: Optional[list[list[float]]] = None
    dist_coeffs: Optional[list[float]] = None

    @classmethod
    def from_cfg(cls, cfg: dict) -> "CameraConfig":
        """按 manifest 的 config 块建配置；缺内参时去畸变退化为恒等。"""
        return cls(
            topic=text(cfg, "camera_topic"),
            camera_matrix=matrix(cfg, "camera_matrix", 3, 3),
            dist_coeffs=floats(cfg, "dist_coeffs"),
        )


class CameraFrames:
    """最新一帧的缓存。ROS 订阅回调调用 `update()`；检测侧用 `latest()` / `frame()`。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._seen_mono: float = 0.0
        self._camera_matrix = None
        self._dist_coeffs = None
        self._topic = ""
        self._qos = "best_effort"
        self._cb_count = 0

    # ── 配置 ────────────────────────────────────────────────────────────
    @classmethod
    def from_cfg(cls, cfg: dict) -> "CameraFrames":
        """按 manifest 的 config 块建一个已配置好的帧缓存（话题 + 内参）。"""
        cam = CameraConfig.from_cfg(cfg)
        frames = cls()
        frames.set_topic(cam.topic)
        frames.configure(cam.camera_matrix, cam.dist_coeffs)
        return frames

    def set_topic(self, topic: str, qos: str = "best_effort") -> None:
        """设置要订阅的图像话题与 QoS（在 attach() 之前调）。

        话题与 QoS 由编排层经 atlas 解析 `robonix/primitive/camera/rgb` 契约得到
        （解析不到时用配置兜底），这里只负责订阅。
        """
        self._topic = str(topic or "").strip()
        self._qos = qos

    def configure(self, camera_matrix=None, dist_coeffs=None) -> None:
        """设置标定内参；不设置则 `undistort()` 为恒等操作（并告警一次）。"""
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        if camera_matrix is None or dist_coeffs is None:
            log.warning("camera intrinsics not configured — frames stay "
                        "undistorted (hand-eye homography assumes undistorted "
                        "pixels; corner error can reach ~30px otherwise)")

    # ── 订阅（ROS 节点上挂回调，帧由回调喂进来）──────────────────────────
    def attach(self, node) -> bool:
        """在 rclpy 节点上订阅 `set_topic()` 指定的图像话题。

        回调把消息转成 bgr8 帧（与 YOLO/VLM 检测路径一致）交给 `update()`。
        返回是否订阅成功；失败只告警，由调用方决定怎么办。
        """
        if not self._topic:
            log.warning("camera topic not set — not subscribing")
            return False
        try:
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except Exception as e:  # noqa: BLE001
            log.warning("camera subscriber unavailable: %s", e)
            return False
        bridge = CvBridge()

        def _on_image(msg) -> None:
            try:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:  # noqa: BLE001
                return
            self.update(frame)
            if self._cb_count % 100 == 0:
                log.info("camera frame #%d %dx%d", self._cb_count,
                         frame.shape[1], frame.shape[0])
            self._cb_count += 1

        node.create_subscription(Image, self._topic, _on_image,
                                 qos_from_profile(self._qos))
        log.info("camera subscribed: %s", self._topic)
        return True

    # ── 写入（ROS 回调）────────────────────────────────────────────────
    def update(self, frame: np.ndarray) -> None:
        """记录最新一帧及其到达时刻。"""
        with self._lock:
            self._frame = frame
            self._seen_mono = time.monotonic()

    # ── 读取 ────────────────────────────────────────────────────────────
    def latest(self) -> Tuple[Optional[np.ndarray], float]:
        """返回 (最新帧, 该帧的年龄秒数)；无帧时 (None, 0.0)。"""
        with self._lock:
            frame, seen = self._frame, self._seen_mono
        age = (time.monotonic() - seen) if frame is not None else 0.0
        return frame, age

    def clear(self) -> None:
        """丢掉缓存帧（deactivate 时用）。"""
        with self._lock:
            self._frame = None
            self._seen_mono = 0.0

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        """按标定内参去畸变；未配置或失败时原样返回（不抛）。"""
        if self._camera_matrix is None or self._dist_coeffs is None:
            return frame
        try:
            import cv2
            return cv2.undistort(
                frame,
                np.asarray(self._camera_matrix, dtype=np.float64),
                np.asarray(self._dist_coeffs, dtype=np.float64),
            )
        except Exception as e:  # noqa: BLE001
            log.warning("undistort failed (%s) — using raw frame", e)
            return frame

    def frame(self, max_age_s: float,
              wait_s: float = 0.0) -> Optional[np.ndarray]:
        """取一张**可用**的图（已去畸变）：

        - 还没有帧时最多等 `wait_s` 秒（技能刚激活、首帧未到时用得上）；
        - 帧比 `max_age_s` 旧则视为不可用（臂抖动/卡流时避免用陈旧图）；
        - 拿到了 -> 去畸变后返回；等不到 -> None（调用方决定怎么报错）。
        """
        deadline = time.monotonic() + max(0.0, wait_s)
        while True:
            frame, age = self.latest()
            if frame is not None and age <= max_age_s:
                return self.undistort(frame)
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.1)
