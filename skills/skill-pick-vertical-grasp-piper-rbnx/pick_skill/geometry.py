# SPDX-License-Identifier: MulanPSL-2.0
"""几何换算：像素 → 臂基座 XY（2D 手眼 homography），以及"落盒点排除"判定。

职责边界：这里只有**标定出来的几何关系**，不含检测、不含臂动作。
  - `Homography`：像素中心 → `arm/base_link` 的 XY（与 grasp_pose 服务同一套算法）；
  - `DropZone`：落盒点 + 排除半径，用来判断"这个目标是不是已经被分拣进盒子里了"。

为什么独立成模块：这两个量来自**标定与部署配置**，检测与编排都只是使用者。
"""
from __future__ import annotations

import logging
import math
from typing import Optional, Sequence, Tuple

log = logging.getLogger("pick_skill.geometry")


class Homography:
    """像素 → 臂基座 XY 的 2D 单应变换。

    `matrix` 为 3x3 嵌套列表/数组（与 grasp_pose / 手眼标定同一份数据）。
    未配置时 `to_arm_xy()` 抛 ValueError —— 调用方据此判断"几何不可用"。
    """

    def __init__(self, matrix: Optional[Sequence[Sequence[float]]] = None) -> None:
        self._m = matrix

    @property
    def configured(self) -> bool:
        return self._m is not None

    def to_arm_xy(self, u: float, v: float) -> Tuple[float, float]:
        """像素 (u, v) → 臂基座 (x, y)，单位米。缺矩阵/退化时抛 ValueError。"""
        h = self._m
        if h is None:
            raise ValueError("homography matrix is not configured")
        try:
            h00, h01, h02 = (float(x) for x in h[0])
            h10, h11, h12 = (float(x) for x in h[1])
            h20, h21, h22 = (float(x) for x in h[2])
        except (TypeError, ValueError, IndexError) as e:
            raise ValueError(f"bad homography matrix: {e}") from e
        x = h00 * u + h01 * v + h02
        y = h10 * u + h11 * v + h12
        w = h20 * u + h21 * v + h22
        if abs(w) < 1e-12:
            raise ValueError("homography projection has near-zero scale")
        return x / w, y / w

    def bbox_center_to_arm_xy(self, bbox: Sequence[float]) -> Optional[Tuple[float, float]]:
        """取 bbox 中心投到臂基座 XY；bbox 不合法或投影失败 -> None（不抛）。"""
        if len(bbox) < 4:
            return None
        u = 0.5 * (float(bbox[0]) + float(bbox[2]))
        v = 0.5 * (float(bbox[1]) + float(bbox[3]))
        try:
            return self.to_arm_xy(u, v)
        except ValueError:
            return None


class DropZone:
    """落盒点 + 排除半径。

    语义（沿用整理前的规则）：抓取目标落在落盒点 `radius_m` 之内，说明它是
    **已经分拣进盒子的东西**，不是新目标 —— 此时应上报"未检测到"，让分拣循环
    自然结束，而不是把臂驱到盒子里那个不可达的位姿。
    """

    def __init__(self, center_xy: Sequence[float], radius_m: float) -> None:
        self.center_xy = (float(center_xy[0]), float(center_xy[1]))
        self.radius_m = float(radius_m)

    @property
    def enabled(self) -> bool:
        return self.radius_m > 0.0

    def distance(self, arm_xy: Sequence[float]) -> float:
        return math.hypot(float(arm_xy[0]) - self.center_xy[0],
                          float(arm_xy[1]) - self.center_xy[1])

    def bbox_inside(self, homography: Homography,
                    bbox: Sequence[float]) -> Tuple[bool, Optional[float]]:
        """这个像素框是否落在排除区里。返回 (是否在内, 距离米 or None)。

        几何不可用（没矩阵 / 投影失败 / 排除关闭）时返回 (False, None)，
        即"不排除"，与整理前行为一致（宁可多抓一次，也不误吞目标）。
        """
        if not self.enabled:
            return False, None
        arm_xy = homography.bbox_center_to_arm_xy(bbox)
        if arm_xy is None:
            return False, None
        d = self.distance(arm_xy)
        return d <= self.radius_m, d
