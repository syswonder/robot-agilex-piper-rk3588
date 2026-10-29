# SPDX-License-Identifier: MulanPSL-2.0
"""位姿：技能与上游服务之间传递的 PoseStamped。

职责边界：只处理"位姿这个东西本身"——构造、平移、以及三种表示之间的转换。
不调服务、不发话题、不读配置、不碰臂。

同一个位姿在三处出现，是三副面孔而不是三个概念：

    dict        技能内部流通的形状（字段与 codegen 的 to_dict() 对齐）：
                {"header": {"stamp": {"sec":…, "nanosec":…}, "frame_id": …},
                 "pose":   {"position":    {"x":…, "y":…, "z":…},
                            "orientation": {"x":…, "y":…, "z":…, "w":…}}}
    protobuf    走 gRPC 请求/响应（geometry_msgs_pb2.PoseStamped）
    MCP 数据类  组装 MCP 工具返回值（codegen 生成的数据类）

坐标系约定：arm/base_link（roboarm_ik 只接受 ""/base_link/arm/base_link）。
"""
from __future__ import annotations

from typing import Any

from geometry_msgs_mcp import (  # noqa: E402
    PoseStamped, Pose, Point, Quaternion,
)
from std_msgs_mcp import Header  # noqa: E402
from builtin_interfaces_mcp import Time  # noqa: E402


# ── 构造与变换（技能内部用）────────────────────────────────────────────────
def empty_pose() -> dict:
    """空位姿（全零位置 + 单位四元数）。响应里"没有位姿"时用它占位，
    字段名与嵌套必须和 codegen 的 PoseStamped.from_dict() 形状一致。"""
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0},
                   "frame_id": "arm/base_link"},
        "pose":   {"position":    {"x": 0.0, "y": 0.0, "z": 0.0},
                   "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
    }


def vertical_pose(x: float, y: float, z: float) -> dict:
    """竖直向下（yaw=0）的位姿：夹爪朝下，位置在 (x, y, z)。

    朝向与 grasp_pose 的 _vertical_quaternion(0) 一致：(x,y,z,w)=(0,1,0,0)。
    """
    return {
        "header": {"stamp": {"sec": 0, "nanosec": 0},
                   "frame_id": "arm/base_link"},
        "pose":   {"position":    {"x": float(x), "y": float(y), "z": float(z)},
                   "orientation": {"x": 0.0, "y": 1.0, "z": 0.0, "w": 0.0}},
    }


def with_z(pose: dict, z: float) -> dict:
    """位姿的副本，只把位置 z 换成 z（米），x/y/姿态不动。深拷贝，不改原字典。

    悬停位就是这么从抓取位姿推出来的：x/y 与朝向沿用目标位姿，高度换成桌面高度
    加一个偏移（见 atlas_bridge 的高度模型）。
    """
    import copy
    d = copy.deepcopy(pose)
    pos = d.get("pose", {}).get("position", {})
    pos["z"] = float(z)
    return d


# ── 转换（跨格式边界用）────────────────────────────────────────────────────
def from_protobuf(msg: Any) -> dict:
    """protobuf PoseStamped → dict（上游服务的 gRPC 响应转成技能内部形状）。"""
    return {
        "header": {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "frame_id": str(msg.header.frame_id),
        },
        "pose": {
            "position": {
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
            },
            "orientation": {
                "x": float(msg.pose.orientation.x),
                "y": float(msg.pose.orientation.y),
                "z": float(msg.pose.orientation.z),
                "w": float(msg.pose.orientation.w),
            },
        },
    }


def to_protobuf(d: dict) -> Any:
    """dict → protobuf PoseStamped（技能内部形状装进 gRPC 请求）。"""
    import builtin_interfaces_pb2
    import geometry_msgs_pb2
    import std_msgs_pb2

    header = d.get("header", {}) or {}
    stamp = header.get("stamp", {}) or {}
    pose = d.get("pose", {}) or {}
    pos = pose.get("position", {}) or {}
    ori = pose.get("orientation", {}) or {}
    return geometry_msgs_pb2.PoseStamped(
        header=std_msgs_pb2.Header(
            stamp=builtin_interfaces_pb2.Time(
                sec=int(stamp.get("sec", 0)),
                nanosec=int(stamp.get("nanosec", 0)),
            ),
            frame_id=str(header.get("frame_id", "arm/base_link")),
        ),
        pose=geometry_msgs_pb2.Pose(
            position=geometry_msgs_pb2.Point(
                x=float(pos.get("x", 0.0)),
                y=float(pos.get("y", 0.0)),
                z=float(pos.get("z", 0.0)),
            ),
            orientation=geometry_msgs_pb2.Quaternion(
                x=float(ori.get("x", 0.0)),
                y=float(ori.get("y", 0.0)),
                z=float(ori.get("z", 0.0)),
                w=float(ori.get("w", 1.0)),
            ),
        ),
    )


def to_mcp(d: dict) -> PoseStamped:
    """dict → codegen 数据类（组装 MCP 工具返回值）。

    dict 结构必须与上游返回的 PoseStamped to_dict() 形状一致。
    """
    h_in = d.get("header") or {}
    s_in = h_in.get("stamp") or {}
    p_in = (d.get("pose") or {})
    pos_in = p_in.get("position") or {}
    ori_in = p_in.get("orientation") or {}
    return PoseStamped(
        header=Header(
            stamp=Time(sec=int(s_in.get("sec", 0)),
                       nanosec=int(s_in.get("nanosec", 0))),
            frame_id=str(h_in.get("frame_id", "")),
        ),
        pose=Pose(
            position=Point(
                x=float(pos_in.get("x", 0.0)),
                y=float(pos_in.get("y", 0.0)),
                z=float(pos_in.get("z", 0.0))),
            orientation=Quaternion(
                x=float(ori_in.get("x", 0.0)),
                y=float(ori_in.get("y", 0.0)),
                z=float(ori_in.get("z", 0.0)),
                w=float(ori_in.get("w", 1.0))),
        ),
    )
