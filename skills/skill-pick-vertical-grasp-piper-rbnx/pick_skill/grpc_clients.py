# SPDX-License-Identifier: MulanPSL-2.0
"""gRPC 传输：到上游服务的通道与桩，以及四个契约的调用包装。

职责边界：只负责"把请求发出去、把响应变成 dict"。不决定发什么（那是编排层
的事），不重试、不降级、不解释业务含义。

**错误约定：调用失败一律抛 `GrpcCallError`**（连接失败、超时、上游异常）。
调用方按异常处理，不需要再检查返回值里有没有错误字段。

一个 `Clients` 实例 = 一套连接缓存（通道按 endpoint、桩按 kind@endpoint）。
技能 activate 前造好，deactivate 时 `close()`——缓存是实例状态，不留在模块里，
所以不会有"上一次运行残留的通道"。
"""
from __future__ import annotations

import logging
import threading
from typing import Any

from .poses import from_protobuf, to_protobuf

log = logging.getLogger("pick_skill.grpc")


class GrpcCallError(RuntimeError):
    """一次上游 gRPC 调用失败（连不上、超时、上游抛错）。"""


# 契约名 → codegen 生成的桩类名（robonix_contracts_pb2_grpc 里的属性）
_STUB_CLASSES = {
    "grasp_request": "RobonixServicePerceptionGraspPoseGraspRequestStub",
    "execute_grasp": "RobonixServiceManipulationExecuteGraspStub",
    "reset":         "RobonixServiceManipulationResetStub",
    "teach_safe":    "RobonixServiceManipulationTeachSafeStub",
}


class Clients:
    """上游服务的 gRPC 客户端集合（通道与桩按需建、按 endpoint 缓存）。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._channels: dict[str, Any] = {}   # host:port → grpc.Channel
        self._stubs: dict[str, Any] = {}      # "kind@endpoint" → generated stub

    # ── 连接 ────────────────────────────────────────────────────────
    def channel_for(self, endpoint: str):
        """取（或建）一个 endpoint 的 grpc.Channel。"""
        with self._lock:
            ch = self._channels.get(endpoint)
            if ch is not None:
                return ch
        import grpc
        ch = grpc.insecure_channel(
            endpoint, options=[("grpc.enable_http_proxy", 0)])
        with self._lock:
            self._channels[endpoint] = ch
        return ch

    def stub_for(self, kind: str, endpoint: str):
        """取某个契约的 gRPC 桩，按 kind@endpoint 缓存。"""
        key = f"{kind}@{endpoint}"
        with self._lock:
            stub = self._stubs.get(key)
            if stub is not None:
                return stub
        import robonix_contracts_pb2_grpc as contracts_grpc
        try:
            stub_cls = getattr(contracts_grpc, _STUB_CLASSES[kind])
        except (AttributeError, KeyError) as e:
            raise GrpcCallError(
                f"no generated stub for contract {kind!r}: {e}") from e
        stub = stub_cls(self.channel_for(endpoint))
        with self._lock:
            self._stubs[key] = stub
        return stub

    def close(self) -> None:
        """关掉所有缓存的通道并清空桩。"""
        with self._lock:
            for ch in self._channels.values():
                try:
                    ch.close()
                except Exception:  # noqa: BLE001
                    pass
            self._channels.clear()
            self._stubs.clear()

    # ── 契约调用 ────────────────────────────────────────────────────
    def grasp_request(self, endpoint: str, object_name: str, bbox_2d: list,
                      object_center_3d: list, retry: int = 0) -> dict:
        """`grasp_pose.grasp_request`：算抓取位姿。返回 MCP 形状的 dict。"""
        try:
            import grasp_pb2

            req = grasp_pb2.GraspRequest_Request(
                object_name=str(object_name),
                retry=int(retry),
            )
            req.bbox_2d.extend(float(x) for x in bbox_2d or [])
            req.object_center_3d.extend(float(x) for x in object_center_3d or [])
            resp = self.stub_for("grasp_request", endpoint).GraspRequest(req)
            return {
                "grasp_pose": from_protobuf(resp.grasp_pose),
                "gripper_width": float(resp.gripper_width),
                "score": float(resp.score),
                "success": bool(resp.success),
                "message": str(resp.message),
            }
        except GrpcCallError:
            raise
        except Exception as e:  # noqa: BLE001
            log.warning("grpc call grasp_request failed: %s", e)
            raise GrpcCallError(f"grasp_request: {e}") from e

    def execute_grasp(self, endpoint: str, target_pose: dict,
                      gripper_width: float, timeout_s: float) -> dict:
        """`roboarm_ik.execute_grasp`：驱动臂到目标位姿 + 夹爪。"""
        try:
            import manipulation_pb2

            req = manipulation_pb2.ExecuteGrasp_Request(
                target_pose=to_protobuf(target_pose),
                gripper_width=float(gripper_width),
                timeout_s=float(timeout_s),
            )
            resp = self.stub_for("execute_grasp", endpoint).ExecuteGrasp(req)
            return {
                "success": bool(resp.success),
                "message": str(resp.message),
                "elapsed_s": float(resp.elapsed_s),
            }
        except GrpcCallError:
            raise
        except Exception as e:  # noqa: BLE001
            log.warning("grpc call execute_grasp failed: %s", e)
            raise GrpcCallError(f"execute_grasp: {e}") from e

    def reset(self, endpoint: str) -> dict:
        """`roboarm_ik.reset`：回观察位 + 张爪。"""
        try:
            import manipulation_pb2

            resp = self.stub_for("reset", endpoint).Reset(
                manipulation_pb2.Reset_Request(ack=True))
            return {
                "success": bool(resp.success),
                "message": str(resp.message),
                "elapsed_s": float(resp.elapsed_s),
            }
        except GrpcCallError:
            raise
        except Exception as e:  # noqa: BLE001
            log.warning("grpc call reset failed: %s", e)
            raise GrpcCallError(f"reset: {e}") from e

    def teach_safe(self, endpoint: str, hold_gripper: bool = False) -> dict:
        """`roboarm_ik.teach_safe`：回安全位（可选保持握持）。"""
        try:
            import manipulation_pb2

            resp = self.stub_for("teach_safe", endpoint).TeachSafe(
                manipulation_pb2.TeachSafe_Request(
                    ack=True,
                    hold_gripper=bool(hold_gripper),
                ))
            return {
                "success": bool(resp.success),
                "message": str(resp.message),
                "elapsed_s": float(resp.elapsed_s),
            }
        except GrpcCallError:
            raise
        except Exception as e:  # noqa: BLE001
            log.warning("grpc call teach_safe failed: %s", e)
            raise GrpcCallError(f"teach_safe: {e}") from e
