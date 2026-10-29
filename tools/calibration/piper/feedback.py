"""Fresh CAN feedback, using the same link6 reference as the deployed IK."""
from functools import lru_cache
from pathlib import Path
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "services/service-roboarm-ik-rbnx"))
from roboarm_ik.solver import PiperRoboarmIk


@lru_cache(maxsize=1)
def solver():
    return PiperRoboarmIk(urdf_path=str(REPO / "urdf/piper.urdf"))


def read_feedback(arm, timeout_s=3.0):
    """Wait for new pose, joint and status messages; never send motion commands."""
    getters = (arm.piper.GetArmEndPoseMsgs, arm.piper.GetArmJointMsgs,
               arm.piper.GetArmStatus)
    initial = [float(get().time_stamp) for get in getters]
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        stamps = [float(get().time_stamp) for get in getters]
        if all(now > 0 and now != old for now, old in zip(stamps, initial)):
            break
        time.sleep(0.02)
    else:
        raise RuntimeError("CAN 位姿/关节/状态没有新反馈，未记录")
    xyz = arm.end_pos()
    joints = arm.joints()
    fk = solver().chain.forward_kinematics(np.deg2rad(joints)).matrix()
    status = arm.status()
    return {
        "xyz_m": xyz.tolist(), "joints_deg": joints,
        "fk_xyz_m": fk[:3, 3].tolist(),
        "sdk_fk_difference_mm": float(np.linalg.norm(xyz - fk[:3, 3]) * 1000),
        "tilt_from_down_deg": float(np.rad2deg(np.arccos(np.clip(-fk[2, 2], -1, 1)))),
        "ctrl_mode": int(status.ctrl_mode), "teach_status": int(status.teach_status),
        "arm_status": int(status.arm_status), "err_code": int(status.err_code),
        "feedback_timestamps": stamps, "recorded_at_unix": time.time(),
    }


def read_stationary_sample(arm):
    readings = []
    for _ in range(8):
        readings.append(read_feedback(arm))
        time.sleep(0.05)
    positions = np.array([r["xyz_m"] for r in readings])
    spread = float(np.linalg.norm(np.ptp(positions, axis=0)))
    if spread > 0.002:
        raise ValueError(f"机械臂仍在移动（变化 {spread * 1000:.1f} mm），停稳后再记录")
    sample = readings[-1]
    if any(r["err_code"] or r["arm_status"] not in (0, 11) for r in readings):
        raise ValueError(f"机械臂状态异常：{sample['arm_status']} / {sample['err_code']}")
    if max(r["tilt_from_down_deg"] for r in readings) >= 8.0:
        raise ValueError(f"夹爪需要朝下；当前偏斜 {sample['tilt_from_down_deg']:.1f}°，要求 <8°")
    if max(r["sdk_fk_difference_mm"] for r in readings) > 5.0:
        raise ValueError("SDK 末端与运行时 IK 参考点相差超过 5 mm，需先核对工具坐标")
    sample["xyz_m"] = positions.mean(axis=0).tolist()
    sample["stationary_spread_mm"] = spread * 1000
    return sample
