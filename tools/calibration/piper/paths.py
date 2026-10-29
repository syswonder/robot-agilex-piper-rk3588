# Description: piper 标定的产物路径——本子树唯一来源（属性名与 s10/paths.py 对齐，
# 公共代码 common/ 认的就是这些名字）。
#
# 产物是这套工具与部署之间的接口，标定完要抄进 robonix_manifest.local.yaml：
#   intrinsics/{camera_matrix,dist_coeffs}.npy → service.grasp_pose 与 skill.pick 的
#                                               camera_matrix / dist_coeffs（技能检测前按它去畸变）
#   handeye/2d_homography.npy                 → service.grasp_pose 与 skill.pick 的 homography_matrix
#                                               （同一份矩阵，两处都要改）
#   poses/piper_calib_pose.json               → 起始位姿（人工摆好后填，roboarm_ik 的 init_joints_deg）
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ARTIFACTS = ROOT / "artifacts"

# 内参标定（steps/intrinsics.py 产出）
INTRINSICS_DIR = ARTIFACTS / "intrinsics"
CAMERA_MATRIX = INTRINSICS_DIR / "camera_matrix.npy"
DIST_COEFFS = INTRINSICS_DIR / "dist_coeffs.npy"
INTRINSICS_IMAGES = INTRINSICS_DIR / "images"

# 手眼标定（steps/handeye.py 产出）
HANDEYE_DIR = ARTIFACTS / "handeye"
HOMOGRAPHY = HANDEYE_DIR / "2d_homography.npy"
HANDEYE_IMAGE_POINTS = HANDEYE_DIR / "2d_image_points.npy"
HANDEYE_ROBOT_POINTS = HANDEYE_DIR / "2d_robot_points.npy"

# 位姿（起始位姿：标定开始前臂该待在哪儿，也是每次记录的基准点）
POSES_DIR = ARTIFACTS / "poses"
CALIB_POSE = POSES_DIR / "piper_calib_pose.json"
