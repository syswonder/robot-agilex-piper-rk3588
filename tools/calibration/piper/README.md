# 纯臂 piper 的标定台

> 整条部署路径（环境 → 服务 → 镜像/容器 → 清单 → 标定 → 抓取）见 [`README.md`](../../../README.md)；本文只讲这个标定台本身。

一个 Flask 服务，把标定流程分成三步：**内参 → 手眼 → 验证**。整套跑在**宿主机上**：

- **臂**：宿主 `piper_sdk` 直连 USB-CAN（与 S10 共用 `common/robot/` 下的代码），CAN 口自动探测
- **相机**：宿主 `pyorbbecsdk` 直连 Gemini 336L（与相机原语同一个 SDK，同为 1280×720 彩色流）

标定只需要取一帧画面、动一下机械臂，不值得为此启动整套 robonix、再绕一层 ROS 话题与服务；而且标定本来就该在启动 robonix 之前做。

## 跑起来

```bash
conda create -y -n piper-calib python=3.10
conda activate piper-calib
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r tools/calibration/piper/requirements.txt
cd tools/calibration/piper && python app.py --port 8090
# 浏览器打开 http://<本机IP>:8090
```

## 三步

| 页 | 干什么 | 产物 |
|---|---|---|
| `/intrinsics` | 棋盘格采 10~20 张，解相机内参 K/D（画面是**原始**的，不能去畸变） | `artifacts/intrinsics/{camera_matrix,dist_coeffs}.npy` |

内参那页的两个数字填的是**内角点数**，不是格子数——12×8 格的板子填 **11 × 7**（20×14 格的填 19 × 13），填成格子数会一个角点都找不到。格子边长不用填：它只影响解出来的棋盘位姿有多大，解 K/D 用不到。没有板子的话，`../boards/` 里有现成可打印的 A4/A3 版（打印选「实际大小」，按页面默认的 11×7 就行）。

拍法直接影响能不能解出**焦距**：板子要占画面 1/4 以上，每次明显倾斜（±20~30°）、远近也变一变。板子小、姿态单一时，焦距和畸变会互相顶班——误差一样小、参数却差十几倍。工具解算后会自己量一遍（把焦距钉在一串固定值上看误差变不变），量不出来会直接提示重拍。
| `/handeye` | 点画面上的点 → 按臂上示教按钮进示教、手拖夹爪尖对准 → 记录 → 回起始位姿；采够后拟合单应 | `artifacts/handeye/{2d_image_points,2d_robot_points,2d_homography}.npy` |
| `/verify` | 点一个点 → 臂把夹爪尖移过去（像素 → 单应 → IK），人看对不对 | 无（动臂） |

**手眼那步的画面一定是去畸变后的**：标定输入必须与运行时一致（技能检测前也按同一份内参去畸变）。没标内参时，这一步在首页上就是「不可用」，直接开它的 URL 也进不去、接口同样被拒；已有内参时（本子树 `artifacts/` 里的，或从清单里现成那份抄过来的）可以直接开始标手眼——两条入口走的是同一段流程。「验证」同理：缺内参或单应都不让进。

`/handeye` 和 `/verify` 都以 `artifacts/poses/piper_calib_pose.json` 作为**起始位姿**——就是清单里 `roboarm_ik.init_joints_deg` 那个固定姿态（这台是 `[-90, 0, 0, 0, 0, 0]`，臂摆到侧面）。相机固定在顶部俯视，臂立着会挡住桌面，所以标定时让它待在侧面；同时它也是每次记录/移动的固定起点。换臂型或安装方式要重填这个值。

## 产物怎么进部署清单

| 产物 | 清单里的键 |
|---|---|
| `artifacts/intrinsics/camera_matrix.npy` | `skill.pick.config.camera_matrix`（只有技能读它——去畸变是技能自己做的） |
| `artifacts/intrinsics/dist_coeffs.npy` | `skill.pick.config.dist_coeffs`（同上） |
| `artifacts/handeye/2d_homography.npy` | `service.grasp_pose.config.homography_matrix` **和** `skill.pick.config.homography_matrix`（同一份矩阵，两处都要改） |
| `artifacts/handeye/2d_homography_report.txt` | 不用抄，留着看误差（内点、平均/最大误差） |

清单里另外两个值这套工具**不产出**，不要混：

- `default_desktop_height: 0.135`（`grasp_pose`）与 `desktop_height: 0.135`（`pick`）：桌面在臂基座系的高度，两处共用一个值（`grasp_offset: 0`）。换安装方式必须重新测量。
- `roboarm_ik.config.init_joints_deg`：起始/观察位姿，不是标定出来的（相机固定在顶部，这个位姿只是把臂让开）。

## 与 roboarm / 旧工具的关系

这套流程对应 roboarm 的 `calibrate_handeye_2d.py`（采点 + 拟合）与 `test_handeye_2d.py`（验证），但**不再依赖 robonix 栈**：

| 老做法 | 现在 |
|---|---|
| 臂：`/moveit_control/reset`、`/moveit_control/teach_safe`、`/arm/enable_srv` | 直连 CAN，用臂上示教按钮进/出拖动示教 |
| 画面：ROS 话题 `/camera/color/image_raw` | 宿主 `pyorbbecsdk` 直接取图 |
| 验证：经 atlas 调 `manipulation/execute_grasp` | 让臂尖移动到映射点，肉眼看（端到端抓取由运行时的抓取技能自己验） |
| 界面：两个窗口版脚本 | 一个网页标定台（三步各一页） |
| 数据目录：`~/lhw/rbnx_piper_packages/rbnx-boot/hand-eye-data` | `tools/calibration/piper/artifacts/` |

还补上了 roboarm 那套里没有的一环：内参标定。老流程的矩阵是在去畸变图上采的，但去畸变用的内参来自 roboarm 自己的标定工具；现在内参与手眼都在同一个工具里，顺序由流程保证（手眼页要求先有内参）。

## 实机情况

2026-09-11 在 Jetson AGX Orin + Gemini 336L + Piper 上完整走过一遍：内参 11 张（RMS 0.47 px）、手眼 7 个点（平均 3.5 mm）。下面几条是那次踩出来的：

- 相机档位写死在 `vision/camera.py`（1280×720、优先 MJPG）；启动日志里会打印实际用的档位。
- **别用 `kill` 直接杀工具进程**：相机（UVC）没被正常关闭时设备会卡住，之后新起的实例"打开成功但一帧都没有"。彻底停掉工具再起就好；实在不行把相机 USB 拔插一次。相机出问题时页面顶部会显示原因（打开失败 / 取帧超时）。
- "清空图片""清空已采点"这类按钮不占硬件，别的步骤开着也能用；但**拍照、记录点、让臂动**这些必须独占相机/臂，被别的步骤占着会被拒（页面会说你该去哪页点「结束本步骤」）。
- CAN 口自动探测只认 USB-CAN（gs_usb），板载 mttcan 不算；接口是 DOWN 时页面会给出启用命令，但不会替你改接口状态。
- 标定出的矩阵与内参**要人工抄进清单**（对照表在上面），没有自动写回。

配置文件位于 [`config/calibration/piper.yaml`](../../../config/calibration/piper.yaml)。
