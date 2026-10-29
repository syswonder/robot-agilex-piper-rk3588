# skill-pick-vertical-grasp-piper-rbnx（纯臂版）

Robonix 面向用户的抓取技能（AgileX Piper 臂 + 固定顶部相机）。拥有
`robonix/skill/pick/*`。

Catalog name: `robonix.skill.pick.vertical_grasp_piper`.

本包与 **S10 版**（`skill-pick-vertical-grasp-rbnx`）是同一套模块结构的孪生包，
差别只有两处部署事实：相机在**顶部**（而不是臂上），grab 成功后停在**悬停位**
（而不是回安全位）。共享的抓取段/放置段都在 `_grasp_object` / `_place_at`。

## Capability surface

| Contract | Transport | Purpose |
| --- | --- | --- |
| `robonix/skill/pick/driver` | gRPC | Lifecycle (`CMD_INIT` / lazy `CMD_ACTIVATE`) |
| `robonix/skill/pick/pick` | MCP | 分拣：抓起 → 搬进盒子 → 释放 → 空手回安全位 |
| `robonix/skill/pick/grab` | MCP | 抓起并保持握持，停在悬停位（握在手里等指令） |
| `robonix/skill/pick/demo` | MCP | 在 `demo_pos_a` / `demo_pos_b` 之间来回分拣同一物体 |
| `robonix/skill/pick/put_in_box` | MCP | 把已握着的东西搬进盒子、释放、回安全位 |
| `robonix/skill/pick/put_down` | MCP | 就地张爪（零臂运动） |

`grab`（以及 `pick` / `demo`）接受**任意用户措辞**作为 `object_name`：

```json
{"object_name": "红色物体"}
```

It takes the latest camera frame itself, POSTs it to the host GPU YOLO server
(:8770, no `object_name` filter -> every detection on the table), then an LLM
(`grab_match_llm_*`, same provider as the pilot planner) matches the request
against the detections. No match -> "not detected" error, nothing grasped. The
call is synchronous, uses a fixed 60 s budget, and makes one grasp attempt.

## Runtime paths

```text
pilot / executor
  -> pick_skill.grab(object_name)                     MCP
  -> reset to the observation pose (arm/base_link homography is only
     valid from that repeatable pose) — stage 0
  -> in-skill detection (no object_detect service):
       camera frame -> undistort -> host GPU YOLO detect-all (:8770,
       no object_name filter) -> 2-frame consistency -> box exclusion
       -> LLM match (grab_match_llm_*) request vs detections
       (or the VLM path when detect_scheme=vlm)
  -> grasp_pose.grasp_request                        gRPC
  -> roboarm_ik.execute_grasp ×3 (hover / descend+close / lift)
  -> gripper feedback check on the arm's joint_states contract
  -> 停在**悬停位**（抓取点正上方）：夹爪闭合、物体仍在手里，等下一步指令

pilot / executor
  -> pick_skill.pick(object_name)                    MCP
  -> _grasp_object（同上）+ _place_at(盒点)：搬进盒子、释放、空手回安全位

pilot / executor
  -> pick_skill.demo(object_name)                    MCP
  -> 每轮：检测（算它离哪个演示点更远）→ _grasp_object → _place_at(对面那个点)

pilot / executor
  -> pick_skill.put_in_box()                         MCP
  -> _place_at(盒点)：搬进盒子、释放、空手回安全位

pilot / executor
  -> pick_skill.put_down()                           MCP
  -> replay the live joints on the arm's joint_command contract with only the
     gripper opened — zero arm motion
```

话题名与 QoS **都不写死**：`arm/joint_states`、`arm/joint_command`、`camera/rgb`
三个原语契约在 activate 时经 atlas 解析得到（解析不到就报错，不猜）。

Atlas resolves all endpoints. There is no legacy ROS-service or
`/graspnet` fallback in this skill.

## Lifecycle and dependencies

At boot, `CMD_INIT` only parses configuration and the skill remains
`INITIALIZED`. On the first tool call, `CMD_ACTIVATE` resolves upstream
endpoints.

Required:

- `service/perception/grasp_pose/grasp_request` over gRPC;
- `service/manipulation/execute_grasp` over gRPC.

Optional during activation (missing -> warning, degraded behaviour):

- `reset`: 观察位（顶部相机与臂姿态无关，观察位的作用是让臂让开视野——这份
  部署里它本身就是"大轴转开"的姿态）与失败回位；
- `teach_safe`: 放置/失败收尾回安全位（`grab` 成功时**不**用它——它停在悬停位）。

`service/perception/object_detect/detect_object` 刻意**不是**依赖：检测在技能内做
（去畸变 + 两帧一致性 + 措辞匹配 + 盒内排除，服务里没有这些）。

## Grab sequence

1. If `reset` is available, move to the calibrated observation pose.
2. Detect the target in-skill (see above).
3. Compute one vertical grasp pose with `grasp_pose`.
4. `_grasp_and_verify`: hover (pre_pose, gripper open) -> descend to the grasp
   pose and close -> lift back to pre_pose still closed -> check fresh gripper
   feedback. A failed approach or descend aborts; a failed lift only warns (the
   object may already be in the gripper).
5. On success, **hold at the hover pose** with the gripper closed (object still
   held, no lateral motion) — `put_in_box` continues from there.

The grasp z comes from `grasp_pose` (its own configured grasp height); every
in-air height the skill commands comes from the height model below
(`desktop_height` + `travel_offset`). The skill does not perform camera
calibration or IK itself.

## Configuration

```yaml
gripper_open_width: 0.08              # metres
gripper_close_width: 0.0              # metres
gripper_grasp_threshold_width: 0.005  # measured width above this means held
gripper_feedback_timeout_s: 2.0
# 话题默认空：由 atlas 解析 arm/joint_states、arm/joint_command、camera/rgb
# 三个原语契约得到；只有要显式指定时才配 *_topic。
camera_topic: ""
gripper_joint_states_topic: ""
gripper_joint_command_topic: ""
desktop_height: 0.135                 # desktop height in arm/base_link (m).
                                      # PER DEPLOY: set it from your own
                                      # calibration; the frame travels with the
                                      # arm mount, so 实验台与狗背各不相同。
grasp_offset: 0.0                     # grasp/place height = desktop + this
                                      # （纯臂：合爪高度就是桌面高度，所以 0）
travel_offset: 0.10                   # every in-air height = desktop + this
                                      # (grasp hover, lift, carry, drop release)
demo_pos_a / demo_pos_b               # demo 往返的两点（arm/base_link）
yolo_http_url: http://127.0.0.1:8770/detect  # host GPU YOLO (detect-all)
yolo_conf: 0.5                        # confidence floor（土豆~0.52/胡萝卜~0.54）
yolo_iou: 0.45
yolo_two_frame_interval_s: 0.7        # two-frame consistency gap
yolo_two_frame_iou: 0.3               # ... and its same-object IoU
detect_scheme: yolo                   # yolo | vlm
grab_vlm_base_url / _api_key / _model # used when detect_scheme=vlm
grab_match_llm_base_url / _api_key / _model  # wording matcher (text LLM)
homography_matrix / camera_matrix / dist_coeffs  # pixel→arm XY + undistort
box_xy / place_positions / place_fallback_xy     # the single box drop point
place_go_down_before_open: false      # true: descend before releasing; false:
                                      # drop mode — open at the hover pose
place_jitter_enabled: false
place_jitter_m: 0.03
runtime_config_file: ""               # live config file (place mode +
                                      # detect_scheme), re-read before each
                                      # action; absent = use the values above.
                                      # Legacy key name: place_mode_override_file
place_exclusion_radius_m: 0.15        # detections this close to the box are
                                      # treated as already sorted
```

## Layout

```text
skill-pick-vertical-grasp-piper-rbnx/
├── package_manifest.yaml
├── capabilities/
│   ├── driver.v1.toml / pick.v1.toml / grab.v1.toml
│   ├── demo.v1.toml / put_down.v1.toml / put_in_box.v1.toml
│   ├── lib/{pick,demo,grasp,manipulation}/srv/
│   └── service/{perception,manipulation}/
├── pick_skill/
│   ├── atlas_bridge.py   编排：能力声明、生命周期、六个工具的流程、
│   │                     _grasp_object / _place_at 两个公共件、
│   │                     SkillConfig / HeightConfig / PlaceConfig
│   ├── motion.py         运动：Arm 类（move_pose / go_observe / go_teach_safe /
│   │                     open_gripper_in_place）+ GripperConfig，失败抛 MotionError
│   ├── grpc_clients.py   传输：Clients 类（上游通道/桩 + 四个契约调用）
│   ├── poses.py          位姿：构造、平移、dict/protobuf/MCP 三种表示转换
│   ├── camera.py         取图：帧缓存、新鲜度、去畸变（含图像订阅）+ CameraConfig
│   ├── geometry.py       像素 → 臂基座 XY + 落盒点排除判定
│   ├── detect.py         检测：yolo/vlm 统一入口（可关盒内排除）+ DetectConfig
│   └── config.py         配置读取：num/text/flag/matrix/xy… 统一类型与坏值处理
└── scripts/{build,start}.sh
```

This is a pure-Python orchestration package. Its build is codegen-only; it
does not vendor a ROS package.

分工原则：`atlas_bridge.py` 只做业务编排，只调其余模块的公开函数；运动、传输、
位姿、取图、检测、配置各自成模块，不互相穿透。每个模块用 `from_cfg` 解析自己
那份配置（谁消费谁定义键名与默认值），`init` 只负责组装。

## Build and verification

```bash
bash scripts/build.sh

# With the deploy running:
rbnx caps | grep pick
rbnx ask "请把胡萝卜抓起来"
```

## License

MulanPSL-2.0.
