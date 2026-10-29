# Piper Package Catalog 接入与提交准备

源码仓库：<https://github.com/syswonder/robot-agilex-piper-rk3588>，默认分支 `main`。
Catalog 收录通过官方仓库的 PR 提交；源码及资源保留在本源码仓库。

本项目按 [官方 catalog](https://github.com/syswonder/robonix-package-catalog)
的当前规则接入：整机清单必须在仓库根目录，普通包可通过 catalog `path:` 收录同一
仓库的子目录。Catalog PR 只修改其根目录 `catalog.yaml`；清单、README、契约与资源
都放在本整机仓库，由官方 CI 读取默认分支。整机封面使用 `assets/promo/01.jpg`（1600×1000、8-bit RGB JPEG、低于 2 MB）。

根入口为 `robonix_manifest.yaml`，发布名 `robonix.robot.agilex.piper_rk3588`。
机器专用参数保留在忽略发布的 `robonix_manifest.local.yaml`；不提交本地密钥。
新入口的相对包路径全在本仓库内，Soma 使用根目录 `soma.yaml`、
`config/soma.yaml` 和 `urdf/piper.urdf`。

## 收录范围

本次使用 **A whole robot**，只在 `robots:` 中登记
`robonix.robot.agilex.piper_rk3588`。下列依赖随整机源码一起提供，
通过仓库内相对路径解析，不另外新增 `packages:` 条目。

为了保留本项目修改、不覆盖已收录的上游包，10 个 Robonix 依赖的 `package.name`
使用 `.piper_rk3588` 后缀。provider ID、能力 contract ID 和调用路径不变。
包维护者信息保留已有署名，整机 catalog 同时列出仓库维护者 kaileliu。

| 目录（相对仓库根） | 用途 | 接口 |
| --- | --- | --- |
| primitives/primitive-orbbec-dabai_dcw-camera-rbnx | Gemini 336L，部署覆盖 launch 文件 | ROS 2 图像/内参，gRPC 生命周期 |
| primitives/primitive-agilex-piper-arm-rbnx | Piper CAN 控制及反馈 | ROS 2，gRPC 生命周期 |
| primitives/primitive-agilex-piper-description-rbnx | URDF、TF 发布 | 内部 ROS 2 TF，不声明路由能力 |
| primitives/primitive-audio-mic-rbnx | 本机 ALSA 麦克风 | gRPC |
| primitives/primitive-audio-client-bridge-rbnx | 操作员端麦克风/扬声器桥 | gRPC，内部 WebSocket |
| services/service-object-detect-rbnx | HTTP YOLO / VLM 检测适配 | MCP，gRPC 生命周期 |
| services/service-grasp-pose-rbnx | 像素到抓取位姿 | gRPC |
| services/service-roboarm-ik-rbnx | Python IK 及动作执行 | gRPC |
| services/service-speech | ASR/TTS/对话 | gRPC，MCP 的 speak/list_speakers |
| skills/skill-pick-vertical-grasp-piper-rbnx | pick/demo/grab/put_in_box/put_down | MCP，gRPC 生命周期 |
| yolo_npu | RK3588 NPU HTTP 推理后端 | 宿主 HTTP 支持进程，无 Atlas 能力 |

完整机器可通过文字命令使用抓取链路；语音需另外启动宿主 FunASR。音频桥与本机
麦克风同时存在时，客户端必须明确选定输入 provider。NPU 包在宿主启动，不加入
容器内 `rbnx service:` 列表：HTTP 不是 catalog 接受的 Robonix contract transport，
因此它准确声明 `capabilities: []`，Robonix-facing 接口由检测服务提供。

## 从源码安装

1. 在 RK3588 Linux ARM64 主机安装 Docker、Python 3.10、uv、Git，确认 RKNPU 驱动
   和设备可用。获取本仓库的完整源码，模型与 SDK 二进制随源码保留。
2. 按 `vendor/PINS.md` 获取固定版本 Robonix 及子模块，再用
   `bash containers/piper/run.sh -b -e 'rbnx --version'` 构建容器；已有对应镜像可复用。
   包内 build 脚本负责 ROS、Python 依赖和 codegen。
3. 在宿主执行 `bash yolo_npu/setup_runtime.sh`，然后在独立终端运行
   `bash yolo_npu/run_yolo_npu.sh`。用 `curl -fsS http://127.0.0.1:8770/health`
   核对设备为 `rk3588-npu`。不要同时运行另一个占用 8770 的检测服务。
4. `cp robonix_manifest.yaml robonix_manifest.local.yaml`，
   `chmod 600 robonix_manifest.local.yaml`。编辑这个完整清单副本中的 LLM 占位值、
   USB-CAN 接口、相机 launch/分辨率、标定矩阵及放置/观察位参数。
   `PIPER_WORKSPACE=/root/workspace` 是容器挂载路径；若直接在宿主运行需改为实际绝对
   根路径。模板的矩阵与姿态来自历史部署，不能用于替代新安装的标定。
5. 使用标定工具完成并通过独立验证后，在现场确认 CAN 的 1 Mbps 比特率、使能策略与
   电机反馈。根清单默认 `can0`、`auto_can_setup: false`、`auto_enable: false`。
   允许驱动自动使能时才把 `auto_enable` 改为 true。IK 初始化可回观察位，boot 不是
   纯读取操作。确认工作区清空后再运行：

   ```bash
   bash containers/piper/run.sh -e 'cd /root/workspace && rbnx build -f robonix_manifest.local.yaml'
   bash containers/piper/run.sh -e 'cd /root/workspace && rbnx boot -f robonix_manifest.local.yaml --no-update-check --verbose'
   ```

语音的宿主安装、模型获取及启动见 `host-services/README.md` 和
`host-services/funasr/`。默认 TTS 使用静音 Mock 后端；配置真实 TTS 后才能获得语音
回复。LLM 密钥、网络代理与机器专用参数不进入公开源码。

## 提交前校验与源码导出

预检直接调用官方 builder 的严格新条目校验；只把 GitHub 文件读取适配为本地发布
文件清单。校验 metadata、名称唯一性、transport、包类型、契约、IDL、图片和整机
依赖路径。Robonix 全局契约使用本部署固定的 `vendor/robonix`；因此本地通过与远端
默认分支实际可访问是两项验证，不能据此声称远端 CI 已通过。

```bash
python3 -m venv /tmp/piper-catalog-venv
/tmp/piper-catalog-venv/bin/pip install -r catalog/requirements.txt
git clone https://github.com/syswonder/robonix-package-catalog /tmp/robonix-package-catalog
git -C /tmp/robonix-package-catalog checkout 65cf91c32acb52e0ec72ff7ef4a003c02c78f00f

# 校验当前公开源码仓库的提交准备。
/tmp/piper-catalog-venv/bin/python scripts/check_package_submission.py \
  --catalog-source /tmp/robonix-package-catalog \
  --repo https://github.com/syswonder/robot-agilex-piper-rk3588
/tmp/piper-catalog-venv/bin/python scripts/export_package_submission.py
```

生成 `dist/catalog/catalog.additions.yaml`、`dist/catalog/validation.json`、
`dist/piper-rk3588-source.tar.gz` 及逐文件 SHA256 清单。导出先使用 `catalog/publication.json` 的明确发布白名单，再应用 Git 的 ignore 规则，
不依赖当前目录本身是 Git 仓库，排除私密覆盖文件、日志、构建产物、Python 依赖缓存。
扫描常见密钥和私钥格式、拒绝越界符号链接及超过 GitHub 100 MiB 的单文件。
`dist/` 自身不会递归进入导出。导出不会更改源项目的 Git 状态。

发布后，本仓库 Actions 会以同一官方校验器执行预检并上传报告。
将 additions 中唯一的 `robots` 条目合并进 catalog 现有 `robots:` 列表，不能覆盖已有
列表或增加第二个顶层同名键。Catalog PR 只提交其 `catalog.yaml` 的这些新增条目。

当前源码仓库已经公开，校验默认使用真实仓库地址。新增条目生成于
`dist/catalog/catalog.additions.yaml`。提交前检查源码仓库默认分支，
再以真实 `--repo` 重新预检，提交仅修改 `catalog.yaml` 的 PR。
官方 CI 将使用 GitHub API 重新读取实际内容，包括当时的 Robonix 全局契约、文件树
和图片。源码仓库 Actions 通过与官方收录 PR 的 CI 通过是两项独立的检查。

## 验证边界

目录接入通过不等于实机抓取通过。当前现场手眼标定独立验证未通过，物体抓取尚未
验证；本次只做源码、目录规范和既有 NPU 后处理回归验证，不启动机械臂或改变设备
状态。现场记录保留在本机，不打包到公开发布文件。
