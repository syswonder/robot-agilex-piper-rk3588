# ARM64 / Jetson Thor adaptation of the robonix repo's docker/Dockerfile.
#
# Purpose: provide a ROS 2 Humble + Rust + gRPC toolchain so we can build and
# boot the robonix CORE inside a container on aarch64.
#
# Deliberately NO GPU / deep-learning here. Per deploy decision "乙", YOLO (and
# later VLA) inference runs on the HOST (roboarm/.venv, native CUDA) as a
# network service that the pipeline calls over --net host. So this container
# never needs CUDA / torch / lerobot / mujoco — exactly the parts of the
# upstream x86 Dockerfile that do not exist / do not build on Thor.
#
# Base: ros:humble-ros-base (arm64, headless, already present locally).
# NOTE: osrf/ros:humble-desktop is amd64-only, so it cannot run on aarch64.
# Pose solving is pure-Python IK (roboarm_ik), so no MoveIt/rviz is needed.
ARG ROS_BASE_IMAGE=ros:humble-ros-base
FROM ${ROS_BASE_IMAGE}

# ── TUNA mirrors (apt: ubuntu-ports for arm64 + ROS 2) ─────────────────
# ros2.sources may be a symlink into /usr/share/ros-apt-source; follow it.
RUN set -eux; \
    find /etc/apt/sources.list.d/ -maxdepth 1 \( -name '*.list' -o -name '*.sources' \) -print0 \
      | xargs -0 -r sed -i --follow-symlinks \
          -e 's|http://packages\.ros\.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
          -e 's|^Types: deb deb-src$|Types: deb|'; \
    if [ -f /etc/apt/sources.list ]; then \
        sed -i --follow-symlinks \
            -e 's|http://archive\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://security\.ubuntu\.com/ubuntu/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' \
            -e 's|http://ports\.ubuntu\.com/ubuntu-ports/\?|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|g' \
            /etc/apt/sources.list; \
    fi

ENV PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    UV_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    HF_ENDPOINT=https://hf-mirror.com

ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp

# ── ROS middleware + build/dev toolchain (all arm64-available) ─────────
# iproute2 (`ip`) is an UNDECLARED runtime dep of piper_ctl: piper_sdk's
# C_STD_CAN.get_can_bitrate() shells out to `ip -details link show <can>`
# to verify the bus is 1 Mbps before opening it (can_encapsulation_v0_4_0.py).
# Without `ip` it throws FileNotFoundError → CAN_BITRATE_ERR → the arm driver
# refuses to connect ("no JointState within 30s"), even though the AF_CAN data
# path itself works. The proven host stack has iproute2; our slim base did not.
# alsa-utils provides `arecord`, the capture backend for the mic primitive
# (primitives/primitive-audio-mic-rbnx → robonix/primitive/audio/mic, the voice
# path's audio source). The mic driver shells out to
# `arecord -D plughw:2,0 -f S16_LE -r 16000 -c 1 -t raw`; the USB mic's
# /dev/snd nodes are already visible via -v /dev:/dev (see docker/run.sh).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates wget gnupg curl git \
    iproute2 \
    alsa-utils \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-zenoh-cpp \
    ros-humble-test-msgs \
    python3-pip \
    python3-grpcio \
    python3-protobuf \
    python3-dev \
    python3-vcstool \
    build-essential \
    protobuf-compiler \
    libeigen3-dev \
    libclang-dev \
    bash-completion \
    python-is-python3 \
 && rm -rf /var/lib/apt/lists/*

# ── ROS build deps for the grasp packages (pick_ok / roboarm_ik arch) ──
# MoveIt is GONE: pose solving moved to service-roboarm-ik-rbnx, a pure-Python
# kinpy+scipy IK service (no move_group, no C++ planning node). So the whole
# ros-humble-moveit* / OMPL stack is dropped.
# The remaining keys are rosdep-resolved for the surviving ROS packages:
#   orbbec_camera (image/camera_info/cv_bridge), piper arm+description
#   (joint_state_publisher, controller_manager), and roboarm_ik's colcon
#   msg packages (piper_msgs, graspnet_msgs). tf2 kept for TF lookups.
# GUI/sim keys (rviz, gazebo) are omitted — headless deploy.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-controller-manager \
    ros-humble-image-publisher \
    ros-humble-cv-bridge \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    libgflags-dev \
    libgoogle-glog-dev \
    nlohmann-json3-dev \
    python3-opencv \
 && rm -rf /var/lib/apt/lists/*

# ── Python gRPC tooling + zenoh + grasp-pipeline Python deps ───────────
# setuptools pinned <80: colcon --symlink-install runs `setup.py develop`
# for ament_python packages, a command setuptools removed in 80+. The
# >=77 floor keeps grpcio-tools happy. openai → object_detect VLM fallback,
# fastmcp → pick_skill (MCP server). kinpy → roboarm_ik FK chain
# (kinpy.build_serial_chain_from_urdf); scipy → its SLSQP IK solver.
# (transforms3d dropped — easy_handeye2 is gone in the roboarm_ik arch.)
# piper_sdk → piper_ctl arm driver (piper_ctrl_single_node.py does
#   `from piper_sdk import C_PiperInterface_V2`); it's an undeclared runtime dep
#   of piper_ctl_rbnx (its build.sh only colcon-builds the ROS msgs/node, never
#   installs the SDK). 0.6.1 is the version the proven roboarm stack runs
#   against this same physical Piper arm. Pulls in python-can as a dependency.
# scipy → also an undeclared runtime dep of piper_ctl (piper_ctrl_single_node.py
#   line 17: `from scipy.spatial.transform import Rotation` for Euler↔quaternion).
#   Same story as piper_sdk: never installed by piper_ctl_rbnx's build.sh.
# numpy PINNED <2 (1.26.4): the ROS Humble C-extensions (cv_bridge, sensor_msgs
#   typesupport, rclpy) are compiled against numpy 1.x. scipy/transforms3d
#   otherwise pull numpy 2.x, whose reorganized C-API makes those extensions
#   throw "numpy.core.multiarray failed to import" — which crashed llm_detect's
#   rclpy thread at CMD_INIT. 1.26.4 is the last 1.x and satisfies scipy 1.15.3.
# --timeout/--retries: the TUNA mirror occasionally stalls mid-wheel (scipy
# is ~30MB); without a longer timeout the layer fails on a transient read.
# mcp PINNED <2 (and fastmcp<4, which would otherwise pull mcp>=2):
#   robonix-api does `from mcp.server.fastmcp import FastMCP` (robonix_api/
#   capability.py); mcp 2.x renamed that to mcp.server.mcpserver.MCPServer.
#   Unpinned, pip resolves fastmcp 4.x -> mcp 2.x, and every package that
#   registers an MCP capability (camera, detector, …) dies at import with
#   "ModuleNotFoundError: No module named 'mcp.server.fastmcp'" — soma stage 1
#   then fails and the whole boot rolls back ("package exited before
#   registering with atlas"). The older image escaped this only because it was
#   built back when mcp 1.x was still what pip picked.
RUN pip3 install --no-cache-dir --timeout 120 --retries 5 \
    grpcio-tools "eclipse-zenoh>=1.2,<2" \
    "numpy==1.26.4" "setuptools>=77,<80" openai "fastmcp<4" "mcp<2" "piper_sdk==0.6.1" \
    "scipy==1.15.3" kinpy

# ── uv (Python package/venv manager) ───────────────────────────────────
# The speech service package (services/service-speech, added for voice
# control) is the first package whose build.sh is uv-managed: it does
# `uv venv` + `uv sync` to materialise an isolated rbnx-build/venv/, then
# `uv pip install` the sherpa-onnx wake-word wheels. The 7 grasp packages
# use colcon / system-pip and never needed uv, so the base image lacks it.
# Installed from the TUNA mirror (PIP_INDEX_URL above) for CN reliability.
# Lands on PATH at /usr/local/bin/uv.
RUN pip3 install --no-cache-dir --timeout 120 --retries 5 uv

# ── grpcurl (arm64) for gRPC acceptance checks — best-effort ───────────
# grpcurl is only a manual aid for poking the gRPC meta endpoint; it is NOT
# required to build or boot the robonix core. GitHub release downloads are
# flaky from CN (ghfast.top reset mid-transfer during the first build), so we
# try several mirrors with short timeouts and DO NOT fail the image build if
# they all miss — it can be dropped in later. See OPERATIONS.md §5.
RUN set -eux; \
    GRPCURL_VERSION=1.9.3; \
    F="grpcurl_${GRPCURL_VERSION}_linux_arm64.tar.gz"; \
    P="fullstorydev/grpcurl/releases/download/v${GRPCURL_VERSION}/${F}"; \
    ok=0; \
    for base in \
        "https://ghfast.top/https://github.com" \
        "https://gh-proxy.com/https://github.com" \
        "https://ghproxy.net/https://github.com" \
        "https://github.com" ; do \
        echo "grpcurl: trying ${base}/${P}"; \
        if curl -fsSL --connect-timeout 10 --max-time 120 "${base}/${P}" -o /tmp/grpcurl.tgz; then \
            tar -xzf /tmp/grpcurl.tgz -C /usr/local/bin grpcurl && ok=1 && break; \
        fi; \
    done; \
    rm -f /tmp/grpcurl.tgz; \
    if [ "$ok" = 1 ]; then grpcurl --version || true; \
    else echo "WARN: grpcurl not installed (all mirrors failed); non-essential for Phase 1, skipping"; fi

# ── Rust (needed to build robonix; Cargo.toml is edition 2024) ─────────
# rustup 走 TUNA；**crate 文件走 rsproxy**。
# 注意别用 TUNA 的 crates 源：它只镜像了索引，稀疏索引里 config.json 的 dl 仍指向
# static.crates.io，实际下载会落到境外——2026-09-11 构建就卡死在 time-0.3.53 上
# （transfer too slow，重试后失败）。rsproxy（字节）的索引里 dl 指向它自己，
# 实测 273 KB 的包 0.8s 拿到。
ENV RUSTUP_DIST_SERVER=https://mirrors.tuna.tsinghua.edu.cn/rustup \
    RUSTUP_UPDATE_ROOT=https://mirrors.tuna.tsinghua.edu.cn/rustup/rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
ENV PATH=/root/.cargo/bin:$PATH
RUN mkdir -p /root/.cargo && \
    printf '[source.crates-io]\nreplace-with = "rsproxy-sparse"\n\n[source.rsproxy-sparse]\nregistry = "sparse+https://rsproxy.cn/index/"\n' \
      > /root/.cargo/config.toml

WORKDIR /root/workspace

# ── robonix 框架源码：烤进镜像并编译 ────────────────────────────────────
# 见 docs/ARCHITECTURE.md 决策 1：robonix 极少更新 → 烤进镜像；packages 常变 → 挂载。
# 烤进来还消除了"只读挂载让 PEP 517 安装写 egg-info 失败"这类适配（容器内可写）。
# 需要构建上下文为仓库根：docker build -f containers/piper/Dockerfile <repo>
# 注意：必须烤到**挂载点之外**。运行时 <repo> 会挂到 /root/workspace，
# 烤在那里的东西会被整个盖掉（PATH 指向存在、文件却看不见）。
# 因此源码与编译产物都放 /opt/robonix（系统级位置，不与挂载冲突）。
ENV CARGO_TARGET_DIR=/opt/robonix/build/target
COPY vendor/robonix /opt/robonix/src
RUN cargo build --release --manifest-path /opt/robonix/src/Cargo.toml

# rbnx + 5 个 Rust 系统组件产物都在 $CARGO_TARGET_DIR/release；加进 PATH，
# 这样即使绕过 entrypoint（直接 docker exec）也能用 rbnx。
ENV PATH=/opt/robonix/build/target/release:$PATH

# 注册 robonix 源码路径（写 ~/.robonix/config.yaml）。源码位置在镜像里固定，
# 所以这一步在构建期做即可 —— 镜像自洽：不经 entrypoint 也能 `rbnx path ...`。
RUN rbnx setup /opt/robonix/src
