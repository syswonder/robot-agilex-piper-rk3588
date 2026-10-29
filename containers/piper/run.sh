#!/usr/bin/env bash
# Build & run the ARM64 robonix-core container for the Piper grasp deploy.
#
# robonix 源码**烤进镜像**（构建时 COPY + cargo build，见 docs/ARCHITECTURE.md
# 决策 1），运行时不再挂载 vendor/robonix —— 宿主上改它容器不会实时反映，需要
# 时临时加 -v 覆盖（开发用）。deploy 仓库本身仍挂载（可写，含配置与状态）。
#
# The container is created DETACHED (runs `sleep infinity`) so it persists
# across detaches; both an interactive shell and one-off automation commands
# attach to it via `docker exec`. This is what lets Phase 1 build/verify run
# non-interactively (a plain `docker run -it` needs a TTY and can't be scripted).
#
# Usage:
#   ./run.sh                 # build image if missing, ensure container, attach (interactive)
#   ./run.sh -b              # force rebuild image, ensure container, attach (interactive)
#   ./run.sh -e "CMD …"      # ensure container, run CMD non-interactively (env pre-sourced)
#   ./run.sh -b -e "CMD …"   # rebuild, then run CMD non-interactively
#   ./run.sh -d              # delete the container
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"     # <repo>/containers/piper
DEPLOY_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"    # <repo>
IMAGE_NAME="${IMAGE_NAME:-piper-robonix:std-humble-arm64}"
CONTAINER_NAME="${CONTAINER_NAME:-robonix_yolo_dev}"
ROS_BASE_IMAGE="${ROS_BASE_IMAGE:-ros:humble-ros-base}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

FORCE_BUILD=false
EXEC_MODE=false
EXEC_CMD=""
while [ $# -gt 0 ]; do
    case "$1" in
        -b|--build)  FORCE_BUILD=true; shift ;;
        -d|--delete) docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
                     echo "[*] removed container $CONTAINER_NAME"; exit 0 ;;
        -e|--exec)   EXEC_MODE=true; shift; EXEC_CMD="$*"; break ;;
        -h|--help)   echo "Usage: $0 [-b|--build] [-e|--exec \"CMD …\"] | -d|--delete"; exit 0 ;;
        *)           echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── Build image if missing (or forced) ─────────────────────────────────
if $FORCE_BUILD || ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    # robonix 源码烤进镜像，所以构建时需要 vendor/robonix（见 vendor/PINS.md）。
    if [ ! -d "$DEPLOY_ROOT/vendor/robonix" ]; then
        echo "ERROR: vendor/robonix 不存在——镜像构建需要它（获取方式见 vendor/PINS.md）" >&2
        exit 1
    fi
    echo "[*] Building $IMAGE_NAME (base=$ROS_BASE_IMAGE) …"
    # --network=host：RK3588 内核的 netfilter 只编了 filter/nat/mangle，没有 raw 表，
    # 而 Docker 29 的 direct-access-filtering 要在 raw 表给 bridge 加 DROP 规则，
    # 走默认 bridge 会直接 "Table does not exist" 构建失败。容器运行时本来就
    # --net host（见下方 docker run），构建期的 apt/pip/cargo 下载走 host 网络即可。
    docker build --network=host -f "$SCRIPT_DIR/Dockerfile" --build-arg "ROS_BASE_IMAGE=$ROS_BASE_IMAGE" \
        -t "$IMAGE_NAME" "$DEPLOY_ROOT"
fi

# ── Ensure a detached, persistent container exists and is running ──────
# --net host: ROS2/zenoh/gRPC discovery, host CAN interfaces (piper_ctl, later),
#             and reaching the host-side YOLO inference service (decision 乙).
# No --gpus: this container is CPU-only by design.
# robonix 源码在镜像里（可写），primitives/、services/、skills/ 和 config/ 由仓库挂载提供。
# PID 1 is `sleep infinity`, so the container stays up after we detach; we
# attach with `docker exec` (interactive shell, or one-off command below).
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    MOUNT_ROOT="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/root/workspace"}}{{.Source}}{{end}}{{end}}' "$CONTAINER_NAME")"
    if [[ "$MOUNT_ROOT" != "$DEPLOY_ROOT" ]]; then
        echo "ERROR: $CONTAINER_NAME mounts $MOUNT_ROOT; expected $DEPLOY_ROOT" >&2
        exit 1
    fi
    docker start "$CONTAINER_NAME" >/dev/null 2>&1 || true
else
    echo "[*] Creating detached container $CONTAINER_NAME …"
    # USB passthrough for the Orbbec camera (canonical OrbbecSDK-in-docker
    # recipe): the SDK opens the device over libusb (char major 189) and the
    # color UVC nodes (major 81). --device-cgroup-rule grants the container
    # permission to those majors; -v /dev:/dev makes the nodes visible and
    # survives the device re-enumeration Orbbec does on stream start. This is
    # narrower than --privileged (no extra capabilities). If a future camera
    # still won't open, --privileged is the fallback.
    # NOTE: host CAN (piper_ctl's can_piper) needs no device flag — SocketCAN
    # lives in the network namespace, already shared via --net host.
    # Audio (USB mic for the voice/ASR path): -v /dev:/dev already exposes the
    # /dev/snd nodes, but Docker MASKS /proc/asound by default, so ALSA's
    # snd_card enumeration finds "no soundcards" and arecord -D hw:N,0 fails
    # with "Cannot get card index". Bind-mounting the real /proc/asound over
    # the mask lets ALSA resolve card indices. char major 116 = ALSA devices.
    docker run -d \
        --name "$CONTAINER_NAME" \
        --hostname piper-robonix \
        --net host \
        --device-cgroup-rule='c 81:* rmw' \
        --device-cgroup-rule='c 189:* rmw' \
        --device-cgroup-rule='c 116:* rmw' \
        -v /dev:/dev \
        -v "$DEPLOY_ROOT:/root/workspace" \
        -v "$SCRIPT_DIR/docker-entrypoint.sh:/docker-entrypoint.sh:ro" \
        -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
        -e ROBONIX_META_GRPC_ADDR="0.0.0.0:50051" \
        -e ROBONIX_META_GRPC_ENDPOINT="127.0.0.1:50051" \
        "$IMAGE_NAME" \
        sleep infinity >/dev/null
fi

# ── Dispatch: one-off command (non-interactive) or interactive shell ───
# The entrypoint sources ROS + Rust env, then either execs the passed command
# or drops into an interactive bash when none is given.
if $EXEC_MODE; then
    exec docker exec "$CONTAINER_NAME" bash /docker-entrypoint.sh bash -c "$EXEC_CMD"
fi

echo "[*] Attaching to $CONTAINER_NAME"
exec docker exec -it "$CONTAINER_NAME" bash /docker-entrypoint.sh
