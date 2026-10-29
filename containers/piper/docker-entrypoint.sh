#!/bin/bash
# Entry point for the Piper robonix-core container (arm64).
# Adapted from the robonix repo's own docker/docker-entrypoint.sh:
#   - dropped the x86-only osmesa LD_LIBRARY_PATH lines (no GPU rendering here)
#   - robonix 源码与编译产物由**镜像自带**（/opt/robonix，见 Dockerfile 与
#     docs/ARCHITECTURE.md 决策 1），不再依赖宿主挂载。
set -e

# Use this deployment's optional network settings for cloud model requests.
# The file lives in the mounted workspace, so existing containers also pick it up.
# config/ is the canonical location. Existing installations may keep the
# same optional private files under deploy/piper/ until they migrate them.
for settings in network.local.env tencent-asr.local.env; do
    if [ -r "/root/workspace/config/$settings" ]; then
        source "/root/workspace/config/$settings"
    elif [ -r "/root/workspace/deploy/piper/$settings" ]; then
        source "/root/workspace/deploy/piper/$settings"
    fi
done

mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root
export XDG_RUNTIME_DIR=/tmp/runtime-root

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$HOME/.cargo/env" 2>/dev/null || true

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROBONIX_META_GRPC_ADDR="${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"
# rmw_zenoh clients retry finding the router this many times before proceeding
# without discovery — covers a node starting a hair before the router binds.
export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
# robonix 源码在 /opt/robonix/src（镜像自带，与 /root/workspace 的仓库挂载互不干扰；
# 烤在 /root/workspace 下会被挂载盖掉——这是踩过的坑）。
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/opt/robonix/build/target}"
# robonix core binaries (rbnx + the 5 system builtins) are `cargo build`-ed into
# $CARGO_TARGET_DIR/release, which is host-persisted (./.robonix-build) and so
# survives container re-creation. `cargo install` copies them to ~/.cargo/bin,
# but that lives in the container's writable layer and is LOST when the container
# is recreated (e.g. to add the /dev device mount). Putting release/ on PATH is
# both faster (no recompile) and durable across re-creation.
export PATH="$CARGO_TARGET_DIR/release:$PATH"

if ! grep -q "### ROBONIX_DOCKER_ENV ###" ~/.bashrc 2>/dev/null; then
cat <<'EOF' >> ~/.bashrc
### ROBONIX_DOCKER_ENV ###
source /opt/ros/humble/setup.bash
source "$HOME/.cargo/env" 2>/dev/null || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROBONIX_META_GRPC_ADDR="${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"
export ROBONIX_META_GRPC_ENDPOINT="${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"
export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/opt/robonix/build/target}"
export PATH="$CARGO_TARGET_DIR/release:$PATH"
EOF
fi

if [ -f /usr/share/bash-completion/bash_completion ]; then
    # shellcheck disable=SC1091
    . /usr/share/bash-completion/bash_completion
fi

# ── Zenoh router (rmw_zenoh_cpp discovery daemon) ──────────────────────
# Under rmw_zenoh_cpp, ROS 2 nodes in separate processes only discover each
# other through a running rmw_zenohd router. The robonix core (Rust gRPC) does
# not need it — that's why the Phase-1 system-only boot worked without one — but
# the real pipeline's ROS nodes do: each primitive wrapper (camera, arm driver)
# subscribes to a topic that the node it spawned publishes, and with no router
# those two processes never find each other ("no Image/JointState within 30s").
# Start ONE router, detached so it outlives this exec session and is shared by
# every shell; the :7447 listen check stops repeat execs from spawning duplicates.
start_zenoh_router() {
    [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ] || return 0
    local bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
    [ -x "$bin" ] || { echo "[entrypoint] WARN: rmw_zenohd missing at $bin — ROS discovery may fail"; return 0; }
    # Idempotent guard: a router is "already up" iff something is LISTENING on
    # :7447. Do NOT pgrep the cmdline for "$bin" — pgrep -f matches any process
    # whose args merely contain this path (e.g. the `bash /docker-entrypoint.sh
    # rbnx boot …` wrapper, or a shell running a command that names it), so it
    # false-matches and we skip starting a router that isn't actually there.
    if python3 -c 'import socket; socket.create_connection(("127.0.0.1",7447),0.2).close()' 2>/dev/null; then
        return 0
    fi
    setsid "$bin" >/tmp/rmw_zenohd.log 2>&1 </dev/null &
    echo "[entrypoint] started rmw_zenohd (pid $!) → /tmp/rmw_zenohd.log"
    # Wait (bounded) for it to bind :7447 so the first ROS node connects on try 1.
    for _ in $(seq 1 20); do
        python3 -c 'import socket; socket.create_connection(("127.0.0.1",7447),0.2).close()' 2>/dev/null && break
        sleep 0.25
    done
}
start_zenoh_router

# ── robonix source registration ────────────────────────────────────────
# `rbnx setup <src>` writes ~/.robonix/config.yaml (robonix_source_path). Like
# ~/.cargo/bin, that file lives in the writable layer and is lost on re-create,
# so regenerate it if missing. Cheap (just writes the config; no build), and
# safe to repeat. Without it, `rbnx boot` can't resolve the source tree.
ensure_robonix_setup() {
    command -v rbnx >/dev/null 2>&1 || return 0
    [ -f "$HOME/.robonix/config.yaml" ] && return 0
    local src=/opt/robonix/src
    [ -d "$src" ] || return 0
    echo "[entrypoint] registering robonix source: rbnx setup $src"
    rbnx setup "$src" || echo "[entrypoint] WARN: rbnx setup failed"
}
ensure_robonix_setup

# Dual purpose: with args, run them non-interactively (env already sourced
# above) — this is how automation / `run.sh --exec` drives the container.
# With no args, print the banner and drop into an interactive shell.
if [ "$#" -gt 0 ]; then
    exec "$@"
fi

echo -e "[*] \033[1mPiper robonix docker\033[0m  ROS2 \033[33m${ROS_DISTRO}\033[0m  ($(uname -m))"
echo "[*] RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "[*] robonix source (ro): /opt/robonix/src  (自带于镜像，非挂载)"
echo "[*] deploy root   (rw): /root/workspace"
echo "[*] CARGO_TARGET_DIR=$CARGO_TARGET_DIR"
exec bash
