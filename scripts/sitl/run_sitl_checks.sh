#!/usr/bin/env bash
# 在 Docker 中编译 PX4 SITL（SIH 四旋翼，无 Gazebo），配合 MAVROS 确认重构方案依赖的
# 飞控行为（系统性分析 N1、N2）。
# 用法：scripts/sitl/run_sitl_checks.sh [build|check|all]，默认 all
# 环境变量：
#   PX4_REPO          本地 PX4-Autopilot 仓库，默认与本仓库同级；只从中克隆，不改动
#   PX4_REF           PX4 版本，默认 v1.15.4（机载版本）
#   PX4_SITL_WORKDIR  克隆与编译目录，默认 ~/.cache/backtrack_px4_sitl（编译产物属主为 root）
#   DOCKER            docker 命令，需要 sudo 时设为 "sudo docker"
#   JOBS              编译并行数，默认 2
# 退出码：0 全部符合；1 与预期不符；2 起飞等准备失败；3 SIH 估计高度漂移把飞机带离地面，
# 本次 N2 无效，重跑即可
set -euo pipefail

STAGE="${1:-all}"
DOCKER="${DOCKER:-docker}"
JOBS="${JOBS:-2}"
PX4_REF="${PX4_REF:-v1.15.4}"
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
PX4_REPO="${PX4_REPO:-$(dirname "$REPO_ROOT")/PX4-Autopilot}"
WORK="${PX4_SITL_WORKDIR:-${XDG_CACHE_HOME:-$HOME/.cache}/backtrack_px4_sitl}"
SRC="$WORK/PX4-Autopilot"
IMAGE="backtrack-px4-sitl:${PX4_REF}"
CONTAINER="backtrack_px4_sitl"

# SITL 用到的子模块；NuttX、Gazebo 等不需要的不拉
SUBMODULES=(
  src/modules/mavlink/mavlink
  src/drivers/gps/devices
  src/lib/events/libevents
  src/lib/heatshrink/heatshrink
  src/lib/crypto/monocypher
  src/lib/crypto/libtomcrypt
  src/lib/crypto/libtommath
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
  src/lib/cdrstream/cyclonedds
  src/lib/cdrstream/rosidl
  src/modules/zenoh/zenoh-pico
)

prepare_source() {
  mkdir -p "$WORK"
  if [ ! -d "$SRC/.git" ]; then
    git clone -q --no-checkout "$PX4_REPO" "$SRC"
  fi
  git -C "$SRC" checkout -q -f "$PX4_REF"
  git -C "$SRC" submodule update --init --recursive --jobs 4 -- "${SUBMODULES[@]}"
  echo "PX4: $PX4_REF @ $(git -C "$SRC" rev-parse --short HEAD)"
}

start_container() {
  if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
    $DOCKER build -t "$IMAGE" "$HERE"
  fi
  if [ "$($DOCKER inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    $DOCKER rm -f "$CONTAINER" >/dev/null 2>&1 || true
    $DOCKER run -d --init --name "$CONTAINER" \
      -v "$SRC":/px4 -v "$HERE":/checks:ro \
      "$IMAGE" sleep infinity >/dev/null
  fi
}

build_px4() {
  $DOCKER exec "$CONTAINER" bash -lc "
    set -e
    cd /px4
    make px4_sitl_default j=$JOBS > /px4/build_sitl.log 2>&1 || { tail -40 /px4/build_sitl.log; exit 1; }
    ls -l build/px4_sitl_default/bin/px4"
}

run_checks() {
  $DOCKER exec "$CONTAINER" bash -lc '
    source /opt/ros/noetic/setup.bash
    pkill -f "^python3 /checks/check_fcu_behavior.py" || true
    pkill -x px4 || true; pkill -x roslaunch || true; sleep 1
    rm -rf /px4/build/px4_sitl_default/rootfs/eeprom /px4/build/px4_sitl_default/rootfs/parameters*.bson
    cd /px4/build/px4_sitl_default/rootfs
    PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ../bin/px4 -d > /tmp/px4.log 2>&1 &
    roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580 > /tmp/mavros.log 2>&1 &
    sleep 5
    status=0
    python3 /checks/check_fcu_behavior.py || status=$?
    pkill -x roslaunch || true; pkill -x px4 || true; sleep 2
    ulog_status=0
    python3 /checks/check_fcu_behavior.py --ulog "$(ls -t log/*/*.ulg | head -1)" || ulog_status=$?
    [ "$status" -ne 0 ] || status=$ulog_status
    exit $status'
}

case "$STAGE" in
  build) prepare_source; start_container; build_px4 ;;
  check) start_container; run_checks ;;
  all)   prepare_source; start_container; build_px4; run_checks ;;
  *) echo "用法：$0 [build|check|all]" >&2; exit 2 ;;
esac
