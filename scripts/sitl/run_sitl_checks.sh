#!/usr/bin/env bash
# 在 Docker 中编译 PX4 SITL（SIH 四旋翼，无 Gazebo），配合 MAVROS：
# - check：确认重构方案依赖的飞控行为（系统性分析 N1、N2）；
# - scenarios：在同一个容器里编译并运行返航节点，回放方案第 11 节中涉及飞控交互的场景和 F8（第 14.3 节）。
# 用法：scripts/sitl/run_sitl_checks.sh [build|check|all]，默认 all
#       scripts/sitl/run_sitl_checks.sh scenarios [场景编号 ...]，不带编号运行全部场景
# 环境变量：
#   PX4_REPO              本地 PX4-Autopilot 仓库，默认与本仓库同级；只从中克隆，不改动
#   PX4_REF               PX4 版本，默认 v1.15.4（机载版本）
#   PX4_SITL_WORKDIR      克隆与编译目录，默认 ~/.cache/backtrack_px4_sitl（编译产物属主为 root）
#   TRAJECTORY_CORE_REPO  trajectory_core 仓库路径，默认与本仓库同级；CORE_REF 为其引用，默认 origin/dev
#   S15_MINUTES           S15 的录制时长（分钟），默认 5
#   DOCKER                docker 命令，需要 sudo 时设为 "sudo docker"
#   JOBS                  编译并行数，默认 2
# 退出码：0 全部符合；1 与预期不符；2 起飞等准备失败；3 SIH 估计高度漂移把飞机带离地面，
# 本次 N2 无效，重跑即可。scenarios 中 SIH 偶尔传感器超时、飞控进入失效保护（与本节点无关），
# 该场景无效，自动重跑一次
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
CORE_REPO="${TRAJECTORY_CORE_REPO:-$(dirname "$REPO_ROOT")/trajectory_core}"
CORE_REF="${CORE_REF:-origin/dev}"
CORE_EXPORT="$WORK/trajectory_core"
IMAGE="backtrack-px4-sitl:${PX4_REF}"
CONTAINER="backtrack_px4_sitl"
SCENARIOS=(S1 S4 S6 S7 S11 S12 S13 S15 S18 S21 S26 S27 F8a F8b)

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

# 导出算法库的指定版本，不改动其仓库的检出状态；目录本身保留，容器里的挂载才不会失效
export_core() {
  mkdir -p "$CORE_EXPORT"
  find "$CORE_EXPORT" -mindepth 1 -delete
  git -C "$CORE_REPO" archive "$CORE_REF" | tar -x -C "$CORE_EXPORT"
  echo "trajectory_core: $CORE_REF @ $(git -C "$CORE_REPO" rev-parse --short "$CORE_REF")"
}

# 镜像按 Dockerfile 构建（已有的层走缓存）；镜像更新后重建容器
start_container() {
  local out
  out="$($DOCKER build -q -t "$IMAGE" "$HERE" 2>&1)" || { echo "$out"; exit 1; }
  mkdir -p "$CORE_EXPORT"
  local image_id
  image_id="$($DOCKER image inspect -f '{{.Id}}' "$IMAGE")"
  if [ "$($DOCKER inspect -f '{{.State.Running}} {{.Image}}' "$CONTAINER" 2>/dev/null)" != "true $image_id" ]; then
    $DOCKER rm -f "$CONTAINER" >/dev/null 2>&1 || true
    $DOCKER run -d --init --name "$CONTAINER" \
      -v "$SRC":/px4 -v "$HERE":/checks:ro \
      -v "$REPO_ROOT":/ws/src/px4_backtrack_optimised:ro -v "$CORE_EXPORT":/ws/src/trajectory_core:ro \
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

build_node() {
  $DOCKER exec "$CONTAINER" bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash && cd /ws
    catkin_make -DCMAKE_BUILD_TYPE=Release -j$JOBS > /ws/build.log 2>&1 || { tail -40 /ws/build.log; exit 1; }"
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

# 每个场景重新启动 PX4、MAVROS 和返航节点；日志在容器的 /tmp/sitl/<场景>/
run_scenario() {
  $DOCKER exec -e S15_MINUTES="${S15_MINUTES:-5}" "$CONTAINER" bash -lc "
    source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
    pkill -x roslaunch || true; pkill -x px4 || true; sleep 2
    dir=/tmp/sitl/$1 && rm -rf \$dir && mkdir -p \$dir
    rm -rf /px4/build/px4_sitl_default/rootfs/eeprom /px4/build/px4_sitl_default/rootfs/parameters*.bson
    cd /px4/build/px4_sitl_default/rootfs
    PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim ../bin/px4 -d > \$dir/px4.log 2>&1 &
    roslaunch /checks/backtrack_sitl.launch log_dir:=\$dir > \$dir/ros.log 2>&1 &
    sleep 5
    status=0
    python3 /checks/sitl_scenarios.py $1 \$dir || status=\$?
    pkill -x roslaunch || true; pkill -x px4 || true; sleep 2
    if [ \$status -ne 0 ] && grep -q 'No valid data from' \$dir/px4.log; then
      echo 'PX4 SIH 传感器超时，飞控进入失效保护（与本节点无关），本次无效'
      status=3
    fi
    exit \$status"
}

run_scenarios() {
  local status=0 failed=() code
  for s in "$@"; do
    code=0
    run_scenario "$s" || code=$?
    if [ "$code" -eq 3 ]; then
      echo "重跑 $s"
      code=0
      run_scenario "$s" || code=$?
    fi
    [ "$code" -eq 0 ] || { status=1; failed+=("$s"); }
  done
  echo
  if [ "$status" -eq 0 ]; then
    echo "SITL 场景 $# 个，全部通过"
  else
    echo "SITL 场景 $# 个，未通过：${failed[*]}"
  fi
  return $status
}

case "$STAGE" in
  build) prepare_source; start_container; build_px4 ;;
  check) start_container; run_checks ;;
  all)   prepare_source; start_container; build_px4; run_checks ;;
  scenarios)
    shift
    [ $# -gt 0 ] || set -- "${SCENARIOS[@]}"
    export_core; start_container; build_node; run_scenarios "$@" ;;
  *) echo "用法：$0 [build|check|all] 或 $0 scenarios [场景编号 ...]" >&2; exit 2 ;;
esac
