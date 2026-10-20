#!/usr/bin/env bash
# 在 scripts/docker/run_tests.sh 留下的容器中编译并运行闭环危害场景（假飞控 + 真实 BacktrackUseCase）。
# 用法：scripts/analysis/run_hazard_scenarios.sh [amd64|arm64] [场景编号 ...]
# 环境变量：DOCKER，同 run_tests.sh
set -euo pipefail

ARCH="${1:-amd64}"
shift || true
DOCKER="${DOCKER:-docker}"
CONTAINER="backtrack_noetic_${ARCH}"

if ! $DOCKER container inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "容器 $CONTAINER 不存在，请先运行 scripts/docker/run_tests.sh $ARCH" >&2
  exit 1
fi

$DOCKER exec "$CONTAINER" bash -lc '
  set -e
  source /opt/ros/noetic/setup.bash && source /ws/devel/setup.bash
  SRC=/ws/src/px4_backtrack_optimised
  g++ -O2 -std=c++17 "$SRC/scripts/analysis/hazard_scenarios.cpp" -o /ws/hazard_scenarios \
    -I"$SRC/include" -I/ws/src/trajectory_core/include -I/usr/include/eigen3 -I/opt/ros/noetic/include \
    -L/ws/devel/lib -L/opt/ros/noetic/lib \
    -lpx4_backtrack_optimised_lib -ltrajectory_core -lroscpp -lrosconsole -lrostime -lstdc++fs \
    -Wl,-rpath,/ws/devel/lib:/opt/ros/noetic/lib
  /ws/hazard_scenarios "$SRC/config/trajectory_config.yaml" "$@"
' hazard_scenarios "$@"
