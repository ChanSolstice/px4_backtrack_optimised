#!/usr/bin/env bash
# 在 ROS Noetic 容器中编译本仓库和 trajectory_core，并运行两者的全部单测。
# 用法：scripts/docker/run_tests.sh [amd64|arm64] [trajectory_core 的 git 引用，默认 origin/dev]
# 环境变量：
#   TRAJECTORY_CORE_REPO  trajectory_core 仓库路径，默认与本仓库同级
#   DOCKER                docker 命令，默认 docker；需要 sudo 时设为 "sudo docker"
#   JOBS                  编译并行数，默认 1（内存小的机器上更稳）
# arm64 通过 QEMU 模拟（需要 qemu-user-static），与机载架构一致，但编译明显更慢。
# 结束后保留容器，scripts/bench/run_gen_bench.sh 会复用其中的编译结果。
set -euo pipefail

ARCH="${1:-amd64}"
CORE_REF="${2:-origin/dev}"
DOCKER="${DOCKER:-docker}"
JOBS="${JOBS:-1}"
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
CORE_REPO="${TRAJECTORY_CORE_REPO:-$(dirname "$REPO_ROOT")/trajectory_core}"
CORE_EXPORT="${TMPDIR:-/tmp}/backtrack_trajectory_core_${ARCH}"
IMAGE="backtrack-noetic:${ARCH}"
CONTAINER="backtrack_noetic_${ARCH}"

# 导出算法库的指定版本，不改动其仓库的检出状态
rm -rf "$CORE_EXPORT" && mkdir -p "$CORE_EXPORT"
git -C "$CORE_REPO" archive "$CORE_REF" | tar -x -C "$CORE_EXPORT"
echo "trajectory_core: $CORE_REF @ $(git -C "$CORE_REPO" rev-parse --short "$CORE_REF")"

if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1; then
  $DOCKER build --platform "linux/${ARCH}" -t "$IMAGE" "$HERE"
fi

$DOCKER rm -f "$CONTAINER" >/dev/null 2>&1 || true
$DOCKER run -d --platform "linux/${ARCH}" --name "$CONTAINER" \
  -v "$REPO_ROOT":/ws/src/px4_backtrack_optimised:ro \
  -v "$CORE_EXPORT":/ws/src/trajectory_core:ro \
  "$IMAGE" sleep infinity >/dev/null

$DOCKER exec -e JOBS="$JOBS" "$CONTAINER" bash -lc '
  set -e
  source /opt/ros/noetic/setup.bash && cd /ws
  echo "$(uname -m), $(gcc --version | head -1)"
  catkin_make -DCMAKE_BUILD_TYPE=Release -j"$JOBS" > /ws/build.log 2>&1 || { tail -40 /ws/build.log; exit 1; }
  catkin_make run_tests -j"$JOBS" > /ws/test.log 2>&1 || true
  # 失败时返回非零；汇总里的总数会重复计数，以各包的 Summary 为准
  catkin_test_results build/test_results
'
