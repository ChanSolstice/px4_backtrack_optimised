#!/usr/bin/env bash
# 在 scripts/docker/run_tests.sh 留下的容器中运行错误注入检查（检验单测能否发现典型错误）。
# 用法：scripts/analysis/run_mutation_check.sh [amd64|arm64] [组件或注入名 ...]
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

$DOCKER exec "$CONTAINER" python3 /ws/src/px4_backtrack_optimised/scripts/analysis/mutation_check.py "$@"
