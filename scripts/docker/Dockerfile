# ROS Noetic（Ubuntu 20.04，GCC 9.4，CMake 3.16），与机载 RK3588 的软件环境一致。
# arm64 由 run_tests.sh 传入单架构的 arm64v8/ros；旧构建器会忽略 --platform，多架构标签会被解析成本机架构
ARG BASE=ros:noetic-ros-base-focal
FROM ${BASE}
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-tf2 ros-noetic-visualization-msgs \
      libeigen3-dev python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
