# PX4 SITL（SIH，无 Gazebo）+ ROS Noetic MAVROS，用于在 SITL 中确认飞控行为、运行返航节点的场景。
FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-mavros ros-noetic-mavros-msgs \
      build-essential cmake ninja-build git file rsync unzip zip ccache \
      python3-pip python3-dev python3-setuptools python3-wheel \
      libxml2-dev libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# PX4 v1.15.4 编译 SITL 需要的 Python 包（Tools/setup/requirements.txt 的子集），
# 以及检查脚本读 ULog 用的 pyulog
RUN pip3 install --no-cache-dir \
      empy==3.3.4 jinja2 pyyaml jsonschema kconfiglib future pyros-genmsg \
      packaging toml numpy lxml pyulog

# MAVROS 需要的地理数据
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# 源码从主机挂载，属主与容器用户不同；PX4 编译时要用 git describe 取版本
RUN git config --global --add safe.directory '*'

# 在同一个容器里编译、运行返航节点（SITL 场景，方案第 14.3 节）
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-tf2 ros-noetic-visualization-msgs libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /px4
