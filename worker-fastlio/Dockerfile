# Build context: repository root (see infra/docker-compose.yml)
#
# Separate ROS2/Ubuntu-based service for the real FAST-LIO2 LiDAR-IMU
# odometry engine. Cannot share the main `worker` image (python:3.12-slim
# on Debian bookworm) because FAST-LIO2 (hku-mars/FAST_LIO, ROS2 branch)
# requires ROS2 Humble on Ubuntu 22.04 + PCL + Eigen + livox_ros_driver2.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git \
        python3-pip python3-colcon-common-extensions \
        libpcl-dev libeigen3-dev \
        ros-humble-pcl-conversions ros-humble-pcl-ros \
        ros-humble-tf2-eigen \
    && apt-get clean

# Livox-SDK2 (C library, required by livox_ros_driver2's CMake build even
# though we only use livox_ros_driver2 for its CustomMsg .msg definition --
# our bridge publishes messages directly, it does not run the real Livox
# driver node against hardware).
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 \
    && cd /opt/Livox-SDK2 && mkdir build && cd build \
    && cmake .. && make -j"$(nproc)" && make install

WORKDIR /opt/fastlio_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    && git clone --depth 1 -b ROS2 https://github.com/hku-mars/FAST_LIO.git \
    && cd FAST_LIO && git submodule update --init --recursive

# livox_ros_driver2 ships its own bespoke build.sh (not a plain colcon
# package layout) that expects to be run from inside the package directory.
WORKDIR /opt/fastlio_ws/src/livox_ros_driver2
RUN bash -c "source /opt/ros/humble/setup.bash && ./build.sh humble"

WORKDIR /opt/fastlio_ws
RUN bash -c "source /opt/ros/humble/setup.bash && source /opt/fastlio_ws/src/livox_ros_driver2/../../install/setup.bash 2>/dev/null; colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release"

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv
RUN uv pip install --system --break-system-packages --no-cache celery[redis] boto3 pyyaml numpy rosbags laspy

WORKDIR /srv
COPY worker-fastlio/fastlio_bridge.py ./fastlio_bridge.py
COPY worker-fastlio/mid360_template.yaml ./mid360_template.yaml
COPY worker-fastlio/entrypoint.sh ./entrypoint.sh
RUN chmod +x ./entrypoint.sh

ENTRYPOINT ["/srv/entrypoint.sh"]