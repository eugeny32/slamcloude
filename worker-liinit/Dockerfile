# Isolated ROS1 environment for hku-mars/LiDAR_IMU_Init ("LI-Init") --
# independently (re-)computes LiDAR-IMU extrinsic + time offset from raw
# sensor data, to cross-check the values worker-fastlio currently takes at
# face value from the S20's calibration.yaml. ROS1-only tool (catkin,
# livox_ros_driver v1, not the v2/ROS2 driver worker-fastlio uses), so it
# cannot share worker-fastlio's ROS2 image -- same reasoning as
# worker-gtsam's numpy<2 isolation, just for a ROS distro instead of a
# Python package.
FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libpcl-dev libeigen3-dev \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
    ros-noetic-eigen-conversions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ceres-solver 2.0.0 exactly, as specified by LI-Init's README (apt's version
# in focal is older and untested against this code).
RUN git clone --depth 1 --branch 2.0.0 https://github.com/ceres-solver/ceres-solver.git /tmp/ceres-solver \
    && mkdir /tmp/ceres-solver/build && cd /tmp/ceres-solver/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make -j"$(nproc)" \
    && make install \
    && rm -rf /tmp/ceres-solver

RUN pip3 install --no-cache-dir numpy rosbags

WORKDIR /root/catkin_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone --depth 1 https://github.com/hku-mars/LiDAR_IMU_Init.git

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j\"$(nproc)\""

COPY liinit_bridge.py /root/catkin_ws/liinit_bridge.py
# livox_mid360.launch hardcodes loading config/mid360.yaml -- overwrite the
# stock file (which targets a different rig with an external MAVROS IMU on
# /mavros/imu/data_raw, not this device's built-in Livox IMU) rather than
# adding a same-named file the launch file would never load.
COPY mid360_liinit.yaml /root/catkin_ws/src/LiDAR_IMU_Init/config/mid360.yaml
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
