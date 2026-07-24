# hku-mars/Voxel-SLAM -- the SAME LiDAR-inertial SLAM core the SHARE
# PointClouds Studio vendor engine (share_slam2_offline.exe) is built on:
# reverse-engineering that binary earlier this session found the C++
# namespace `voxelslam::VOXEL_SLAM`, a `BTC`/`BTCOctoTree` loop-closure
# descriptor, GTSAM `GPSFactor2`/`BetweenFactor<Pose3>`, and hierarchical
# global bundle adjustment (`HBA_add_edge`/`multi_margi`/`topDownProcess`) --
# every one of which maps 1:1 onto this open-source repo's files
# (voxelslam.cpp, BTC.cpp, preintegration.hpp, loop_refine.hpp).
#
# Unlike FAST-LIO2 (worker-fastlio), which is odometry ONLY, Voxel-SLAM is
# the full stack: odometry + local bundle-adjustment + LiDAR loop closure +
# global bundle adjustment -- i.e. the modules that let the vendor reach
# 0.42m where our FAST-LIO2 front-end plateaus at ~4m. ROS1-only (catkin,
# livox_ros_driver v1), so it's its own image, same as worker-liinit.
FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libpcl-dev libeigen3-dev libboost-all-dev libtbb-dev \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-tf \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# GTSAM 4.0.3 exactly (Voxel-SLAM's README-pinned version). Build with the
# system Eigen (GTSAM's bundled Eigen otherwise clashes with PCL/ROS Eigen at
# link time) and without the march=native tuning (portability across the
# build host vs the run host).
RUN git clone --depth 1 --branch 4.0.3 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && mkdir /tmp/gtsam/build && cd /tmp/gtsam/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig \
    && rm -rf /tmp/gtsam

# numpy/rosbags for the offline bridge; open3d(0.17 = last py3.8 build)/laspy/
# lazrs to assemble the saved PCDs into a frame_pose + world-frame LAZ;
# celery[redis]/boto3 to run as a pipeline worker (celery_worker.py).
# Noetic's stock pip is too old to resolve open3d's manylinux2014 wheel, so
# upgrade pip first.
# No lazrs (its newer builds pull puccinialin, a rust installer needing
# py3.9+): the assembler writes uncompressed .las, which laspy reads back by
# magic bytes regardless of extension.
# --ignore-installed PyYAML: Noetic ships PyYAML 5.3.1 via distutils, which
# pip can't cleanly uninstall when a dependency wants a newer one.
RUN python3 -m pip install --no-cache-dir --upgrade "pip<25" setuptools wheel \
    && python3 -m pip install --no-cache-dir --ignore-installed PyYAML \
    "numpy<1.25" rosbags "open3d==0.17.0" laspy "celery[redis]" boto3

WORKDIR /root/catkin_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone --depth 1 https://github.com/hku-mars/Voxel-SLAM.git \
    # VoxelSLAMPointCloud2 is an RViz *display plugin* (needs the rviz dev
    # package to configure) used only for on-screen visualization -- this
    # image is headless, and the core `voxel_slam` node doesn't depend on it,
    # so drop it to avoid pulling in rviz just to build a plugin we never load.
    && rm -rf Voxel-SLAM/VoxelSLAMPointCloud2

# Patch Voxel-SLAM to read the bag directly inside the node (offline,
# synchronous) + apply the S20 IMU g->m/s^2 fix. See offline_bag.patch.py.
COPY offline_bag.patch.py /tmp/offline_bag.patch.py
RUN python3 /tmp/offline_bag.patch.py

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j\"$(nproc)\""

COPY voxelslam_bridge.py /root/catkin_ws/voxelslam_bridge.py
COPY celery_worker.py /root/catkin_ws/celery_worker.py
COPY mid360_s20.yaml /root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/config/mid360.yaml
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Default: standalone offline run (docker run ... --bag ...). The docker-compose
# `worker-voxelslam` service overrides this to run the Celery worker instead:
#   command: bash -lc "source /opt/ros/noetic/setup.bash &&
#                      source /root/catkin_ws/devel/setup.bash &&
#                      celery -A celery_worker.app worker -Q voxelslam --loglevel=info"
ENTRYPOINT ["/entrypoint.sh"]
