#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

roscore &
ROSCORE_PID=$!
sleep 3

roslaunch lidar_imu_init livox_mid360.launch rviz:=false &
LIINIT_PID=$!
sleep 5

python3 /root/catkin_ws/liinit_bridge.py "$@"
BRIDGE_EXIT=$?

RESULT_FILE="/root/catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt"
if [ -f "$RESULT_FILE" ]; then
    echo "=== Initialization_result.txt ==="
    cat "$RESULT_FILE"
    cp "$RESULT_FILE" /data/liinit_result.txt 2>/dev/null || true
else
    echo "=== NO RESULT FILE PRODUCED at $RESULT_FILE ==="
fi

kill $LIINIT_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
exit $BRIDGE_EXIT
