#!/bin/bash
# Offline mode: the patched voxelslam node reads the bag directly (no external
# publisher / Python bridge). Usage:
#   docker run --rm -v <dir>:/data slamcloude-worker-voxelslam \
#       --bag /data/scan.bag [--save-dir /data/voxelslam_out] [--bagname s20]
set -e
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

BAG=""
SAVE_DIR="/data/voxelslam_out"
BAGNAME="s20"
GBA_WAIT_SEC=3600
while [ $# -gt 0 ]; do
  case "$1" in
    --bag) BAG="$2"; shift 2;;
    --save-dir) SAVE_DIR="$2"; shift 2;;
    --bagname) BAGNAME="$2"; shift 2;;
    --gba-wait-sec) GBA_WAIT_SEC="$2"; shift 2;;
    *) echo "unknown arg $1"; shift;;
  esac
done
if [ -z "$BAG" ]; then echo "ERROR: --bag is required"; exit 2; fi

roscore &
ROSCORE_PID=$!
sleep 3

# The node reads General/bagname (and save_path) from the config, and writes
# its PCDs to save_path/bagname -- refusing to run if that folder already has
# data. The config hardcodes one bagname, so patch it to this run's --bagname
# before launch, letting different scans coexist under save_path.
CFG=/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/config/mid360.yaml
sed -i "s#^  bagname:.*#  bagname: \"$BAGNAME\"#" "$CFG"

mkdir -p "$SAVE_DIR"
rm -rf "$SAVE_DIR/$BAGNAME"

# The patched node reads /bag_path (global param) in main(); set it before the
# node launches.
rosparam set /bag_path "$BAG"
sleep 1

roslaunch voxel_slam vxlm_mid360.launch rviz:=false > /tmp/voxelslam_node.log 2>&1 &
NODE_PID=$!

STATE_FILE="$SAVE_DIR/$BAGNAME/alidarState.txt"
echo "[entrypoint] node launched, reading $BAG offline; waiting for global BA -> $STATE_FILE"
deadline=$(( $(date +%s) + GBA_WAIT_SEC ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  if [ -f "$STATE_FILE" ]; then
    s1=$(stat -c %s "$STATE_FILE" 2>/dev/null || echo 0)
    sleep 8
    s2=$(stat -c %s "$STATE_FILE" 2>/dev/null || echo 0)
    if [ "$s1" = "$s2" ] && [ "$s1" != "0" ]; then
      echo "[entrypoint] alidarState.txt stable ($s2 bytes) -- global BA done"
      break
    fi
  fi
  sleep 5
done

echo "=== voxelslam node log (tail) ==="
tail -n 30 /tmp/voxelslam_node.log 2>/dev/null || true

kill $NODE_PID 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true
echo "[entrypoint] done"
