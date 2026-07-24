#!/usr/bin/env python3
"""Bridge: replay an S20 ROS1 bag (LiDAR + IMU) into hku-mars/LiDAR_IMU_Init
("LI-Init"), which independently computes the LiDAR-IMU extrinsic transform
and time offset from the data itself -- used to cross-check the values
worker-fastlio currently takes at face value from the S20's calibration.yaml
(see gtsam_georeference.py and mid360_template.yaml for what was found/fixed
there: IMU_time_offset was previously unapplied at all).

Same parsing/unit-conversion logic as worker-fastlio/fastlio_bridge.py
(reused verbatim: raw accel is in g, not m/s^2; the LiDAR message's own
embedded `timebase` field is unreliable and the bag's envelope timestamp is
used instead) -- only the ROS1/livox_ros_driver-v1 (not ROS2/v2) message
publishing differs, since LI-Init is ROS1-only.

Usage:
    python3 liinit_bridge.py --bag /data/scan.bag [--speed 1.0]

LI-Init itself (roslaunch'd by entrypoint.sh before this script runs) needs
real excitation (translation AND rotation in multiple axes) to converge --
a short/nearly-static recording will simply never produce a result file.
"""
import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from livox_ros_driver.msg import CustomMsg, CustomPoint
from rosbags.rosbag1 import Reader

G_MS2 = 9.80665

_LIDAR_TOPICS = ["/livox/lidar", "/livox/lidar_node", "/points", "/livox/points"]
_IMU_TOPICS = ["/livox/imu", "/imu/data", "/imu_raw", "/imu", "/rtk_agent/imu"]

_CUSTOM_POINT_DTYPE = np.dtype([
    ("offset_time", "<u4"),
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("reflectivity", "u1"),
    ("tag", "u1"),
    ("line", "u1"),
])


def _header_end(raw: bytes) -> int:
    off = 12
    fid_len = struct.unpack_from("<I", raw, off)[0]
    return off + 4 + fid_len


def _parse_lidar_msg(raw: bytes):
    off = _header_end(raw)
    off += 8  # on-device timebase -- unreliable, ignored (see fastlio_bridge.py)
    off += 4  # point_num (redundant with array length below)
    off += 4  # lidar_id (1) + rsvd (3)
    arr_len = struct.unpack_from("<I", raw, off)[0]
    off += 4
    return np.frombuffer(raw, dtype=_CUSTOM_POINT_DTYPE, count=arr_len, offset=off)


def _parse_imu_msg(raw: bytes):
    off = _header_end(raw) + 32 + 72
    wx, wy, wz = struct.unpack_from("<ddd", raw, off)
    off2 = off + 24 + 72
    ax, ay, az = struct.unpack_from("<ddd", raw, off2)  # units: g
    return (wx, wy, wz), (ax, ay, az)


def publish_lidar(pub, t_sec: float, pts: np.ndarray):
    msg = CustomMsg()
    sec = int(t_sec)
    nsec = int(round((t_sec - sec) * 1e9))
    msg.header.stamp = rospy.Time(sec, nsec)
    msg.header.frame_id = "livox_frame"
    msg.timebase = sec * 1_000_000_000 + nsec
    msg.point_num = len(pts)
    msg.lidar_id = 0
    msg.points = [
        CustomPoint(
            offset_time=int(pt["offset_time"]),
            x=float(pt["x"]), y=float(pt["y"]), z=float(pt["z"]),
            reflectivity=int(pt["reflectivity"]), tag=int(pt["tag"]), line=int(pt["line"]),
        )
        for pt in pts
    ]
    pub.publish(msg)


def publish_imu(pub, t_sec: float, gyro, acc_g):
    msg = Imu()
    sec = int(t_sec)
    nsec = int(round((t_sec - sec) * 1e9))
    msg.header.stamp = rospy.Time(sec, nsec)
    msg.header.frame_id = "livox_frame"
    msg.orientation_covariance = [0.0] * 9
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
    msg.linear_acceleration.x = acc_g[0] * G_MS2
    msg.linear_acceleration.y = acc_g[1] * G_MS2
    msg.linear_acceleration.z = acc_g[2] * G_MS2
    pub.publish(msg)


def replay_bag(lidar_pub, imu_pub, bag_path: Path, speed: float, lidar_stride: int):
    """Publish at a wall-clock pace matching the bag's own relative timing
    (scaled by `speed`), computed against an absolute schedule (t0_wall +
    (bag_t - t0_bag)/speed) rather than sleeping a fixed per-message delta --
    the latter compounds any per-message processing overhead into
    accumulating lag (measured: a 328s bag took 582s wall at speed=2.0, i.e.
    slower than even real-time, because CustomPoint construction + rospy
    serialization cost more than the intended per-message sleep). LI-Init's
    own time-offset estimate came out wildly wrong (72.9s) on that first
    run, most likely *because* of this drift -- this fixes the pacing itself
    and downsamples LiDAR points (lidar_stride) to cut the dominant
    per-message cost, rather than trying to guess how much drift the
    algorithm can tolerate.
    """
    with Reader(bag_path) as reader:
        lidar_conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        imu_conn = next((c for c in reader.connections if c.topic in _IMU_TOPICS), None)
        conns = [c for c in (lidar_conn, imu_conn) if c is not None]
        print(f"[liinit_bridge] lidar_topic={lidar_conn.topic if lidar_conn else None} "
              f"imu_topic={imu_conn.topic if imu_conn else None} lidar_stride={lidar_stride}", flush=True)
        n_lidar = n_imu = 0
        t0_bag = None
        t0_wall = time.time()
        max_lag_s = 0.0
        for connection, msg_time_ns, rawdata in reader.messages(connections=conns):
            t = msg_time_ns / 1e9
            raw = bytes(rawdata)
            if t0_bag is None:
                t0_bag = t
            if speed > 0:
                target_wall = t0_wall + (t - t0_bag) / speed
                now = time.time()
                lag = now - target_wall
                if lag > max_lag_s:
                    max_lag_s = lag
                if target_wall > now:
                    time.sleep(target_wall - now)
            if lidar_conn is not None and connection.topic == lidar_conn.topic:
                pts = _parse_lidar_msg(raw)
                if lidar_stride > 1:
                    pts = pts[::lidar_stride]
                publish_lidar(lidar_pub, t, pts)
                n_lidar += 1
            elif imu_conn is not None and connection.topic == imu_conn.topic:
                gyro, acc_g = _parse_imu_msg(raw)
                publish_imu(imu_pub, t, gyro, acc_g)
                n_imu += 1
            if (n_lidar + n_imu) % 500 == 0:
                print(f"[liinit_bridge] replayed lidar={n_lidar} imu={n_imu} "
                      f"wall_elapsed={time.time()-t0_wall:.1f}s max_lag={max_lag_s:.2f}s", flush=True)
    print(f"[liinit_bridge] replay done: lidar={n_lidar} imu={n_imu} msgs, "
          f"wall={time.time()-t0_wall:.1f}s, max_lag={max_lag_s:.2f}s", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--result-wait-sec", type=float, default=60.0,
                     help="how long to keep waiting for Initialization_result.txt after replay finishes")
    ap.add_argument("--lidar-stride", type=int, default=1,
                     help="publish every Nth LiDAR point (reduces per-message rospy overhead)")
    args = ap.parse_args()

    rospy.init_node("liinit_bridge", anonymous=True, disable_signals=True)
    lidar_pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=200)
    imu_pub = rospy.Publisher("/livox/imu", Imu, queue_size=200)
    time.sleep(1.0)  # let publishers register with roscore before the first message

    replay_bag(lidar_pub, imu_pub, Path(args.bag), speed=args.speed, lidar_stride=args.lidar_stride)

    result_path = Path("/root/catkin_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt")
    print(f"[liinit_bridge] waiting up to {args.result_wait_sec}s for {result_path} ...", flush=True)
    deadline = time.time() + args.result_wait_sec
    while time.time() < deadline:
        if result_path.exists() and result_path.stat().st_size > 0:
            print("[liinit_bridge] result file appeared.", flush=True)
            break
        time.sleep(2.0)
    else:
        print("[liinit_bridge] WARNING: no result file after wait -- "
              "recording may lack sufficient excitation (needs real translation "
              "AND rotation in multiple axes).", flush=True)

    print("[liinit_bridge] done.", flush=True)


if __name__ == "__main__":
    main()
