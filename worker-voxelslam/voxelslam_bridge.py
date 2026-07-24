#!/usr/bin/env python3
"""Bridge: replay an S20 ROS1 bag (LiDAR + IMU) into hku-mars/Voxel-SLAM --
the same full LiDAR-inertial SLAM core (odometry + local BA + BTC loop
closure + hierarchical global BA) the SHARE vendor engine is built on.

Message parsing/unit conversion is identical to worker-fastlio and
worker-liinit's bridges (raw accel is in g -> multiply by G_MS2; the LiDAR
message's embedded `timebase` is unreliable so the bag envelope time is
used), and the wall-clock pacing is the self-correcting absolute-schedule
version proven in worker-liinit (a fixed per-message sleep accumulates lag).

After the bag finishes replaying, this sets the ROS param `finish=true`
(exactly what Voxel-SLAM's README says to do via `rosparam set finish true`)
to trigger the global bundle-adjustment pass, then waits for the saved
`.pcd` map to appear under save_path before exiting.

Usage:
    python3 voxelslam_bridge.py --bag /data/scan.bag \
        --save-dir /data/voxelslam_out --bagname s20 [--speed 1.0]
"""
import argparse
import struct
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
    off += 4  # point_num
    off += 4  # lidar_id + rsvd
    arr_len = struct.unpack_from("<I", raw, off)[0]
    off += 4
    return np.frombuffer(raw, dtype=_CUSTOM_POINT_DTYPE, count=arr_len, offset=off)


def _parse_imu_msg(raw: bytes):
    off = _header_end(raw) + 32 + 72
    wx, wy, wz = struct.unpack_from("<ddd", raw, off)
    off2 = off + 24 + 72
    ax, ay, az = struct.unpack_from("<ddd", raw, off2)  # units: g
    return (wx, wy, wz), (ax, ay, az)


# Voxel-SLAM's ekf_imu.hpp aborts (exit 0, "LiDAR time regress") if a scan's
# end time (begin + last point's offset_time) overruns the next scan's begin
# by >10ms. Our 10Hz bag has enough header-interval jitter that raw
# offset_time (up to ~0.1s) trips this. Rather than disable intra-scan deskew
# entirely (point_notime=1, coarser), linearly SCALE each scan's offset_time
# so its span never exceeds this cap -- points keep their true relative
# ordering and near-true relative timing (the scan is just slightly
# time-compressed), preserving most of the deskew benefit.
# Measured scan intervals on this bag: mean 100ms, but min 77.5ms (jitter).
# Regress trips when offset_span > interval - 10ms, so for the 77.5ms worst
# case the span must stay below ~67ms; 60ms leaves margin while still keeping
# ~60% of the intra-scan deskew (vs point_notime=1 which keeps none).
_OFFSET_CAP_NS = 60_000_000  # 0.060s, safely below the 77.5ms min interval


def publish_lidar(pub, t_sec: float, pts: np.ndarray):
    msg = CustomMsg()
    sec = int(t_sec)
    nsec = int(round((t_sec - sec) * 1e9))
    msg.header.stamp = rospy.Time(sec, nsec)
    msg.header.frame_id = "livox_frame"
    msg.timebase = sec * 1_000_000_000 + nsec
    msg.point_num = len(pts)
    msg.lidar_id = 0
    offs = pts["offset_time"].astype(np.float64)
    max_off = offs.max() if len(offs) else 0.0
    if max_off > _OFFSET_CAP_NS:
        offs = offs * (_OFFSET_CAP_NS / max_off)
    offs = offs.astype(np.uint32)
    msg.points = [
        CustomPoint(
            offset_time=int(offs[i]),
            x=float(pt["x"]), y=float(pt["y"]), z=float(pt["z"]),
            reflectivity=int(pt["reflectivity"]), tag=int(pt["tag"]), line=int(pt["line"]),
        )
        for i, pt in enumerate(pts)
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
    with Reader(bag_path) as reader:
        lidar_conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        imu_conn = next((c for c in reader.connections if c.topic in _IMU_TOPICS), None)
        conns = [c for c in (lidar_conn, imu_conn) if c is not None]
        print(f"[voxelslam_bridge] lidar_topic={lidar_conn.topic if lidar_conn else None} "
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
                print(f"[voxelslam_bridge] replayed lidar={n_lidar} imu={n_imu} "
                      f"wall_elapsed={time.time()-t0_wall:.1f}s max_lag={max_lag_s:.2f}s", flush=True)
    print(f"[voxelslam_bridge] replay done: lidar={n_lidar} imu={n_imu} msgs, "
          f"wall={time.time()-t0_wall:.1f}s, max_lag={max_lag_s:.2f}s", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--save-dir", required=True,
                     help="directory Voxel-SLAM's save_path points at (watched for the output .pcd)")
    ap.add_argument("--bagname", default="s20")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--lidar-stride", type=int, default=1)
    ap.add_argument("--gba-wait-sec", type=float, default=1800.0,
                     help="max time to wait for the global-BA pass + saved map after setting finish=true")
    args = ap.parse_args()

    rospy.init_node("voxelslam_bridge", anonymous=True, disable_signals=True)
    lidar_pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=200)
    imu_pub = rospy.Publisher("/livox/imu", Imu, queue_size=200)
    time.sleep(1.0)

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    pcds_before = {p.name for p in save_dir.glob("*.pcd")}

    replay_bag(lidar_pub, imu_pub, Path(args.bag), speed=args.speed, lidar_stride=args.lidar_stride)

    # Give the odometry/local-BA a moment to drain its queue, then trigger the
    # global bundle-adjustment pass (README: `rosparam set finish true`).
    print("[voxelslam_bridge] draining, then setting finish=true to start global BA ...", flush=True)
    time.sleep(15.0)
    rospy.set_param("finish", True)

    print(f"[voxelslam_bridge] waiting up to {args.gba_wait_sec}s for a new .pcd in {save_dir} ...", flush=True)
    deadline = time.time() + args.gba_wait_sec
    last_report = 0.0
    while time.time() < deadline:
        pcds_now = {p.name for p in save_dir.glob("*.pcd")}
        new = pcds_now - pcds_before
        if new:
            # Wait for the file to stop growing before declaring done.
            newest = max((save_dir / n for n in new), key=lambda p: p.stat().st_mtime)
            s1 = newest.stat().st_size
            time.sleep(5.0)
            if newest.stat().st_size == s1 and s1 > 0:
                print(f"[voxelslam_bridge] saved map: {newest} ({s1} bytes)", flush=True)
                break
        if time.time() - last_report > 30:
            print(f"[voxelslam_bridge] still waiting for global BA / map save "
                  f"({int(deadline - time.time())}s left) ...", flush=True)
            last_report = time.time()
        time.sleep(3.0)
    else:
        print("[voxelslam_bridge] WARNING: no saved .pcd appeared within the wait window.", flush=True)

    print("[voxelslam_bridge] done.", flush=True)


if __name__ == "__main__":
    main()
