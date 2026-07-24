#!/usr/bin/env python3
"""Bridge: replay an S20 ROS1 bag (LiDAR + IMU) into a real FAST-LIO2 ROS2
node, collect its /Odometry output, and write frame_pose.txt in the same
format the rest of the slamcloude pipeline already reads (see
worker/pipeline/s20.py read_frame_pose/write_frame_pose: "t x y z qx qy qz
qw" per line).

Standalone usage (no Celery/S3 -- for the definitive validation test before
wiring into the main pipeline):

    python3 fastlio_bridge.py --bag /data/all_....bag \
        --calibration /data/calibration.yaml \
        --out /data/frame_pose_fastlio.txt \
        --workdir /tmp/fastlio_run

Critical fix baked in here (see session notes): the S20's raw IMU
linear_acceleration is natively in **g**, not m/s^2 (confirmed by matching
this bridge's own raw-bag parse against the vendor's own captured
ImuData.txt byte-for-byte: mean norm ~1.0). ROS (REP-145) and FAST-LIO2 both
expect m/s^2, so every published accel sample is multiplied by G_MS2 below.
Feeding un-scaled g-unit values into FAST-LIO2's IESKF is the most likely
cause of the catastrophic (multi-km) trajectory divergence seen in this
session's first, now-abandoned FAST-LIO2 attempt.
"""
import argparse
import re
import shutil
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint
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

FASTLIO_SHARE_CONFIG = Path(
    "/opt/fastlio_ws/install/fast_lio/share/fast_lio/config/mid360.yaml"
)
TEMPLATE_PATH = Path("/srv/mid360_template.yaml")


def _header_end(raw: bytes) -> int:
    off = 12
    fid_len = struct.unpack_from("<I", raw, off)[0]
    return off + 4 + fid_len


def _parse_lidar_msg(raw: bytes):
    """Same layout as worker/pipeline/s20.py _parse_custom_msg_lidar, but
    returns the structured per-point array as-is (not flattened) so we can
    republish offset_time/reflectivity/tag/line unchanged."""
    off = _header_end(raw)
    off += 8  # on-device timebase -- unreliable, ignored (see s20.py docstring)
    off += 4  # point_num (redundant with array length below)
    off += 4  # lidar_id (1) + rsvd (3)
    arr_len = struct.unpack_from("<I", raw, off)[0]
    off += 4
    return np.frombuffer(raw, dtype=_CUSTOM_POINT_DTYPE, count=arr_len, offset=off)


def _parse_imu_msg(raw: bytes):
    """sensor_msgs/msg/Imu, ROS1 wire format: Header, orientation(4f64) +
    its covariance(9f64), angular_velocity(3f64) + its covariance(9f64),
    linear_acceleration(3f64) [+ its covariance -- unused here]."""
    off = _header_end(raw) + 32 + 72
    wx, wy, wz = struct.unpack_from("<ddd", raw, off)
    off2 = off + 24 + 72
    ax, ay, az = struct.unpack_from("<ddd", raw, off2)  # units: g (see module docstring)
    return (wx, wy, wz), (ax, ay, az)


def load_extrinsics(calibration_path: Path):
    """Minimal extractor for LIDAR_IMU_T / LIDAR_IMU_R / IMU_time_offset out
    of the S20's OpenCV-YAML calibration.yaml (full !!opencv-matrix tags
    aren't parseable by plain PyYAML, so this regexes the specific blocks we
    need)."""
    text = calibration_path.read_text(encoding="utf-8", errors="replace")

    m = re.search(r"LIDAR_IMU_T:\s*\[([^\]]+)\]", text)
    t_vals = [float(x) for x in m.group(1).replace("\n", " ").split(",")] if m else [-0.011, -0.02329, 0.04412]

    m = re.search(r"LIDAR_IMU_R:.*?data:\s*\[([^\]]+)\]", text, re.DOTALL)
    r_vals = [float(x) for x in m.group(1).replace("\n", " ").split(",")] if m else [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # FAST-LIO2's laserMapping.cpp applies this as, in imu_cbk:
    #   imu.stamp = imu.stamp_raw - time_offset_lidar_to_imu
    # i.e. it is subtracted from the IMU clock to align it to the LiDAR
    # clock. The S20's own calibration.yaml field is a *hardware-measured*
    # LiDAR-vs-IMU clock offset (found: -0.01s on this unit) but its sign
    # convention relative to FAST-LIO2's subtraction has NOT been verified
    # against a real run -- passed through as-is here (best first guess);
    # if a run's local roughness gets *worse* than the 0.0-offset baseline,
    # try negating this value before assuming the offset itself is wrong.
    m = re.search(r"IMU_time_offset:\s*(-?[\d.eE+-]+)", text)
    time_offset = float(m.group(1)) if m else 0.0

    return t_vals, r_vals, time_offset


def generate_config(calibration_path: Path):
    t_vals, r_vals, time_offset = load_extrinsics(calibration_path)
    text = TEMPLATE_PATH.read_text(encoding="utf-8")
    t_str = "[ " + ", ".join(f"{v:.6f}" for v in t_vals) + " ]"
    r_str = (
        "[ " + ", ".join(f"{v:.6f}" for v in r_vals[0:3]) + ",\n"
        "               " + ", ".join(f"{v:.6f}" for v in r_vals[3:6]) + ",\n"
        "               " + ", ".join(f"{v:.6f}" for v in r_vals[6:9]) + " ]"
    )
    text = re.sub(r"extrinsic_T:\s*\[[^\]]*\]", f"extrinsic_T: {t_str}", text)
    text = re.sub(r"extrinsic_R:\s*\[.*?\]\s*\n", f"extrinsic_R: {r_str}\n", text, flags=re.DOTALL)
    text = re.sub(
        r"time_offset_lidar_to_imu:\s*[\d.eE+-]+",
        f"time_offset_lidar_to_imu: {time_offset:.6f}",
        text,
    )
    FASTLIO_SHARE_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    FASTLIO_SHARE_CONFIG.write_text(text, encoding="utf-8")
    print(f"[fastlio_bridge] wrote config: {FASTLIO_SHARE_CONFIG}", flush=True)
    print(f"[fastlio_bridge]   extrinsic_T={t_vals} extrinsic_R={r_vals} time_offset_lidar_to_imu={time_offset}", flush=True)


class BridgeNode(Node):
    def __init__(self):
        super().__init__("fastlio_bridge")
        qos = QoSProfile(depth=200)
        self.lidar_pub = self.create_publisher(CustomMsg, "/livox/lidar", qos)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", qos)
        self.odom_sub = self.create_subscription(Odometry, "/Odometry", self._on_odom, qos)
        self.cloud_sub = self.create_subscription(PointCloud2, "/cloud_registered", self._on_cloud, qos)
        self.poses: list[tuple] = []
        self.cloud_chunks: list[np.ndarray] = []
        self.cloud_times: list[np.ndarray] = []
        self.n_cloud_points = 0
        self.last_odom_wall = time.time()
        self.last_odom_msg_t = None

    def _on_cloud(self, msg: PointCloud2):
        """/cloud_registered: current scan's points, ALREADY per-point
        undistorted (deskewed against intra-scan IMU motion) and transformed
        to world (camera_init) frame by FAST-LIO2 itself -- see
        laserMapping.cpp publish_frame_world()/RGBpointBodyToWorld(). Using
        this directly avoids our own coarse re-transform of raw body-frame
        points via linear interpolation between ~10Hz /Odometry poses, which
        cannot capture true intra-scan rotation during fast-turning segments.
        """
        field_map = {f.name: f for f in msg.fields}
        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            return
        n = msg.width * msg.height
        if n == 0:
            return
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        data = raw.reshape(n, msg.point_step)
        fx, fy, fz = field_map["x"], field_map["y"], field_map["z"]
        x = data[:, fx.offset:fx.offset + 4].copy().view("<f4").ravel()
        y = data[:, fy.offset:fy.offset + 4].copy().view("<f4").ravel()
        z = data[:, fz.offset:fz.offset + 4].copy().view("<f4").ravel()
        pts = np.column_stack([x, y, z]).astype(np.float64)
        self.cloud_chunks.append(pts)
        # georeference_from_slam needs a per-point gps_time to apply its
        # time-varying RTK drift correction -- FAST-LIO2 already deskews
        # every point in a scan to one reference time (lidar_end_time), so
        # tagging the whole chunk with the message's own header.stamp is
        # exact, not an approximation.
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.cloud_times.append(np.full(len(pts), t_msg, dtype=np.float64))
        self.n_cloud_points += len(pts)

    def _on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.last_odom_wall = time.time()
        self.last_odom_msg_t = t

    def publish_lidar(self, t_sec: float, pts: np.ndarray):
        msg = CustomMsg()
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        msg.header.stamp = RosTime(sec=sec, nanosec=nsec)
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
        self.lidar_pub.publish(msg)

    def publish_imu(self, t_sec: float, gyro, acc_g):
        msg = Imu()
        sec = int(t_sec)
        nsec = int(round((t_sec - sec) * 1e9))
        msg.header.stamp = RosTime(sec=sec, nanosec=nsec)
        msg.header.frame_id = "livox_frame"
        msg.orientation_covariance[0] = -1.0  # "no orientation estimate" (ROS convention)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        # *** the critical fix: raw device units are g, FAST-LIO2/ROS want m/s^2 ***
        msg.linear_acceleration.x = acc_g[0] * G_MS2
        msg.linear_acceleration.y = acc_g[1] * G_MS2
        msg.linear_acceleration.z = acc_g[2] * G_MS2
        self.imu_pub.publish(msg)


def replay_bag(node: BridgeNode, bag_path: Path, speed: float):
    events = []  # (t, kind, raw)
    with Reader(bag_path) as reader:
        lidar_conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        imu_conn = next((c for c in reader.connections if c.topic in _IMU_TOPICS), None)
        conns = [c for c in (lidar_conn, imu_conn) if c is not None]
        print(f"[fastlio_bridge] lidar_topic={lidar_conn.topic if lidar_conn else None} "
              f"imu_topic={imu_conn.topic if imu_conn else None}", flush=True)
        n_lidar = n_imu = 0
        t_prev = None
        t0_wall = time.time()
        for connection, msg_time_ns, rawdata in reader.messages(connections=conns):
            t = msg_time_ns / 1e9
            raw = bytes(rawdata)
            if t_prev is not None and speed > 0:
                dt = (t - t_prev) / speed
                if 0 < dt < 2.0:
                    time.sleep(dt)
            t_prev = t
            if connection.topic == lidar_conn.topic if lidar_conn else False:
                pts = _parse_lidar_msg(raw)
                node.publish_lidar(t, pts)
                n_lidar += 1
            elif imu_conn is not None and connection.topic == imu_conn.topic:
                gyro, acc_g = _parse_imu_msg(raw)
                node.publish_imu(t, gyro, acc_g)
                n_imu += 1
            rclpy.spin_once(node, timeout_sec=0)
            if (n_lidar + n_imu) % 500 == 0:
                print(f"[fastlio_bridge] replayed lidar={n_lidar} imu={n_imu} "
                      f"bag_t={t - (t_prev or t):.1f} wall_elapsed={time.time()-t0_wall:.1f}s", flush=True)
    print(f"[fastlio_bridge] replay done: lidar={n_lidar} imu={n_imu} msgs, "
          f"wall={time.time()-t0_wall:.1f}s", flush=True)


def wait_for_convergence(node: BridgeNode, grace_sec: float, max_wait_sec: float):
    deadline = time.time() + max_wait_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - node.last_odom_wall > grace_sec:
            print(f"[fastlio_bridge] no new /Odometry for {grace_sec}s -- "
                  f"assuming FAST-LIO2 has caught up", flush=True)
            return
    print("[fastlio_bridge] WARNING: hit max_wait_sec before convergence detected", flush=True)


def write_frame_pose(poses: list[tuple], out_path: Path):
    poses = sorted(poses, key=lambda p: p[0])
    with open(out_path, "w", encoding="utf-8") as f:
        for t, x, y, z, qx, qy, qz, qw in poses:
            f.write(f"{t:.9f} {x:.6f} {y:.6f} {z:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")
    print(f"[fastlio_bridge] wrote {len(poses)} poses to {out_path}", flush=True)


def write_cloud_laz(chunks: list, times: list, out_path: Path):
    if not chunks:
        print("[fastlio_bridge] WARNING: no /cloud_registered points accumulated -- check that scan_publish_en/dense_publish_en are true in mid360.yaml", flush=True)
        return
    import laspy
    pts = np.concatenate(chunks, axis=0)
    gps_time = np.concatenate(times, axis=0)
    print(f"[fastlio_bridge] writing {len(pts)} /cloud_registered points to {out_path}", flush=True)
    # point_format 6 has no gps_time field on its own -- format 7 (XYZI + RGB
    # + gps_time) matches what bag_lidar_to_laz/georeference_from_slam
    # already produce/expect elsewhere in the pipeline.
    hdr = laspy.LasHeader(point_format=7, version="1.4")
    hdr.offsets = np.array([pts[:, 0].min(), pts[:, 1].min(), pts[:, 2].min()])
    hdr.scales = np.array([0.001, 0.001, 0.001])
    with laspy.open(out_path, mode="w", header=hdr) as wrt:
        chunk = 500_000
        n = len(pts)
        for i in range(0, n, chunk):
            sl = slice(i, i + chunk)
            p = laspy.ScaleAwarePointRecord.zeros(len(pts[sl]), header=hdr)
            p.x = pts[sl, 0]
            p.y = pts[sl, 1]
            p.z = pts[sl, 2]
            p.gps_time = gps_time[sl]
            wrt.write_points(p)
    print(f"[fastlio_bridge] wrote cloud LAZ: {out_path}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--out-cloud", default=None, help="if set, write accumulated /cloud_registered points here (LAZ)")
    ap.add_argument("--speed", type=float, default=1.0, help="bag replay speed multiplier")
    ap.add_argument("--grace-sec", type=float, default=8.0)
    ap.add_argument("--max-wait-sec", type=float, default=120.0)
    args = ap.parse_args()

    generate_config(Path(args.calibration))

    print("[fastlio_bridge] launching fast_lio mapping.launch.py ...", flush=True)
    proc = subprocess.Popen(
        ["ros2", "launch", "fast_lio", "mapping.launch.py", "rviz:=false"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
    )

    def _drain_output():
        for line in proc.stdout:
            print(f"[fast_lio] {line.rstrip()}", flush=True)

    import threading
    t = threading.Thread(target=_drain_output, daemon=True)
    t.start()

    print("[fastlio_bridge] waiting 8s for fast_lio node to come up ...", flush=True)
    time.sleep(8.0)

    rclpy.init()
    node = BridgeNode()
    try:
        replay_bag(node, Path(args.bag), speed=args.speed)
        wait_for_convergence(node, grace_sec=args.grace_sec, max_wait_sec=args.max_wait_sec)
    finally:
        write_frame_pose(node.poses, Path(args.out))
        print(f"[fastlio_bridge] accumulated {node.n_cloud_points} /cloud_registered points "
              f"in {len(node.cloud_chunks)} scans", flush=True)
        if args.out_cloud:
            write_cloud_laz(node.cloud_chunks, node.cloud_times, Path(args.out_cloud))
        node.destroy_node()
        rclpy.shutdown()
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()

    if not node.poses:
        print("[fastlio_bridge] ERROR: zero /Odometry poses received", flush=True)
        sys.exit(1)
    print("[fastlio_bridge] done.", flush=True)


if __name__ == "__main__":
    main()