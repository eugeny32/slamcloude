#!/usr/bin/env python3
"""Celery worker exposing the Voxel-SLAM kernel as a pipeline step.

Listens on the "voxelslam" queue for `voxelslam.process`, which the main
worker's COMPUTE_SLAM step dispatches cross-queue (send_task(..., queue=
"voxelslam")). It downloads a scan's bag + calibration from S3/MinIO, runs the
offline Voxel-SLAM node (odometry -> local BA -> loop closure -> global BA),
assembles the per-scan PCDs + post-BA trajectory into a frame_pose.txt and a
world-frame cloud LAZ (with intensity + gps_time), and uploads both back to S3
for the downstream COLORIZE / GEOREFERENCE steps.

Lives in the ROS1 (Noetic, py3.8) worker-voxelslam image, isolated from the
main worker for the same reason worker-fastlio is: it needs a full ROS stack.
"""
import glob
import os
import subprocess
import time
from pathlib import Path

import boto3
import numpy as np
from celery import Celery

REDIS_URL = os.environ["REDIS_URL"]
S3_ENDPOINT = os.environ.get("S3_ENDPOINT_URL", "http://minio:9000")
S3_KEY = os.environ.get("S3_ACCESS_KEY", "minioadmin")
S3_SECRET = os.environ.get("S3_SECRET_KEY", "minioadmin")
RAW_BUCKET = os.environ.get("S3_RAW_BUCKET", "raw")

app = Celery("voxelslam", broker=REDIS_URL, backend=REDIS_URL)
app.conf.update(
    task_serializer="json", result_serializer="json", accept_content=["json"],
    task_acks_late=True, worker_prefetch_multiplier=1,
)


def _s3():
    return boto3.client(
        "s3", endpoint_url=S3_ENDPOINT,
        aws_access_key_id=S3_KEY, aws_secret_access_key=S3_SECRET,
    )


def _quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _run_node(bag_path: Path, save_dir: Path, bagname: str, timeout_s: float = 5400) -> Path:
    """Run the offline Voxel-SLAM node until its global-BA trajectory
    (alidarState.txt) is written; returns the per-scan-PCD output dir."""
    out_dir = save_dir / bagname
    state = out_dir / "alidarState.txt"
    save_dir.mkdir(parents=True, exist_ok=True)
    if out_dir.exists():
        subprocess.run(["rm", "-rf", str(out_dir)], check=False)

    cfg = "/root/catkin_ws/src/Voxel-SLAM/VoxelSLAM/config/mid360.yaml"
    subprocess.run(["sed", "-i", f's#^  bagname:.*#  bagname: "{bagname}"#', cfg], check=True)
    subprocess.run(["sed", "-i", f's#^  save_path:.*#  save_path: "{save_dir}/"#', cfg], check=True)

    env = os.environ.copy()
    roscore = subprocess.Popen(["roscore"], env=env)
    time.sleep(4)
    subprocess.run(["rosparam", "set", "/bag_path", str(bag_path)], env=env, check=True)
    node = subprocess.Popen(
        ["roslaunch", "voxel_slam", "vxlm_mid360.launch", "rviz:=false"], env=env,
    )
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline:
            if state.exists() and state.stat().st_size > 0:
                s1 = state.stat().st_size
                time.sleep(8)
                if state.stat().st_size == s1:
                    break
            time.sleep(5)
    finally:
        node.terminate()
        roscore.terminate()
        time.sleep(2)
    if not state.exists():
        raise RuntimeError("Voxel-SLAM produced no alidarState.txt (global BA did not finish)")
    return out_dir


def _assemble(out_dir: Path, frame_pose_out: Path, cloud_out: Path):
    """Per-scan PCDs (local frame) + post-BA poses -> frame_pose.txt +
    world-frame LAZ (XYZ + intensity + gps_time)."""
    import open3d as o3d
    import laspy

    poses, times = [], []
    with open(out_dir / "alidarState.txt") as f, open(frame_pose_out, "w") as fp:
        for line in f:
            p = line.split()
            if len(p) < 8:
                continue
            t = float(p[0]); px, py, pz = float(p[1]), float(p[2]), float(p[3])
            qx, qy, qz, qw = float(p[4]), float(p[5]), float(p[6]), float(p[7])
            poses.append((_quat_to_R(qx, qy, qz, qw), np.array([px, py, pz])))
            times.append(t)
            fp.write(f"{t:.9f} {px:.6f} {py:.6f} {pz:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n")

    pcds = sorted(glob.glob(str(out_dir / "*.pcd")), key=lambda s: int(os.path.basename(s)[:-4]))
    xs, ys, zs, ts, iis = [], [], [], [], []
    for pf in pcds:
        idx = int(os.path.basename(pf)[:-4])
        if idx >= len(poses):
            continue
        R, t = poses[idx]
        pc = o3d.t.io.read_point_cloud(pf)
        pts = pc.point["positions"].numpy().astype(np.float64)
        if len(pts) == 0:
            continue
        w = pts @ R.T + t
        xs.append(w[:, 0]); ys.append(w[:, 1]); zs.append(w[:, 2])
        ts.append(np.full(len(w), times[idx]))
        iis.append(pc.point["intensity"].numpy().ravel().astype(np.float64)
                   if "intensity" in pc.point else np.zeros(len(w)))

    x = np.concatenate(xs); y = np.concatenate(ys); z = np.concatenate(zs)
    gt = np.concatenate(ts); iv = np.concatenate(iis)
    hdr = laspy.LasHeader(point_format=6, version="1.4")
    hdr.offsets = np.array([x.min(), y.min(), z.min()])
    hdr.scales = np.array([0.001, 0.001, 0.001])
    las = laspy.LasData(hdr)
    las.x, las.y, las.z = x, y, z
    las.gps_time = gt
    las.intensity = (np.clip(iv, 0, 255).astype(np.uint16) * 257)
    las.write(cloud_out)
    return len(x)


@app.task(name="voxelslam.process")
def process(bag_key: str, calibration_key: str,
            frame_pose_out_key: str, cloud_out_key: str, bagname: str = "scan") -> dict:
    s3 = _s3()
    work = Path("/tmp/vxlm_job")
    work.mkdir(parents=True, exist_ok=True)
    bag = work / "scan.bag"
    cal = work / "calibration.yaml"
    s3.download_file(RAW_BUCKET, bag_key, str(bag))
    if calibration_key:
        try:
            s3.download_file(RAW_BUCKET, calibration_key, str(cal))
        except Exception:
            pass

    out_dir = _run_node(bag, work / "out", bagname)
    fp_path = work / "frame_pose.txt"
    # Uncompressed .las (no lazrs backend in this image); laspy reads it back
    # by magic bytes even though the S3 key ends .laz.
    cloud_path = work / "cloud.las"
    n = _assemble(out_dir, fp_path, cloud_path)

    s3.upload_file(str(fp_path), RAW_BUCKET, frame_pose_out_key)
    s3.upload_file(str(cloud_path), RAW_BUCKET, cloud_out_key)
    return {"points": n, "frame_pose_key": frame_pose_out_key, "cloud_key": cloud_out_key}
