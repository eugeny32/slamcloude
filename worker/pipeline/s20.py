"""SHARE S20 scanner-specific processing: bag extraction, camera colorisation.

The S20 backpack is a Livox MID-360 LiDAR + 3 cameras:
  /usb_cam/image_raw/compressed   -- nav camera, 20 Hz, 640x480 pinhole
  /camera_agent/img_left/compressed  -- left fisheye, 0.5 Hz, 3504x4672
  /camera_agent/img_right/compressed -- right fisheye, 0.5 Hz, 3504x4672

Data can be uploaded as:
  - A ZIP of PCD folders (default, bag_lidar_enabled=False)
  - A ZIP containing a ROS1 bag file (bag_lidar_enabled=True)

Bag format: classic ROS1 "#ROSBAG V2.0" container (NOT ROS2/sqlite3). Message
wire format is tightly packed with no alignment padding, so raw `struct`
unpacking with a "<" prefix matches the layout exactly. LiDAR points arrive as
livox_ros_driver2/msg/CustomMsg (not sensor_msgs/PointCloud2). RTK position is
best read from the vendor-custom rtk_agent/msg/PVTSLNMsg (real 3D fix with a
solution-quality flag); sensor_msgs/msg/NavSatFix is a fallback but always
reports altitude=0 on this device (no NTRIP-derived height).
"""

import io
import struct
import zipfile
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import laspy
import numpy as np
from pyproj import CRS, Transformer
from scipy.optimize import least_squares

from pipeline.processing import ProcessingError

_CAM_TOPICS: dict[str, str] = {
    "/usb_cam/image_raw/compressed": "nav",
    "/camera_agent/img_left/compressed": "left",
    "/camera_agent/img_right/compressed": "right",
}

_LIDAR_TOPICS = ["/livox/lidar", "/livox/lidar_node", "/points", "/livox/points"]

_PVT_TOPICS = ["/rtk_agent/pvtsln", "/rtk_agent/pvtsln_sync"]

_FIX_TOPICS = [
    "/rtk_agent/navsatfix",
    "/rtk_agent/navsatfix_sync",
    "/navsatfix",
    "/fix",
    "/gnss/fix",
    "/ublox/fix",
]

_IMU_TOPICS = ["/livox/imu", "/imu/data", "/imu_raw", "/imu", "/rtk_agent/imu"]

# rtk_agent PosType codes accepted for georeferencing: RTK-FIXED ONLY --
# NARROW_INT (50) and INS_RTKFIXED (56). Float/wide-lane/single solutions
# (NARROW_FLOAT=34, WIDE_INT=49, INS_RTKFLOAT=55, L1_FLOAT=32, SINGLE=16, ...)
# are deliberately rejected: their cm->dm position error would be baked into
# the trajectory alignment. This matches the vendor engine's own high-accuracy
# gate (its rtk_quality.txt: "pos_type=NARROW_INT(50)/INS_RTKFIXED(56),
# HDOP<0.9, hgtstd<0.1m"). Where no fixed epoch is available the trajectory is
# left to the LiDAR-inertial SLAM (Voxel-SLAM / FAST-LIO) rather than pulled
# toward an unreliable float fix.
_FIXED_POS_TYPES = (50, 56)
_RTK_MAX_HDOP = 0.9      # matches vendor high-accuracy gate
_RTK_MAX_HGTSTD = 0.1    # metres, matches vendor high-accuracy gate


@dataclass
class FramePose:
    """Interpolatable body-frame trajectory from frame_pose.txt."""
    times: np.ndarray
    tx: np.ndarray
    ty: np.ndarray
    tz: np.ndarray
    qx: np.ndarray
    qy: np.ndarray
    qz: np.ndarray
    qw: np.ndarray


def read_frame_pose(path: str | Path) -> FramePose:
    """Parse frame_pose.txt: t x y z qx qy qz qw per line."""
    cols: list[list[float]] = [[] for _ in range(8)]
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            for i in range(8):
                cols[i].append(float(parts[i]))
    if not cols[0]:
        raise ProcessingError(f"frame_pose file is empty or unparseable: {path}")
    return FramePose(
        times=np.array(cols[0], dtype=np.float64),
        tx=np.array(cols[1], dtype=np.float64),
        ty=np.array(cols[2], dtype=np.float64),
        tz=np.array(cols[3], dtype=np.float64),
        qx=np.array(cols[4], dtype=np.float64),
        qy=np.array(cols[5], dtype=np.float64),
        qz=np.array(cols[6], dtype=np.float64),
        qw=np.array(cols[7], dtype=np.float64),
    )


def write_frame_pose(fp: FramePose, path: str | Path) -> None:
    """Write frame_pose.txt: t x y z qx qy qz qw per line (matches read_frame_pose)."""
    with open(path, "w", encoding="utf-8") as f:
        for i in range(len(fp.times)):
            f.write(
                f"{fp.times[i]:.9f} {fp.tx[i]:.6f} {fp.ty[i]:.6f} {fp.tz[i]:.6f} "
                f"{fp.qx[i]:.9f} {fp.qy[i]:.9f} {fp.qz[i]:.9f} {fp.qw[i]:.9f}\n"
            )

def _interp_frame_pose(fp: FramePose, times: np.ndarray):
    """Vectorised interpolation returning (tx,ty,tz,qx,qy,qz,qw) arrays."""
    q = np.column_stack([fp.qx, fp.qy, fp.qz, fp.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    return (
        np.interp(times, fp.times, fp.tx),
        np.interp(times, fp.times, fp.ty),
        np.interp(times, fp.times, fp.tz),
        np.interp(times, fp.times, q[:, 0]),
        np.interp(times, fp.times, q[:, 1]),
        np.interp(times, fp.times, q[:, 2]),
        np.interp(times, fp.times, q[:, 3]),
    )


def _apply_frame_pose_to_points(x, y, z, times, fp: FramePose):
    """Transform points from body frame to world_slam frame using frame_pose."""
    itx, ity, itz, iqx, iqy, iqz, iqw = _interp_frame_pose(fp, times)
    norms = np.sqrt(iqx**2 + iqy**2 + iqz**2 + iqw**2)
    norms = np.where(norms < 1e-9, 1.0, norms)
    iqx /= norms; iqy /= norms; iqz /= norms; iqw /= norms

    x2, y2, z2 = iqx**2, iqy**2, iqz**2
    wx, wy, wz = iqw * iqx, iqw * iqy, iqw * iqz
    xy, xz, yz = iqx * iqy, iqx * iqz, iqy * iqz

    wx_body = x * (1 - 2*(y2 + z2)) + y * 2*(xy - wz) + z * 2*(xz + wy)
    wy_body = x * 2*(xy + wz) + y * (1 - 2*(x2 + z2)) + z * 2*(yz - wx)
    wz_body = x * 2*(xz - wy) + y * 2*(yz + wx) + z * (1 - 2*(x2 + y2))

    return wx_body + itx, wy_body + ity, wz_body + itz


def _rotvec_from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> axis-angle rotation vector (Rodrigues)."""
    cos_ang = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = float(np.arccos(cos_ang))
    if ang < 1e-8:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-12:
        return np.zeros(3)
    return axis / axis_norm * ang


def _matrix_from_rotvec(v: np.ndarray) -> np.ndarray:
    """Axis-angle rotation vector -> rotation matrix (Rodrigues)."""
    ang = float(np.linalg.norm(v))
    if ang < 1e-8:
        return np.eye(3)
    axis = v / ang
    K = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ])
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)


def _visual_relative_rotation(img_i, img_j, K: np.ndarray, orb, bf, min_inliers: int):
    """ORB + essential-matrix relative rotation between two grayscale images.

    Returns (R_vis, n_inliers, inlier_ratio) or None if there aren't enough
    confident feature matches to trust an estimate (textureless scene, no
    overlap, degenerate two-view geometry).
    """
    import cv2

    kp1, des1 = orb.detectAndCompute(img_i, None) if img_i is not None else (None, None)
    kp2, des2 = orb.detectAndCompute(img_j, None) if img_j is not None else (None, None)
    if des1 is None or des2 is None or len(kp1) < 15 or len(kp2) < 15:
        return None
    matches = bf.knnMatch(des1, des2, k=2)
    good = [m for m, n in matches if n.distance > 0 and m.distance < 0.75 * n.distance]
    if len(good) < min_inliers:
        return None
    pts1 = np.array([kp1[m.queryIdx].pt for m in good])
    pts2 = np.array([kp2[m.trainIdx].pt for m in good])
    E, mask = cv2.findEssentialMat(pts1, pts2, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        return None
    n_inliers, R_vis, _t_vis, _mask_pose = cv2.recoverPose(E, pts1, pts2, K, mask=mask)
    if n_inliers < min_inliers:
        return None
    return R_vis, int(n_inliers), n_inliers / len(good)


def _collect_visual_observations(
    nav_dir: Path,
    nav_names: list[str],
    nav_ts: np.ndarray,
    frame_pose: FramePose,
    R_cam_in_body: np.ndarray,
    K: np.ndarray,
    step_s: float = 3.0,
    max_frames: int = 400,
    min_gap_s: float = 20.0,
    max_pos_dist_m: float = 4.0,
    min_inliers: int = 20,
) -> list[tuple[float, float, np.ndarray, float]]:
    """Collect visual relative-rotation observations against SLAM's own claim,
    both from consecutive frame pairs (dense, full time coverage, each
    individually noisy) and from genuine revisits (loop closures: sparse but
    a direct check against long-accumulated drift). Both matter: dense-only
    integration is a random walk (noise alone produces apparent "drift" when
    summed, even with zero true bias) and loop-closure-only leaves whatever
    time range never revisits itself with no correction at all. Combined and
    fed into a single robust joint optimization (_fit_pose_graph_correction),
    the smoothness prior plus Huber loss let the many weak dense constraints
    average out their own noise while the loop closures anchor the long-range
    absolute drift -- neither signal alone was reliable (see git history /
    session notes: pure loop-closure and pure dense-integration attempts each
    failed for different, complementary reasons).

    Returns (t_i, t_j, rotvec_world, weight) tuples; rotvec_world is already
    rotated into the shared world_slam frame (see the derivation in the
    inline comment below), not camera_j's own frame.
    """
    import cv2

    order = np.argsort(nav_ts)
    names = [nav_names[i] for i in order]
    ts = nav_ts[order]
    t0, t1 = float(ts[0]), float(ts[-1])

    grid = np.arange(t0, t1, step_s)
    sub_idx = sorted({int(np.argmin(np.abs(ts - g))) for g in grid})
    if len(sub_idx) > max_frames:
        sub_idx = sub_idx[:: max(1, len(sub_idx) // max_frames)]

    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    def R_body_world(t: float) -> np.ndarray:
        qx = np.interp(t, frame_pose.times, q[:, 0])
        qy = np.interp(t, frame_pose.times, q[:, 1])
        qz = np.interp(t, frame_pose.times, q[:, 2])
        qw = np.interp(t, frame_pose.times, q[:, 3])
        n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-9:
            return np.eye(3)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        x2, y2, z2 = qx * qx, qy * qy, qz * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        return np.array([
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ])

    def sensor_pos(t: float) -> np.ndarray:
        return np.array([
            np.interp(t, frame_pose.times, frame_pose.tx),
            np.interp(t, frame_pose.times, frame_pose.ty),
            np.interp(t, frame_pose.times, frame_pose.tz),
        ])

    orb = cv2.ORB_create(nfeatures=1200)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    img_cache: dict[int, np.ndarray] = {}

    def get_img(idx: int):
        if idx not in img_cache:
            img_cache[idx] = cv2.imread(str(nav_dir / names[idx]), cv2.IMREAD_GRAYSCALE)
        return img_cache[idx]

    def make_observation(i: int, j: int):
        res = _visual_relative_rotation(get_img(i), get_img(j), K, orb, bf, min_inliers)
        if res is None:
            return None
        R_vis, n_inliers, inlier_ratio = res
        Rwi = R_body_world(ts[i]) @ R_cam_in_body
        Rwj = R_body_world(ts[j]) @ R_cam_in_body
        R_slam_rel = Rwj.T @ Rwi
        R_err = R_vis @ R_slam_rel.T
        # R_err's rotation vector lives in camera_j's own (rotated) frame, not
        # a frame shared/comparable across different observation pairs (each
        # pair has a different Rwj) -- rotate it into the common world_slam
        # frame. Derivation: R_vis ~= Rcb^T Rwj_body^T Exp(w_i-w_j) Rwi_body
        # Rcb = Exp(Rwj^T (w_i-w_j)) @ R_slam_rel for small-signal w, so
        # rotvec(R_err) ~= Rwj^T (w_i-w_j), i.e. (w_j-w_i) ~= -Rwj @ rotvec(R_err).
        v_world = -(Rwj @ _rotvec_from_matrix(R_err))
        weight = (min(n_inliers, 80) / 80.0) * inlier_ratio
        return float(ts[i]), float(ts[j]), v_world, weight

    observations: list[tuple[float, float, np.ndarray, float]] = []

    # Dense: consecutive subsampled frames, full time coverage.
    for a in range(len(sub_idx) - 1):
        i, j = sub_idx[a], sub_idx[a + 1]
        if not (0 < ts[j] - ts[i] <= 3 * step_s):
            continue
        obs = make_observation(i, j)
        if obs is not None:
            observations.append(obs)

    # Loop closures: genuine revisits, direct long-range drift evidence.
    positions = {i: sensor_pos(ts[i]) for i in sub_idx}
    for a in range(len(sub_idx)):
        i = sub_idx[a]
        for b in range(a + 1, len(sub_idx)):
            j = sub_idx[b]
            if ts[j] - ts[i] < min_gap_s:
                continue
            if np.linalg.norm(positions[j] - positions[i]) > max_pos_dist_m:
                continue
            obs = make_observation(i, j)
            if obs is not None:
                observations.append(obs)

    return observations


def _fit_pose_graph_correction(
    observations: list[tuple[float, float, np.ndarray, float]],
    t_start: float,
    t_end: float,
    ctrl_step_s: float = 15.0,
    smooth_weight: float = 0.5,
    anchor_weight: float = 1.0,
    huber_scale_deg: float = 5.0,
):
    """Jointly fit a smoothed, piecewise-linear cumulative attitude-drift
    correction delta(t) (world_slam-frame axis-angle rotation vector) from
    the combined dense + loop-closure visual observations, via robust
    nonlinear least squares (Huber loss).

    This replaces an earlier plain (non-robust) linear least-squares version:
    with the noise characteristics of monocular ORB/essential-matrix
    estimates on this camera, a handful of individually bad or moderately-off
    observations dominated a plain-lstsq fit or a naive cumulative sum (both
    tried and rejected -- see visually_correct_frame_pose's docstring
    history). Huber loss down-weights outlier residuals automatically
    (iteratively reweighted least squares) instead of requiring a hand-tuned
    hard threshold, and jointly optimizing all observations plus a
    smoothness prior (discourage large jumps between adjacent control
    points) and a start-anchor (delta(t_start) ~= 0) in one solve lets dense
    and loop-closure evidence reinforce each other rather than being treated
    as two disconnected signals.
    """
    ctrl_t = np.arange(t_start, t_end + ctrl_step_s, ctrl_step_s)
    n_ctrl = len(ctrl_t)
    if n_ctrl < 2 or not observations:
        return np.array([t_start, t_end]), np.zeros((2, 3))

    def bracket(t: float):
        idx = int(np.searchsorted(ctrl_t, t)) - 1
        idx = int(np.clip(idx, 0, n_ctrl - 2))
        t0_, t1_ = ctrl_t[idx], ctrl_t[idx + 1]
        w = 0.0 if t1_ == t0_ else (t - t0_) / (t1_ - t0_)
        return idx, w

    obs_brackets = [(bracket(ti), bracket(tj), v, w) for ti, tj, v, w in observations]

    def residuals(x: np.ndarray) -> np.ndarray:
        delta = x.reshape(n_ctrl, 3)
        rows = []
        for (ai, wi), (aj, wj), v, w in obs_brackets:
            d_i = (1 - wi) * delta[ai] + wi * delta[ai + 1]
            d_j = (1 - wj) * delta[aj] + wj * delta[aj + 1]
            rows.append(w * ((d_j - d_i) - v))
        for k in range(n_ctrl - 1):
            rows.append(smooth_weight * (delta[k + 1] - delta[k]))
        rows.append(anchor_weight * delta[0])
        return np.concatenate(rows)

    x0 = np.zeros(n_ctrl * 3)
    result = least_squares(
        residuals, x0, loss="huber", f_scale=np.radians(huber_scale_deg), max_nfev=20000
    )
    return ctrl_t, result.x.reshape(n_ctrl, 3)


def visually_correct_frame_pose(
    frame_pose: FramePose,
    cam_zip_path: str | Path | None,
    calibration_path: str | Path | None,
    workdir: str | Path,
    bag_path: str | Path | None = None,
    min_observations: int = 10,
    step_s: float = 3.0,
    max_frames: int = 400,
    min_gap_s: float = 20.0,
    max_pos_dist_m: float = 4.0,
) -> FramePose:
    """Correct frame_pose's orientation using nav-camera evidence (vision-only
    in production; bag_path can optionally add LiDAR-ICP or raw-gyro evidence,
    see below -- both are currently disabled by the caller, tasks.py).

    RTK position alone cannot observe attitude, so a heading-only or
    translation-only correction (see georeference_from_slam) cannot fix
    on-device SLAM roll/pitch/yaw drift -- and that drift is exactly what
    range-dependent geometry warping downstream traces back to. Vision
    (_collect_visual_observations) feeds _fit_pose_graph_correction: dense
    frame-to-frame pairs plus sparse loop-closure revisits. Loop closures are
    the only way to directly check *absolute* drift accumulated over minutes
    (nothing else in this pipeline can), but they're rare (few genuine
    revisits) and individually noisy (monocular ORB/essential-matrix
    estimates on a narrow-FOV camera over low-texture scenes).

    Two additional sensor sources were tried via the optional bag_path
    argument and both REVERTED after real-data testing (kept in this module,
    not called from tasks.py, so they don't run in production):

      - LiDAR ICP loop closure (_collect_lidar_loop_closure_observations):
        direct scan-to-scan geometric registration at genuine revisits. This
        is the technique established open-source LIO systems (FAST-LIO2,
        LIO-SAM, FAST-LIO-SAM, FAST-LIVO2) actually use for this correction,
        and its formula was validated on a synthetic known-ground-truth
        scene+drift (<0.05 deg recovery error) -- but on this device's real
        data it produced wildly wrong per-observation corrections (median 17
        deg, up to 172 deg -- point-to-point ICP converging to a wrong local
        minimum on sparse/possibly-repetitive geometry within the short
        ~0.5s scan windows, not caught by the inlier-ratio/RMS gate) and
        measurably made the result worse (scan e6b4bbe7 median cell Z-range
        roughness: 13.3m vision-only vs 27.2m with LiDAR ICP added -- see
        session notes). Needs covariance/degeneracy-aware correspondence
        rejection (not just inlier-ratio+RMS) before it can be trusted.
      - Raw IMU gyro (_collect_imu_observations): raw /livox/imu gyro
        integration showed an unexplained per-window discrepancy against
        SLAM's own rotation on real data that survived unit/sign/extrinsic/
        quaternion-convention checks.

    Best-effort throughout: any failure (missing calibration/frames/opencv/
    too few reliable observations) returns frame_pose unchanged rather than
    raising, since this correction is a refinement on top of an
    already-functional pipeline, not a hard requirement.
    """
    workdir = Path(workdir)
    observations: list[tuple[float, float, np.ndarray, float]] = []

    if cam_zip_path is not None and calibration_path is not None:
        try:
            import cv2  # noqa: F401  (imported for the module-level name used below)

            cal = _load_calibration_camera(calibration_path)
            if cal is not None:
                fx, fy, cx, cy, R_cam_in_body, _t_cam_in_body = cal
                K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

                nav_dir = workdir / "_visual_drift_nav_frames"
                nav_dir.mkdir(parents=True, exist_ok=True)
                with zipfile.ZipFile(cam_zip_path) as zf:
                    nav_names = sorted(
                        n for n in zf.namelist() if n.startswith("nav_") and n.endswith(".jpg")
                    )
                    for n in nav_names:
                        target = nav_dir / Path(n).name
                        if not target.exists():
                            target.write_bytes(zf.read(n))
                if len(nav_names) >= 10:
                    nav_names_flat = [Path(n).name for n in nav_names]
                    nav_ts = np.array(
                        [int(Path(n).stem.split("_", 1)[1]) / 1e9 for n in nav_names_flat]
                    )
                    observations += _collect_visual_observations(
                        nav_dir, nav_names_flat, nav_ts, frame_pose, R_cam_in_body, K,
                        step_s=step_s, max_frames=max_frames,
                        min_gap_s=min_gap_s, max_pos_dist_m=max_pos_dist_m,
                    )
        except ImportError:
            pass
        except Exception:
            pass

    if bag_path is not None:
        try:
            observations += _collect_lidar_loop_closure_observations(bag_path, frame_pose)
        except Exception:
            pass

    if len(observations) < min_observations:
        return frame_pose

    try:
        ctrl_t, ctrl_rotvec = _fit_pose_graph_correction(
            observations, float(frame_pose.times[0]), float(frame_pose.times[-1])
        )

        q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
        for i in range(1, len(q)):
            if np.dot(q[i], q[i - 1]) < 0:
                q[i] = -q[i]

        t_clip = np.clip(frame_pose.times, ctrl_t[0], ctrl_t[-1])
        delta = np.column_stack([
            np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 0]),
            np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 1]),
            np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 2]),
        ])

        new_q = np.zeros_like(q)
        for k in range(len(frame_pose.times)):
            qx, qy, qz, qw = q[k]
            n_ = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if n_ < 1e-9:
                new_q[k] = q[k]
                continue
            qx, qy, qz, qw = qx / n_, qy / n_, qz / n_, qw / n_
            x2, y2, z2 = qx * qx, qy * qy, qz * qz
            wx, wy, wz = qw * qx, qw * qy, qw * qz
            xy, xz, yz = qx * qy, qx * qz, qy * qz
            R_orig = np.array([
                [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
            ])
            # delta(t) is fitted (_collect_visual_observations /
            # _fit_pose_graph_correction) to satisfy R_true(t) = Exp(delta(t))
            # @ R_slam(t) -- i.e. it's LEFT-multiplied, unnegated, onto the
            # original SLAM rotation. Verified against a synthetic
            # known-ground-truth trajectory (see session notes): applying
            # Exp(-delta) here (an earlier version of this line) silently
            # applied every correction backwards, which is why three
            # different observation/fitting strategies all independently
            # measured zero net improvement despite individually looking
            # like they'd fitted a sane, physically-plausible curve.
            R_delta = _matrix_from_rotvec(delta[k])
            R_corr = R_delta @ R_orig
            tr = np.trace(R_corr)
            if tr > 0:
                s = np.sqrt(tr + 1.0) * 2
                qw2 = 0.25 * s
                qx2 = (R_corr[2, 1] - R_corr[1, 2]) / s
                qy2 = (R_corr[0, 2] - R_corr[2, 0]) / s
                qz2 = (R_corr[1, 0] - R_corr[0, 1]) / s
            elif R_corr[0, 0] > R_corr[1, 1] and R_corr[0, 0] > R_corr[2, 2]:
                s = np.sqrt(1.0 + R_corr[0, 0] - R_corr[1, 1] - R_corr[2, 2]) * 2
                qw2 = (R_corr[2, 1] - R_corr[1, 2]) / s
                qx2 = 0.25 * s
                qy2 = (R_corr[0, 1] + R_corr[1, 0]) / s
                qz2 = (R_corr[0, 2] + R_corr[2, 0]) / s
            elif R_corr[1, 1] > R_corr[2, 2]:
                s = np.sqrt(1.0 + R_corr[1, 1] - R_corr[0, 0] - R_corr[2, 2]) * 2
                qw2 = (R_corr[0, 2] - R_corr[2, 0]) / s
                qx2 = (R_corr[0, 1] + R_corr[1, 0]) / s
                qy2 = 0.25 * s
                qz2 = (R_corr[1, 2] + R_corr[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R_corr[2, 2] - R_corr[0, 0] - R_corr[1, 1]) * 2
                qw2 = (R_corr[1, 0] - R_corr[0, 1]) / s
                qx2 = (R_corr[0, 2] + R_corr[2, 0]) / s
                qy2 = (R_corr[1, 2] + R_corr[2, 1]) / s
                qz2 = 0.25 * s
            new_q[k] = [qx2, qy2, qz2, qw2]

        return FramePose(
            times=frame_pose.times,
            tx=frame_pose.tx, ty=frame_pose.ty, tz=frame_pose.tz,
            qx=new_q[:, 0], qy=new_q[:, 1], qz=new_q[:, 2], qw=new_q[:, 3],
        )
    except Exception:
        return frame_pose


def _extract_bag_from_zip(zip_path: Path, out_dir: Path):
    """Extract the main recording .bag from a ZIP.

    A ZIP may contain more than one .bag (e.g. the main recording plus a
    small info/*.bag metadata sidecar) -- the real data bag is always the
    largest one, so pick by size rather than by first/last match.
    """
    with zipfile.ZipFile(zip_path) as zf:
        candidates = [
            n for n in zf.namelist()
            if n.endswith(".bag") and not n.startswith("__MACOSX")
        ]
        name = max(candidates, key=lambda n: zf.getinfo(n).file_size, default=None)
        if name is None:
            return None
        target = out_dir / Path(name).name
        with zf.open(name) as src, open(target, "wb") as dst:
            while chunk := src.read(8 * 1024 * 1024):
                dst.write(chunk)
        return target


def _resolve_bag_path(bag_zip_or_bag_path: str | Path, tmp_dir: Path) -> Path | None:
    bag_zip_or_bag_path = Path(bag_zip_or_bag_path)
    if bag_zip_or_bag_path.suffix.lower() == ".zip":
        tmp_dir.mkdir(parents=True, exist_ok=True)
        return _extract_bag_from_zip(bag_zip_or_bag_path, tmp_dir)
    return bag_zip_or_bag_path


# ---------------------------------------------------------------------------
# ROS1 raw wire-format parsers.
#
# ROS1 message serialization is tightly packed (no alignment padding), so
# struct.unpack_from with a "<" prefix reproduces the layout exactly -- unlike
# ROS2/CDR, which requires manual alignment bookkeeping. Header is always
# uint32 seq + uint32 stamp.sec + uint32 stamp.nsec + (uint32 len + bytes) frame_id.
# ---------------------------------------------------------------------------

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


def _parse_custom_msg_lidar(raw: bytes, msg_time_ns: int):
    """Parse livox_ros_driver2/msg/CustomMsg (ROS1 wire format).

    msg_time_ns is the bag's own envelope timestamp for this message (from
    rosbags' Reader.messages()), used as the point-time base instead of the
    message's embedded `timebase` field. On real S20 recordings `timebase`
    runs on the Livox unit's own free-running/unsynced internal clock and can
    be off from the true recording time by a large, effectively constant
    offset (observed: ~1.5 years) -- while every other topic (RTK, camera,
    and the on-device SLAM's own frame_pose.txt) is timestamped against the
    bag's envelope clock. Trusting `timebase` here silently clips every
    point's interpolation time to a single frame_pose sample downstream
    (since it falls entirely outside frame_pose's real time range), collapsing
    the whole recording onto one fixed pose.
    """
    try:
        off = _header_end(raw)
        off += 8  # timebase -- unreliable, ignored (see docstring)
        off += 4  # point_num (redundant with array length below)
        off += 4  # lidar_id (1) + rsvd (3)
        arr_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        pts = np.frombuffer(raw, dtype=_CUSTOM_POINT_DTYPE, count=arr_len, offset=off)
        t = msg_time_ns / 1e9 + pts["offset_time"].astype(np.float64) / 1e9
        x = pts["x"].astype(np.float64)
        y = pts["y"].astype(np.float64)
        z = pts["z"].astype(np.float64)
        intensity = pts["reflectivity"].astype(np.float64)
        # Livox emits a body-frame-origin sentinel point for every "no return"
        # sample within a scan -- on real data these are the vast majority of
        # points in some batches and, left in, create a massive coincident
        # cluster that makes the downstream outlier-removal k-NN query
        # pathologically slow (near O(n^2) for a k-d tree with heavy ties).
        range_sq = x * x + y * y + z * z
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (range_sq > 1e-4)
        return x[valid], y[valid], z[valid], t[valid], intensity[valid]
    except Exception:
        return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)


def _parse_compressed_image_ros1(raw: bytes):
    """Parse sensor_msgs/msg/CompressedImage (ROS1 wire format)."""
    try:
        off = _header_end(raw)
        fmt_len = struct.unpack_from("<I", raw, off)[0]
        off += 4 + fmt_len
        data_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        return raw[off: off + data_len]
    except Exception:
        return None


def _parse_navsatfix_ros1(raw: bytes):
    """Parse sensor_msgs/msg/NavSatFix (ROS1 wire format).

    Returns (status, lat, lon, alt) or None. Altitude is always 0.0 on this
    device (RTK agent does not compute an NTRIP-derived height here).
    """
    try:
        off = _header_end(raw)
        status = struct.unpack_from("<b", raw, off)[0]
        off += 3  # int8 status + uint16 service, no padding
        lat, lon, alt = struct.unpack_from("<ddd", raw, off)
        if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
            return status, lat, lon, alt
    except Exception:
        pass
    return None


def _open_ros1_reader(bag_path: Path):
    from rosbags.rosbag1 import Reader as Ros1Reader
    return Ros1Reader(bag_path)


def extract_camera_frames_from_bag(bag_zip_or_bag_path: str | Path, out_dir: str | Path) -> int:
    """Extract compressed camera frames from a ROS1 bag (ZIP or direct .bag)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tmp_dir = out_dir / "_bag_extract_cam"
    bag_path = _resolve_bag_path(bag_zip_or_bag_path, tmp_dir)
    if bag_path is None or not bag_path.exists():
        return 0

    n_written = 0
    with _open_ros1_reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic in _CAM_TOPICS]
        if not conns:
            return 0
        for connection, timestamp, rawdata in reader.messages(connections=conns):
            alias = _CAM_TOPICS[connection.topic]
            jpeg = _parse_compressed_image_ros1(bytes(rawdata))
            if jpeg and len(jpeg) > 100:
                fname = out_dir / f"{alias}_{int(timestamp):019d}.jpg"
                fname.write_bytes(jpeg)
                n_written += 1
    return n_written


def bag_lidar_to_laz(
    bag_zip_or_bag_path: str | Path,
    out_laz: str | Path,
    frame_pose: FramePose | None = None,
) -> int:
    """Extract LiDAR from a ROS1 bag and write LAZ.

    If frame_pose is provided, transforms each point from body frame to
    world_slam frame using per-message interpolated pose.
    Returns number of points written.
    """
    out_laz = Path(out_laz)
    tmp_dir = out_laz.parent / "_bag_extract_lidar"
    bag_path = _resolve_bag_path(bag_zip_or_bag_path, tmp_dir)
    if bag_path is None or not bag_path.exists():
        raise ProcessingError(f"No .bag file found in {bag_zip_or_bag_path}")

    xs, ys, zs, ts_list, intensities = [], [], [], [], []

    with _open_ros1_reader(bag_path) as reader:
        conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        if conn is None:
            available = sorted({c.topic for c in reader.connections})
            raise ProcessingError(f"No LiDAR topic found. Available: {available}")

        for _connection, msg_time_ns, rawdata in reader.messages(connections=[conn]):
            x, y, z, t, intensity = _parse_custom_msg_lidar(bytes(rawdata), msg_time_ns)
            if len(x):
                xs.append(x); ys.append(y); zs.append(z)
                ts_list.append(t); intensities.append(intensity)

    if not xs:
        raise ProcessingError("No LiDAR points extracted from bag")

    all_x = np.concatenate(xs)
    all_y = np.concatenate(ys)
    all_z = np.concatenate(zs)
    all_t = np.concatenate(ts_list)
    all_i = np.concatenate(intensities)

    if frame_pose is not None:
        t_clip = np.clip(all_t, frame_pose.times[0], frame_pose.times[-1])
        all_x, all_y, all_z = _apply_frame_pose_to_points(
            all_x, all_y, all_z, t_clip, frame_pose
        )

    hdr = laspy.LasHeader(point_format=6, version="1.4")
    hdr.offsets = np.array([all_x.min(), all_y.min(), all_z.min()])
    hdr.scales = np.array([0.001, 0.001, 0.001])
    with laspy.open(out_laz, mode="w", header=hdr) as wrt:
        chunk = 500_000
        n = len(all_x)
        for i in range(0, n, chunk):
            sl = slice(i, i + chunk)
            p = laspy.ScaleAwarePointRecord.zeros(len(all_x[sl]), header=hdr)
            p.x = all_x[sl]
            p.y = all_y[sl]
            p.z = all_z[sl]
            p.gps_time = all_t[sl]
            p.intensity = np.clip(all_i[sl] * 655, 0, 65535).astype(np.uint16)
            wrt.write_points(p)

    return int(len(all_x))


def _read_pvtsln_fixes(reader, conn) -> list[tuple[float, float, float, float, int, float, float]]:
    """Read (ts, lat, lon, alt, pos_type, hdop, hgtstd) from rtk_agent/msg/PVTSLNMsg.

    Uses rosbags' dynamic typestore registration from the bag's own embedded
    message definition. Returns [] if registration/parsing fails (custom
    vendor type; caller should fall back to NavSatFix in that case). hdop /
    hgtstd are surfaced so the caller can apply the vendor's RTK-quality gate;
    missing fields default to +inf so they fail the gate rather than pass it.
    """
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    typestore = get_typestore(Stores.ROS1_NOETIC)
    add_types: dict = {}
    add_types.update(get_types_from_msg(conn.msgdef.data, conn.msgtype))
    typestore.register(add_types)

    fixes = []
    for _connection, timestamp, rawdata in reader.messages(connections=[conn]):
        msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
        pos_type = int(msg.bestpos_type.type)
        lat = float(msg.bestpos_lat)
        lon = float(msg.bestpos_lon)
        alt = float(msg.bestpos_hgt)
        hdop = float(getattr(msg, "hdop", float("inf")))
        hgtstd = float(getattr(msg, "bestpos_hgtstd", float("inf")))
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            continue
        fixes.append((timestamp / 1e9, lat, lon, alt, pos_type, hdop, hgtstd))
    return fixes


def bag_to_rtk_pos(bag_path: str | Path, out_pos: str | Path) -> int:
    """Read RTK positions from a ROS1 bag and write an RTKLIB-compatible .pos file.

    Prefers rtk_agent/msg/PVTSLNMsg and keeps ONLY RTK-fixed epochs passing the
    vendor's quality gate (pos_type in {NARROW_INT, INS_RTKFIXED}, HDOP < 0.9,
    hgtstd < 0.1m -- see _FIXED_POS_TYPES/_RTK_MAX_*). Float/single/wide-lane
    solutions are dropped rather than used as a fallback, so downstream
    georeferencing only aligns to trustworthy fixes and leaves everything else
    to the LiDAR-inertial SLAM. Falls back to sensor_msgs/msg/NavSatFix only if
    the PVTSLN topic is absent entirely (horizontal only -- this device always
    reports altitude=0 on that topic). Returns number of epochs written, or 0
    if no usable GNSS topic (or no fixed epoch) is found.
    """
    bag_path = Path(bag_path)
    out_pos = Path(out_pos)
    if not bag_path.exists():
        return 0

    epochs: list[tuple[float, float, float, float]] = []

    with _open_ros1_reader(bag_path) as reader:
        by_topic: dict[str, object] = {}
        for c in reader.connections:
            by_topic.setdefault(c.topic, c)

        pvt_conn = next((by_topic[t] for t in _PVT_TOPICS if t in by_topic), None)
        if pvt_conn is not None:
            try:
                raw_fixes = _read_pvtsln_fixes(reader, pvt_conn)
            except Exception:
                raw_fixes = []
            if raw_fixes:
                # RTK-FIXED only, with the vendor's HDOP/hgtstd quality gate.
                # No float fallback: an epoch that isn't a trustworthy fix is
                # dropped, and the trajectory keeps its LiDAR-inertial estimate
                # there instead of being aligned to an unreliable position.
                chosen = [
                    f for f in raw_fixes
                    if f[4] in _FIXED_POS_TYPES
                    and f[5] < _RTK_MAX_HDOP
                    and f[6] < _RTK_MAX_HGTSTD
                ]
                epochs = [(ts, lat, lon, alt) for ts, lat, lon, alt, _pt, _h, _s in chosen]

        if not epochs:
            fix_conn = next((by_topic[t] for t in _FIX_TOPICS if t in by_topic), None)
            if fix_conn is not None:
                for _connection, timestamp, rawdata in reader.messages(connections=[fix_conn]):
                    parsed = _parse_navsatfix_ros1(bytes(rawdata))
                    if parsed is None:
                        continue
                    status, lat, lon, alt = parsed
                    if status < 0:
                        continue
                    epochs.append((timestamp / 1e9, lat, lon, alt))

    if not epochs:
        return 0

    epochs.sort(key=lambda r: r[0])
    with open(out_pos, "w", encoding="utf-8") as f:
        f.write("% GNSS trajectory extracted from S20 ROS1 bag (RTK)\n")
        f.write("%  GPST                  latitude(deg) longitude(deg)  height(m)   Q  ns\n")
        for ts_unix, lat, lon, alt in epochs:
            dt = datetime.fromtimestamp(ts_unix, tz=UTC)
            f.write(
                f"{dt.strftime('%Y/%m/%d %H:%M:%S.%f')[:-3]}"
                f"  {lat:14.9f}  {lon:14.9f}  {alt:10.4f}   1  1\n"
            )

    return len(epochs)

def bag_to_imu_gyro(bag_path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Read raw gyroscope (angular_velocity) samples from /livox/imu.

    sensor_msgs/msg/Imu, ROS1 tightly-packed wire format: Header, then
    orientation (4x float64) + orientation_covariance (9x float64) before
    angular_velocity (3x float64, rad/s, body frame) -- offset is fixed
    regardless of header content since only frame_id has variable length,
    and _header_end already accounts for that.

    Returns (times [N] unix seconds, angular_velocity [N,3] rad/s), sorted
    by time. Empty arrays if the topic isn't present.
    """
    bag_path = Path(bag_path)
    if not bag_path.exists():
        return np.zeros(0), np.zeros((0, 3))

    times: list[float] = []
    wxyz: list[tuple[float, float, float]] = []

    with _open_ros1_reader(bag_path) as reader:
        conn = next((c for c in reader.connections if c.topic in _IMU_TOPICS), None)
        if conn is None:
            return np.zeros(0), np.zeros((0, 3))
        for _connection, msg_time_ns, rawdata in reader.messages(connections=[conn]):
            raw = bytes(rawdata)
            try:
                off = _header_end(raw) + 32 + 72  # skip orientation + its covariance
                wx, wy, wz = struct.unpack_from("<ddd", raw, off)
            except Exception:
                continue
            times.append(msg_time_ns / 1e9)
            wxyz.append((wx, wy, wz))

    if not times:
        return np.zeros(0), np.zeros((0, 3))
    order = np.argsort(times)
    return np.array(times)[order], np.array(wxyz)[order]


def _integrate_gyro(
    gyro_times: np.ndarray, gyro_wxyz: np.ndarray, t_a: float, t_b: float
) -> np.ndarray | None:
    """Integrate raw gyro samples in [t_a, t_b] into a relative rotation
    matrix via sequential on-manifold composition: R(t+dt) = R(t) @
    Exp(omega(t)*dt) (standard body-frame angular-velocity integration).

    Returns None if there isn't enough gyro coverage in the interval to
    trust the result (need reasonably dense samples spanning close to the
    full [t_a, t_b] range, not just a sparse handful).
    """
    idx = np.searchsorted(gyro_times, [t_a, t_b])
    lo, hi = int(idx[0]), int(idx[1])
    if hi - lo < 5:
        return None
    seg_t = gyro_times[lo:hi]
    seg_w = gyro_wxyz[lo:hi]
    if seg_t[0] - t_a > 0.5 or t_b - seg_t[-1] > 0.5:
        return None  # coverage gap at either end -- don't trust a partial integration

    R_rel = np.eye(3)
    for k in range(len(seg_t) - 1):
        dt = seg_t[k + 1] - seg_t[k]
        if dt <= 0 or dt > 0.5:
            continue
        R_rel = R_rel @ _matrix_from_rotvec(seg_w[k] * dt)
    return R_rel


def _collect_imu_observations(
    gyro_times: np.ndarray,
    gyro_wxyz: np.ndarray,
    frame_pose: FramePose,
    step_s: float = 5.0,
) -> list[tuple[float, float, np.ndarray, float]]:
    """Dense gyro-vs-SLAM relative-rotation observations across the whole
    recording, independent of camera frame availability or genuine revisits.

    Raw gyro integration over a short (~5s) interval is far less noisy than
    monocular ORB/essential-matrix estimates (see _collect_visual_observations'
    docstring for why vision alone wasn't reliable enough): it doesn't depend
    on scene texture, motion blur, or feature matching at all. Comparing it
    to SLAM's own reported relative rotation over the same short interval
    isolates exactly the discrepancy the on-device LIO's fusion introduces
    (drift/bias) that a raw, unfused gyro integration wouldn't have -- this
    gives dense, low-noise coverage for the pose-graph's inter-control-point
    propagation, complementing (not replacing) the sparse but long-range
    visual loop closures that anchor absolute drift at genuine revisits
    (gyro integration alone would itself drift too much to trust over the
    multi-minute gaps loop closures span).

    Verified against a synthetic known-ground-truth trajectory before this
    was wired in (see session notes): v_world = +R_slam(t_a) @
    rotvec(R_gyro_rel @ R_slam_rel_body^T) recovers the true correction with
    <0.01 deg error, unlike three sign/frame variants that were tried and
    rejected by the same test.
    """
    if len(gyro_times) < 10:
        return []

    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    def R_body_world(t: float) -> np.ndarray:
        qx = np.interp(t, frame_pose.times, q[:, 0])
        qy = np.interp(t, frame_pose.times, q[:, 1])
        qz = np.interp(t, frame_pose.times, q[:, 2])
        qw = np.interp(t, frame_pose.times, q[:, 3])
        n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-9:
            return np.eye(3)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        x2, y2, z2 = qx * qx, qy * qy, qz * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        return np.array([
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ])

    t0 = max(float(gyro_times[0]), float(frame_pose.times[0]))
    t1 = min(float(gyro_times[-1]), float(frame_pose.times[-1]))
    if t1 <= t0:
        return []

    grid = np.arange(t0, t1, step_s)
    observations: list[tuple[float, float, np.ndarray, float]] = []
    for k in range(len(grid) - 1):
        ta, tb = float(grid[k]), float(grid[k + 1])
        R_gyro_rel = _integrate_gyro(gyro_times, gyro_wxyz, ta, tb)
        if R_gyro_rel is None:
            continue
        Ra = R_body_world(ta)
        Rb = R_body_world(tb)
        R_slam_rel_body = Ra.T @ Rb
        R_disc = R_gyro_rel @ R_slam_rel_body.T
        v_world = Ra @ _rotvec_from_matrix(R_disc)
        # High, uniform confidence: unlike vision, gyro integration quality
        # doesn't vary per-window with scene content, so a fixed weight
        # (tuned relative to vision's inlier-based 0..1 weight scale) is
        # appropriate rather than a per-observation confidence score.
        observations.append((ta, tb, v_world, 1.5))

    return observations


def _extract_lidar_windows(
    bag_path: str | Path,
    t_centers: list[float],
    window_s: float = 0.5,
    max_points_per_window: int = 15000,
) -> dict[int, np.ndarray]:
    """Single pass over the bag's LiDAR topic, bucketing raw body/LiDAR-frame
    points (x, y, z) into a +/-window_s window around each requested center
    time (indices into t_centers). One pass regardless of how many centers
    are requested -- same cost as bag_lidar_to_laz's full-topic scan.

    Returns {center_index: points[N,3]}; centers with no coverage are
    omitted.
    """
    bag_path = Path(bag_path)
    centers = np.asarray(t_centers, dtype=float)
    buckets: dict[int, list[np.ndarray]] = {i: [] for i in range(len(centers))}
    counts = {i: 0 for i in range(len(centers))}

    with _open_ros1_reader(bag_path) as reader:
        conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        if conn is None:
            return {}
        for _connection, msg_time_ns, rawdata in reader.messages(connections=[conn]):
            t_msg = msg_time_ns / 1e9
            near = np.nonzero(np.abs(centers - t_msg) <= window_s)[0]
            if len(near) == 0:
                continue
            x, y, z, _t, _intensity = _parse_custom_msg_lidar(bytes(rawdata), msg_time_ns)
            if len(x) == 0:
                continue
            pts = np.column_stack([x, y, z])
            for i in near:
                if counts[i] >= max_points_per_window:
                    continue
                buckets[i].append(pts)
                counts[i] += len(pts)

    out: dict[int, np.ndarray] = {}
    rng = np.random.default_rng(0)
    for i, chunks in buckets.items():
        if not chunks:
            continue
        arr = np.concatenate(chunks, axis=0)
        if len(arr) > max_points_per_window:
            idx = rng.choice(len(arr), max_points_per_window, replace=False)
            arr = arr[idx]
        out[i] = arr
    return out


def _icp_align(
    src_pts: np.ndarray,
    dst_pts: np.ndarray,
    init_R: np.ndarray | None = None,
    init_t: np.ndarray | None = None,
    max_iters: int = 30,
    tol: float = 1e-6,
    max_corr_dist: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, float, float] | None:
    """Point-to-point ICP: cKDTree nearest-neighbor correspondence + Kabsch/
    SVD rigid alignment, iterated to convergence.

    Returns (R, t, rms_error, inlier_ratio) such that dst ~= R @ src + t, or
    None if too few points/correspondences to trust the result. A good
    init_R/init_t matters -- point-to-point ICP has no global convergence
    guarantee, only local; callers should seed it from the existing SLAM
    trajectory estimate (a real revisit should only need a small correction).
    """
    from scipy.spatial import cKDTree

    if len(src_pts) < 50 or len(dst_pts) < 50:
        return None

    R = np.eye(3) if init_R is None else np.array(init_R, dtype=float)
    t = np.zeros(3) if init_t is None else np.array(init_t, dtype=float)

    tree = cKDTree(dst_pts)
    prev_rms: float | None = None
    rms = float("inf")
    inlier_ratio = 0.0
    for _ in range(max_iters):
        src_tf = (R @ src_pts.T).T + t
        dists, idx = tree.query(src_tf, k=1)
        mask = dists <= max_corr_dist
        if int(mask.sum()) < 30:
            return None
        matched_src = src_pts[mask]
        matched_dst = dst_pts[idx[mask]]
        inlier_ratio = float(mask.sum()) / len(src_pts)

        src_mean = matched_src.mean(axis=0)
        dst_mean = matched_dst.mean(axis=0)
        src_c = matched_src - src_mean
        dst_c = matched_dst - dst_mean
        H = src_c.T @ dst_c
        U, _S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T
        t = dst_mean - R @ src_mean

        rms = float(np.sqrt(np.mean(np.sum((matched_dst - ((R @ matched_src.T).T + t)) ** 2, axis=1))))
        if prev_rms is not None and abs(prev_rms - rms) < tol:
            break
        prev_rms = rms

    return R, t, rms, inlier_ratio


def _collect_lidar_loop_closure_observations(
    bag_path: str | Path,
    frame_pose: FramePose,
    step_s: float = 5.0,
    window_s: float = 0.5,
    min_gap_s: float = 20.0,
    max_pos_dist_m: float = 4.0,
    max_pairs: int = 60,
    min_points: int = 300,
    max_corr_dist: float = 0.5,
) -> list[tuple[float, float, np.ndarray, float]]:
    """LiDAR point-cloud ICP loop-closure observations: the technique
    established open-source LIO systems (LIO-SAM, FAST-LIO-SAM,
    FAST-LIO-GPS) actually use to correct SLAM orientation drift, as opposed
    to the camera-vision and naive-gyro approaches tried earlier in this
    project (see _collect_visual_observations / _collect_imu_observations
    docstrings and session notes) -- vision was limited by the narrow-FOV
    monochrome nav camera on a low-texture scene, and naive raw gyro
    integration showed an unexplained discrepancy against SLAM's own output
    that ruled out simple unit/sign/extrinsic bugs. Directly registering the
    LiDAR's own geometry against itself at a genuine revisit sidesteps both
    problems: it uses the same sensor the downstream point cloud is built
    from, at full range/precision, with no camera FOV or IMU-bias-drift
    limitation.

    Candidate pairs are found the same way as visual loop closures (genuine
    revisits: time gap + world-frame position proximity from frame_pose's
    own translation), but the observation is direct scan-to-scan ICP
    alignment rather than vision or gyro. ICP here produces a body-frame
    relative rotation (src=body_a points, dst=body_b points, structurally
    identical to gyro integration's R_gyro_rel), NOT a camera-relative
    rotation -- so it reuses the SAME validated body-frame formula as
    _collect_imu_observations (v_world = +R_slam(t_a) @ rotvec(R_rel @
    R_slam_rel_body^T)), not the different camera-frame formula from
    _collect_visual_observations.
    """
    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    def R_body_world(t: float) -> np.ndarray:
        qx = np.interp(t, frame_pose.times, q[:, 0])
        qy = np.interp(t, frame_pose.times, q[:, 1])
        qz = np.interp(t, frame_pose.times, q[:, 2])
        qw = np.interp(t, frame_pose.times, q[:, 3])
        n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if n < 1e-9:
            return np.eye(3)
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
        x2, y2, z2 = qx * qx, qy * qy, qz * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        return np.array([
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ])

    def sensor_pos(t: float) -> np.ndarray:
        return np.array([
            np.interp(t, frame_pose.times, frame_pose.tx),
            np.interp(t, frame_pose.times, frame_pose.ty),
            np.interp(t, frame_pose.times, frame_pose.tz),
        ])

    t0, t1 = float(frame_pose.times[0]), float(frame_pose.times[-1])
    if t1 <= t0:
        return []
    grid = np.arange(t0, t1, step_s)
    positions = {k: sensor_pos(float(g)) for k, g in enumerate(grid)}

    pairs: list[tuple[int, int]] = []
    for a in range(len(grid)):
        for b in range(a + 1, len(grid)):
            if grid[b] - grid[a] < min_gap_s:
                continue
            if np.linalg.norm(positions[b] - positions[a]) > max_pos_dist_m:
                continue
            pairs.append((a, b))
    if not pairs:
        return []
    if len(pairs) > max_pairs:
        idx = np.linspace(0, len(pairs) - 1, max_pairs).astype(int)
        pairs = [pairs[i] for i in idx]

    center_idx: dict[int, int] = {}
    centers: list[float] = []
    for a, b in pairs:
        for k in (a, b):
            if k not in center_idx:
                center_idx[k] = len(centers)
                centers.append(float(grid[k]))

    windows = _extract_lidar_windows(bag_path, centers, window_s=window_s)

    observations: list[tuple[float, float, np.ndarray, float]] = []
    for a, b in pairs:
        pts_a = windows.get(center_idx[a])
        pts_b = windows.get(center_idx[b])
        if pts_a is None or pts_b is None or len(pts_a) < min_points or len(pts_b) < min_points:
            continue

        ta, tb = float(grid[a]), float(grid[b])
        Ra = R_body_world(ta)
        Rb = R_body_world(tb)
        R_slam_rel_body = Ra.T @ Rb
        # NOTE the src/dst order: src=pts_b, dst=pts_a. Point-to-point ICP's
        # natural output (dst ~= R @ src + t) is a *coordinate transform*
        # (body_b points expressed in body_a frame needs Ra.T@Rb -- see
        # derivation below), which is the OPPOSITE direction from what a
        # naive "a then b" ordering would suggest, and the opposite of
        # _collect_visual_observations' camera-pair convention. Verified
        # numerically against a synthetic known-ground-truth scene+drift
        # (see session notes): src=pts_a/dst=pts_b (the first version of
        # this line) silently computed the transpose of the intended
        # rotation. Derivation: p_body_a = Ra.T@(P_world-Ta) and
        # p_body_b = Rb.T@(P_world-Tb) for the same world point P_world =>
        # p_body_a = (Ra.T@Rb)@p_body_b + Ra.T@(Tb-Ta), i.e. dst=body_a,
        # src=body_b, R=Ra.T@Rb=R_slam_rel_body (matching the gyro-derived
        # R_gyro_rel convention in _collect_imu_observations exactly).
        t_init = Ra.T @ (positions[b] - positions[a])

        result = _icp_align(
            pts_b, pts_a, init_R=R_slam_rel_body, init_t=t_init, max_corr_dist=max_corr_dist
        )
        if result is None:
            continue
        R_icp_rel, _t_icp_rel, rms, inlier_ratio = result
        if inlier_ratio < 0.3 or rms > max_corr_dist:
            continue

        R_disc = R_icp_rel @ R_slam_rel_body.T
        v_world = Ra @ _rotvec_from_matrix(R_disc)
        # ICP directly registers the LiDAR's own geometry -- the most direct
        # evidence available (see docstring), so weight it above both vision
        # (0..1 inlier-scaled) and gyro (fixed 1.5).
        weight = 2.5 * inlier_ratio
        observations.append((ta, tb, v_world, weight))

    return observations


def _solve_yaw_similarity_transform(src_pts: np.ndarray, dst_pts: np.ndarray):
    """Find R (3x3, rotation about Z only) and t (3,) such that dst ~= R @ src + t.

    A full 3D rigid (Procrustes/Kabsch) fit from GNSS position correspondences
    is ill-conditioned whenever the walked trajectory is close to a straight
    line (e.g. crossing a bridge or corridor): the rotation component about
    the direction of travel is then determined almost entirely by noise, and
    applying it scatters the whole point cloud incoherently even though the
    fit residuals look fine.

    The on-device LiDAR-inertial SLAM already gravity-aligns roll/pitch via
    the IMU, so only the yaw (heading about the vertical axis) needs
    correcting from GNSS -- and unlike full 3D rotation, 2D yaw + XY
    translation is well-conditioned from as few as two non-coincident points
    even on a perfectly straight path. Z is handled as an independent
    constant offset (SLAM's Z axis is assumed already vertical).
    """
    src_xy = src_pts[:, :2]
    dst_xy = dst_pts[:, :2]
    src_c = src_xy - src_xy.mean(axis=0)
    dst_c = dst_xy - dst_xy.mean(axis=0)
    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R2 = Vt.T @ np.diag([1.0, d]) @ U.T
    t_xy = dst_xy.mean(axis=0) - R2 @ src_xy.mean(axis=0)
    t_z = float((dst_pts[:, 2] - src_pts[:, 2]).mean())

    R = np.eye(3)
    R[:2, :2] = R2
    t = np.array([t_xy[0], t_xy[1], t_z])
    return R, t


def _smooth_residual_median(t: np.ndarray, vals: np.ndarray, window_s: float) -> np.ndarray:
    """Median-filter vals(t) over a sliding time window (t assumed sorted).

    The RTK-vs-SLAM residual used by georeference_from_slam is meant to
    correct genuine SLAM drift accumulated *within* a recording (see that
    function's docstring) -- but for an already self-consistent trajectory
    (e.g. a well-tuned FAST-LIO2 run) the raw per-epoch residual is
    dominated by RTK measurement noise (observed: Z residual std ~10m on a
    trajectory whose own internal roughness was ~4m) rather than real
    drift. Interpolating that raw noisy residual directly (as an earlier
    version of this function did) injects it straight into the point
    cloud: the same physical spot scanned at two different times gets two
    different noise-driven corrections and visibly splits apart. A window
    wide enough to average out epoch-to-epoch RTK noise, but much narrower
    than the full recording, keeps the ability to track real slow drift
    while rejecting that noise -- median (not mean) so an isolated bad
    epoch/multipath spike inside the window can't dominate it either.
    """
    n = len(t)
    out = np.empty(n)
    half = window_s / 2.0
    lo = 0
    hi = 0
    for i in range(n):
        while lo < n and t[lo] < t[i] - half:
            lo += 1
        if hi < i:
            hi = i
        while hi < n and t[hi] <= t[i] + half:
            hi += 1
        out[i] = np.median(vals[lo:hi])
    return out


def georeference_from_slam(
    src_laz: str | Path,
    frame_pose: FramePose,
    rtk_pos_path: str | Path,
    out_laz: str | Path,
    transform_out_path: str | Path | None = None,
    target_crs_epsg: int | None = None,
    residual_smoothing_window_s: float = 20.0,
    diagnostics_out_path: str | Path | None = None,
    target_crs_wkt: str | None = None,
) -> int:
    """Georeference SLAM-frame LAZ using RTK trajectory from bag.

    Finds a SLAM->UTM yaw + XY translation + Z offset by matching frame_pose
    body positions (LiDAR-IMU fused SLAM trajectory) against RTK GNSS
    positions at shared timestamps. Only yaw is corrected (not full 3D
    rotation): SLAM's own roll/pitch are already gravity-aligned via the IMU,
    and a full rigid fit is ill-conditioned whenever the walked path is close
    to a straight line. On top of that single global alignment, the RTK-vs-
    SLAM residual is also interpolated continuously over time and applied per
    point at its own gps_time -- a static transform only fixes the overall
    offset/heading, not drift accumulated *within* the recording (e.g. the
    same spot getting a slightly different world_slam position on revisit
    during a back-and-forth walk); this is the same role a GPS factor plays
    in pose-graph SLAM. Outputs LAZ with UTM CRS. Returns number of points.

    ALTERNATIVE: worker-gtsam/gtsam_georeference.py implements a joint GTSAM
    factor-graph correction (odometry BetweenFactor chain + GPSFactor per RTK
    epoch, solved together rather than sequentially) as a drop-in alternative
    for the FAST-LIO2 path -- it lives in its own isolated environment
    because gtsam requires numpy<2, incompatible with this module's own
    numpy>=2/scipy/opencv dependencies. See that script's docstring for the
    full picture, including that its own LiDAR loop-closure extension did not
    beat it, AND that on the one scan tested, FAST-LIO2's own front-end
    (worker-fastlio) turned out to matter at least as much as any backend
    correction here -- a previously-unapplied calibration.yaml IMU/LiDAR time
    offset alone closed nearly as much roughness gap as this whole function.

    NOTE: an earlier version of this function also tried to fit a
    *time-varying* yaw correction (sliding-window RTK/SLAM comparisons) on
    top of this, to catch on-device heading drift a single constant rotation
    can't. It was reverted: on this scan's narrow/elongated trajectory the
    local windows didn't carry enough independent directional information,
    so the fit was dominated by RTK position noise rather than real drift --
    each window's slightly different fitted angle fractured the point cloud
    into a disconnected zigzag instead of the coherent shape a single global
    yaw (verified to already closely match the vendor SLAM's own reference
    point cloud) produces. If on-device heading drift turns out to be a real
    problem on other scans, a future fix needs a robustness safeguard this
    one lacked -- e.g. requiring much wider windows/high correspondence
    counts before trusting a local fit over the global one, not just cosmetic
    smoothing after the fact.

    If diagnostics_out_path is given, writes a small text report with the
    number of matched RTK/SLAM epochs and the pre-smoothing RTK-vs-SLAM
    residual RMSE (horizontal and vertical) -- the same two numbers the
    vendor SLAM engine itself logs (as "匹配点对数" / "对齐误差") to flag when
    a fit is unreliable (e.g. a near-straight trajectory or poor RTK fix).
    This is informational only here (unlike the vendor, which can suppress
    RTK fusion entirely below its own threshold) -- there isn't yet enough
    data across scans to pick a reliable go/no-go cutoff for this codebase,
    so it's surfaced for a human to read rather than gating behaviour.
    """
    src_laz = Path(src_laz)
    rtk_pos_path = Path(rtk_pos_path)
    out_laz = Path(out_laz)

    rtk_times, rtk_lats, rtk_lons, rtk_alts = [], [], [], []
    with open(rtk_pos_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                dt = datetime.strptime(
                    f"{parts[0]} {parts[1]}", "%Y/%m/%d %H:%M:%S.%f"
                ).replace(tzinfo=UTC)
                rtk_times.append(dt.timestamp())
                rtk_lats.append(float(parts[2]))
                rtk_lons.append(float(parts[3]))
                rtk_alts.append(float(parts[4]))
            except ValueError:
                continue

    if len(rtk_times) < 4:
        raise ProcessingError(
            f"RTK trajectory has only {len(rtk_times)} epochs, need >= 4"
        )

    rtk_t = np.array(rtk_times)
    rtk_lat = np.array(rtk_lats)
    rtk_lon = np.array(rtk_lons)
    rtk_alt = np.array(rtk_alts)

    median_lon = float(np.median(rtk_lon))
    median_lat = float(np.median(rtk_lat))
    if target_crs_wkt:
        # Project-level override by full WKT -- for a custom/local projection
        # that has no EPSG code (e.g. the vendor's "FusionCRS_TM_87"). Takes
        # precedence over target_crs_epsg.
        utm_crs = CRS.from_wkt(target_crs_wkt)
    elif target_crs_epsg is not None:
        # Project-level override (e.g. a regional Gauss-Kruger zone matching
        # an existing local survey/GIS convention) instead of the default
        # auto-computed WGS84 UTM zone.
        utm_crs = CRS.from_epsg(target_crs_epsg)
    else:
        utm_zone = int((median_lon + 180.0) / 6.0) + 1
        hemisphere = "north" if median_lat >= 0 else "south"
        utm_crs_str = f"+proj=utm +zone={utm_zone} +{hemisphere} +datum=WGS84 +units=m +no_defs"
        utm_crs = CRS.from_string(utm_crs_str)
    to_utm = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    rtk_x, rtk_y = to_utm.transform(rtk_lon, rtk_lat)
    rtk_z = rtk_alt

    t_start = max(float(frame_pose.times[0]), float(rtk_t[0]))
    t_end = min(float(frame_pose.times[-1]), float(rtk_t[-1]))
    if t_end <= t_start:
        raise ProcessingError(
            f"No time overlap: SLAM [{frame_pose.times[0]:.1f}, {frame_pose.times[-1]:.1f}], "
            f"RTK [{rtk_t[0]:.1f}, {rtk_t[-1]:.1f}]"
        )

    mask = (rtk_t >= t_start) & (rtk_t <= t_end)
    if mask.sum() < 4:
        raise ProcessingError(f"Only {mask.sum()} RTK epochs in overlap, need >= 4")

    rtk_tm = rtk_t[mask]
    slam_x = np.interp(rtk_tm, frame_pose.times, frame_pose.tx)
    slam_y = np.interp(rtk_tm, frame_pose.times, frame_pose.ty)
    slam_z = np.interp(rtk_tm, frame_pose.times, frame_pose.tz)

    slam_pts = np.column_stack([slam_x, slam_y, slam_z])
    utm_pts = np.column_stack([rtk_x[mask], rtk_y[mask], rtk_z[mask]])
    R, t_vec = _solve_yaw_similarity_transform(slam_pts, utm_pts)

    # A single global similarity transform only corrects the SLAM trajectory's
    # overall offset/heading -- it cannot fix drift that accumulates *within*
    # the recording (e.g. the same physical spot getting a slightly different
    # world_slam position on each revisit during a back-and-forth walk). RTK
    # is available at ~10 Hz throughout, so instead of stopping at one static
    # correction, the residual between RTK and the globally-aligned SLAM
    # trajectory is computed at every RTK epoch and interpolated continuously
    # over time -- equivalent to a GPS-factor pose-graph correction, applied
    # per point at its own gps_time rather than once for the whole cloud.
    order = np.argsort(rtk_tm)
    res_t = rtk_tm[order]
    aligned_xy = (R[:2, :2] @ slam_pts[order, :2].T).T + t_vec[:2]
    aligned_z = slam_pts[order, 2] + t_vec[2]
    res_x = utm_pts[order, 0] - aligned_xy[:, 0]
    res_y = utm_pts[order, 1] - aligned_xy[:, 1]
    res_z = utm_pts[order, 2] - aligned_z

    horizontal_rmse = float(np.sqrt(np.mean(res_x**2 + res_y**2)))
    vertical_rmse = float(np.sqrt(np.mean(res_z**2)))
    n_matched = int(mask.sum())
    print(
        f"[georeference_from_slam] RTK/SLAM alignment: {n_matched} matched epochs, "
        f"horizontal RMSE={horizontal_rmse:.3f}m vertical RMSE={vertical_rmse:.3f}m "
        f"(pre-smoothing residual; large values mean either poor RTK quality "
        f"or a degenerate/near-straight matched trajectory)"
    )
    if diagnostics_out_path is not None:
        Path(diagnostics_out_path).write_text(
            "RTK/SLAM alignment diagnostics\n"
            f"matched_epochs: {n_matched}\n"
            f"horizontal_rmse_m: {horizontal_rmse:.4f}\n"
            f"vertical_rmse_m: {vertical_rmse:.4f}\n"
            f"residual_smoothing_window_s: {residual_smoothing_window_s}\n",
            encoding="utf-8",
        )

    # Smooth over RTK measurement noise before it gets baked into the point
    # cloud -- see _smooth_residual_median's docstring. The right window
    # width is source-dependent, not a universal constant: on kiss-icp
    # trajectories (real drift accumulates over seconds) a 20s window (the
    # default) tracks it well -- widening to 1000s measurably regressed
    # e6b4bbe7 from 9.5m to 11.5m median cell roughness in testing. On a
    # well-tuned FAST-LIO2 trajectory (self-consistent locally, drift is
    # much slower -- tens of meters over hundreds of seconds) the opposite
    # holds: 20s still lets that slow drift's local slope distort nearby
    # points (21m), while widening toward the whole recording's length
    # converges on the correct single near-static offset (3.9m) -- callers
    # should pass a much wider value for FAST-LIO2-sourced frame_pose.
    res_x = _smooth_residual_median(res_t, res_x, residual_smoothing_window_s)
    res_y = _smooth_residual_median(res_t, res_y, residual_smoothing_window_s)
    res_z = _smooth_residual_median(res_t, res_z, residual_smoothing_window_s)

    if transform_out_path is not None:
        # Saved so a later step (colorize, which needs points back in the
        # local SLAM frame to match frame_pose for camera projection) can
        # invert this exact transform rather than re-deriving its own.
        np.savez(
            transform_out_path,
            R=R, t=t_vec, res_t=res_t, res_x=res_x, res_y=res_y, res_z=res_z,
        )

    def _to_utm_drift_corrected(pts: np.ndarray, gps_t: np.ndarray) -> np.ndarray:
        utm = (R @ pts.T).T + t_vec
        t_clip = np.clip(gps_t, res_t[0], res_t[-1])
        utm[:, 0] += np.interp(t_clip, res_t, res_x)
        utm[:, 1] += np.interp(t_clip, res_t, res_y)
        utm[:, 2] += np.interp(t_clip, res_t, res_z)
        return utm

    CHUNK = 2_000_000
    x_min = y_min = z_min = np.inf

    with laspy.open(src_laz) as reader:
        for chunk in reader.chunk_iterator(CHUNK):
            pts = np.column_stack([
                np.asarray(chunk.x, dtype=np.float64),
                np.asarray(chunk.y, dtype=np.float64),
                np.asarray(chunk.z, dtype=np.float64),
            ])
            utm = _to_utm_drift_corrected(pts, np.asarray(chunk.gps_time, dtype=np.float64))
            x_min = min(x_min, float(utm[:, 0].min()))
            y_min = min(y_min, float(utm[:, 1].min()))
            z_min = min(z_min, float(utm[:, 2].min()))

    n_out = 0
    with laspy.open(src_laz) as reader:
        # Rebuilt fresh rather than copy.deepcopy(reader.header)/deepcopy(chunk):
        # laspy's point-record deepcopy has a known recursion bug on large
        # records (RecursionError), so records are reconstructed explicitly
        # instead -- same pattern as colorize_laz/bag_lidar_to_laz.
        hdr = laspy.LasHeader(point_format=reader.header.point_format, version=reader.header.version)
        hdr.add_crs(utm_crs)
        hdr.offsets = np.array([x_min, y_min, z_min])
        hdr.scales = np.array([0.001, 0.001, 0.001])
        with laspy.open(out_laz, mode="w", header=hdr) as writer:
            for chunk in reader.chunk_iterator(CHUNK):
                pts = np.column_stack([
                    np.asarray(chunk.x, dtype=np.float64),
                    np.asarray(chunk.y, dtype=np.float64),
                    np.asarray(chunk.z, dtype=np.float64),
                ])
                utm = _to_utm_drift_corrected(pts, np.asarray(chunk.gps_time, dtype=np.float64))
                p = laspy.ScaleAwarePointRecord.zeros(len(chunk), header=hdr)
                for dim in chunk.point_format.dimension_names:
                    if dim in ("X", "Y", "Z"):
                        continue
                    setattr(p, dim, np.asarray(getattr(chunk, dim)))
                p.x = utm[:, 0]
                p.y = utm[:, 1]
                p.z = utm[:, 2]
                writer.write_points(p)
                n_out += len(chunk)

    return n_out


def _load_calibration_camera(cal_path: str | Path | None):
    """Load nav camera intrinsics + LiDAR->camera extrinsics from calibration.yaml.

    The file is OpenCV FileStorage YAML: a "%YAML:1.0" header (not valid
    YAML -- PyYAML requires a space, "%YAML 1.0") and "!!opencv-matrix" tags
    on every matrix node, both of which make plain yaml.safe_load raise
    immediately. Both are stripped before parsing; !!opencv-matrix nodes then
    fall through as ordinary {rows, cols, dt, data} mappings, which is
    exactly the shape already handled below.
    """
    if cal_path is None or not Path(cal_path).exists():
        return None
    try:
        import re
        import yaml
        with open(cal_path, encoding="utf-8") as f:
            text = f.read()
        text = re.sub(r"^%YAML:[\d.]+\s*\n", "", text)
        text = text.replace("!!opencv-matrix", "")
        d = yaml.safe_load(text)
        if not d:
            return None
        intrinsic = d.get("intrinsic", d)
        cam = None
        for key in ("fisheye_middle", "usb_cam", "nav_cam", "camera", "cam0"):
            if key in intrinsic:
                cam = intrinsic[key]
                break
        if cam is None and "camera_matrix" in intrinsic:
            cam = intrinsic
        if cam is None:
            return None
        K_raw = cam.get("camera_matrix") or cam.get("K")
        if K_raw is None:
            return None
        if isinstance(K_raw, dict):
            K_data = K_raw.get("data", [])
        elif K_raw and isinstance(K_raw[0], list):
            K_data = [v for row in K_raw for v in row]
        else:
            K_data = list(K_raw)
        if len(K_data) < 9:
            return None
        fx, fy = float(K_data[0]), float(K_data[4])
        cx, cy = float(K_data[2]), float(K_data[5])

        R_cam_in_body = np.eye(3, dtype=np.float64)
        t_cam_in_body = np.zeros(3, dtype=np.float64)
        ext_raw = d.get("extrinsic", {}).get("lidar_middlecamera")
        if ext_raw is not None:
            ext_data = ext_raw.get("data", []) if isinstance(ext_raw, dict) else list(ext_raw)
            if len(ext_data) >= 16:
                # lidar_middlecamera maps a LiDAR/body-frame point into the
                # camera frame (p_cam = R_lc @ p_body + t_lc); colorize_laz
                # needs the inverse -- camera pose expressed in body frame
                # (p_body = R_cam_in_body @ p_cam + t_cam_in_body).
                M = np.array(ext_data[:16], dtype=np.float64).reshape(4, 4)
                R_lc = M[:3, :3]
                t_lc = M[:3, 3]
                R_cam_in_body = R_lc.T
                t_cam_in_body = -R_lc.T @ t_lc

        return (fx, fy, cx, cy, R_cam_in_body, t_cam_in_body)
    except Exception:
        return None


def colorize_laz(
    laz_path: str | Path,
    cam_zip_path: str | Path,
    frame_pose: FramePose,
    out_laz: str | Path,
    calibration_path: str | Path | None = None,
    transform_path: str | Path | None = None,
) -> int:
    """Colorize a LiDAR LAZ by projecting nav-cam frames onto points.

    frame_pose's trajectory (and therefore the camera-pose interpolation
    used for projection) is always in the local SLAM frame. If laz_path has
    already been georeferenced (its points are in UTM, not local SLAM
    metres), pass the transform.npz written by georeference_from_slam via
    transform_path -- it is inverted to recover local-frame coordinates for
    the projection math only; the point cloud is still written out with its
    original (georeferenced) coordinates unchanged.
    """
    try:
        from PIL import Image as _PILImage
    except ImportError as exc:
        raise ProcessingError("Pillow not available: pip install Pillow") from exc

    laz_path = Path(laz_path)
    cam_zip_path = Path(cam_zip_path)
    out_laz = Path(out_laz)

    cal = _load_calibration_camera(calibration_path)
    if cal is not None:
        fx, fy, cx_px, cy_px, R_cam_in_body, t_cam_in_body = cal
    else:
        fx = fy = 500.0
        cx_px, cy_px = 320.0, 240.0
        R_cam_in_body = np.eye(3, dtype=np.float64)
        t_cam_in_body = np.zeros(3, dtype=np.float64)
    R_bic = R_cam_in_body.T

    with zipfile.ZipFile(cam_zip_path) as zf:
        all_names = sorted(zf.namelist())
    nav_names = sorted(n for n in all_names if n.startswith("nav_") and n.endswith(".jpg"))
    if not nav_names:
        nav_names = sorted(n for n in all_names if n.startswith("left_") and n.endswith(".jpg"))
    if not nav_names:
        import shutil
        shutil.copy2(laz_path, out_laz)
        return 0

    nav_ts = np.array(
        [int(Path(n).stem.split("_", 1)[1]) / 1e9 for n in nav_names],
        dtype=np.float64,
    )

    fp_times = frame_pose.times
    t_min, t_max = float(fp_times[0]), float(fp_times[-1])
    _q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for _i in range(1, len(_q)):
        if np.dot(_q[_i], _q[_i - 1]) < 0:
            _q[_i] = -_q[_i]
    _fp_qx, _fp_qy, _fp_qz, _fp_qw = _q[:, 0], _q[:, 1], _q[:, 2], _q[:, 3]

    las = laspy.read(laz_path)
    x_arr = np.array(las.x, dtype=np.float64)
    y_arr = np.array(las.y, dtype=np.float64)
    z_arr = np.array(las.z, dtype=np.float64)
    n_pts = len(x_arr)

    if hasattr(las, "gps_time") and las.gps_time is not None and len(las.gps_time):
        pt_ts = np.asarray(las.gps_time, dtype=np.float64)
    else:
        pt_ts = np.linspace(t_min, t_max, n_pts)
    pt_ts_c = np.clip(pt_ts, t_min, t_max)

    if transform_path is not None and Path(transform_path).exists():
        # laz_path is already georeferenced (UTM); invert that transform to
        # get the local SLAM-frame coordinates the projection math below
        # needs, without altering the coordinates actually written out.
        tdata = np.load(transform_path)
        R_su = tdata["R"]; t_su = tdata["t"]
        res_t = tdata["res_t"]; res_x = tdata["res_x"]; res_y = tdata["res_y"]; res_z = tdata["res_z"]
        t_clip_res = np.clip(pt_ts, res_t[0], res_t[-1])
        dx = np.interp(t_clip_res, res_t, res_x)
        dy = np.interp(t_clip_res, res_t, res_y)
        dz = np.interp(t_clip_res, res_t, res_z)
        utm_pts = np.column_stack([x_arr - dx, y_arr - dy, z_arr - dz])
        slam_pts = (R_su.T @ (utm_pts - t_su).T).T
        x_proj, y_proj, z_proj = slam_pts[:, 0], slam_pts[:, 1], slam_pts[:, 2]
    else:
        x_proj, y_proj, z_proj = x_arr, y_arr, z_arr

    R_out = np.zeros(n_pts, dtype=np.uint8)
    G_out = np.zeros(n_pts, dtype=np.uint8)
    B_out = np.zeros(n_pts, dtype=np.uint8)
    n_coloured = 0

    ni = np.searchsorted(nav_ts, pt_ts_c)
    ni = np.clip(ni, 0, len(nav_ts) - 1)
    li = np.maximum(ni - 1, 0)
    ni = np.where(np.abs(pt_ts_c - nav_ts[li]) < np.abs(pt_ts_c - nav_ts[ni]), li, ni)

    with zipfile.ZipFile(cam_zip_path) as zf:
        for fi, nm in enumerate(nav_names):
            mask = ni == fi
            if not mask.any():
                continue
            try:
                img = np.array(
                    _PILImage.open(io.BytesIO(zf.read(nm))).convert("RGB"),
                    dtype=np.uint8,
                )
                h_img, w_img = img.shape[:2]
            except Exception:
                continue

            t_f = np.clip(nav_ts[fi], t_min, t_max)
            tx = float(np.interp(t_f, fp_times, frame_pose.tx))
            ty = float(np.interp(t_f, fp_times, frame_pose.ty))
            tz = float(np.interp(t_f, fp_times, frame_pose.tz))
            qx = float(np.interp(t_f, fp_times, _fp_qx))
            qy = float(np.interp(t_f, fp_times, _fp_qy))
            qz = float(np.interp(t_f, fp_times, _fp_qz))
            qw = float(np.interp(t_f, fp_times, _fp_qw))
            nr = (qx*qx + qy*qy + qz*qz + qw*qw) ** 0.5
            if nr < 1e-9:
                continue
            qx /= nr; qy /= nr; qz /= nr; qw /= nr

            R_bw = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qw*qz),  2*(qx*qz+qw*qy)],
                [2*(qx*qy+qw*qz),  1-2*(qx*qx+qz*qz), 2*(qy*qz-qw*qx)],
                [2*(qx*qz-qw*qy),  2*(qy*qz+qw*qx),  1-2*(qx*qx+qy*qy)],
            ], dtype=np.float64)
            t_bw = np.array([tx, ty, tz], dtype=np.float64)

            pts_w = np.column_stack([x_proj[mask], y_proj[mask], z_proj[mask]])
            pts_body = (R_bw.T @ (pts_w - t_bw).T).T
            pts_cam = (R_bic @ (pts_body - t_cam_in_body).T).T

            valid = pts_cam[:, 2] > 0.1
            if not valid.any():
                continue

            u = fx * pts_cam[valid, 0] / pts_cam[valid, 2] + cx_px
            v = fy * pts_cam[valid, 1] / pts_cam[valid, 2] + cy_px
            ui = np.round(u).astype(np.int32)
            vi = np.round(v).astype(np.int32)
            in_img = (ui >= 0) & (ui < w_img) & (vi >= 0) & (vi < h_img)
            if not in_img.any():
                continue

            ci = np.where(mask)[0][valid][in_img]
            R_out[ci] = img[vi[in_img], ui[in_img], 0]
            G_out[ci] = img[vi[in_img], ui[in_img], 1]
            B_out[ci] = img[vi[in_img], ui[in_img], 2]
            n_coloured += int(in_img.sum())

    hdr = laspy.LasHeader(point_format=7, version="1.4")
    hdr.offsets = np.array([x_arr.min(), y_arr.min(), z_arr.min()])
    hdr.scales = np.array([0.001, 0.001, 0.001])
    try:
        src_crs = laspy.read(laz_path).header.parse_crs()
        if src_crs is not None:
            hdr.add_crs(src_crs)
    except Exception:
        pass

    with laspy.open(out_laz, mode="w", header=hdr) as wrt:
        for i in range(0, n_pts, 1_000_000):
            sl = slice(i, i + 1_000_000)
            p = laspy.ScaleAwarePointRecord.zeros(len(x_arr[sl]), header=hdr)
            p.x = x_arr[sl]
            p.y = y_arr[sl]
            p.z = z_arr[sl]
            p.red   = R_out[sl].astype(np.uint16) * 257
            p.green = G_out[sl].astype(np.uint16) * 257
            p.blue  = B_out[sl].astype(np.uint16) * 257
            if hasattr(las, "intensity") and las.intensity is not None:
                p.intensity = np.asarray(las.intensity[sl], dtype=np.uint16)
            if hasattr(las, "gps_time") and las.gps_time is not None:
                p.gps_time = np.asarray(las.gps_time[sl], dtype=np.float64)
            wrt.write_points(p)

    return n_coloured


def _quat_from_matrix(R: np.ndarray) -> tuple[float, float, float, float]:
    """Standard trace-based rotation-matrix -> quaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def bag_to_kiss_icp_trajectory(
    bag_zip_or_bag_path: str | Path,
    workdir: str | Path,
    min_range: float = 0.5,
    max_range: float = 100.0,
    min_points: int = 50,
) -> FramePose:
    """Recompute the sensor trajectory from raw LiDAR geometry alone, using
    kiss-icp (PRBonn, pip-installable, no ROS) -- a mature, KITTI-benchmarked
    LiDAR odometry pipeline with adaptive-threshold correspondence rejection
    and an incremental voxel-hashed local map (scan-to-map, not isolated
    scan-to-scan windows).

    This replaces two earlier, both-abandoned attempts at the same underlying
    goal (a from-scratch trajectory independent of the vendor's on-device
    SLAM, per literature on FAST-LIO2/LIO-SAM-family systems -- see session
    notes):
      - A hand-rolled windowed point-to-point ICP loop-closure corrector
        (_collect_lidar_loop_closure_observations): synthetically validated
        but converged to wrong local minima on this device's real (sparse/
        possibly-repetitive) geometry, making results worse (13.3m -> 27.2m
        roughness on scan e6b4bbe7).
      - A full ROS2 FAST-LIO2 integration (separate worker-fastlio service,
        real hku-mars/FAST_LIO): built and ran successfully end-to-end, but
        its IMU-coupled EKF diverged to multi-km trajectories within ~10-30s
        of any real motion despite the raw IMU stream itself testing
        completely physically sane throughout (median accel magnitude
        9.797 m/s^2, no NaN/Inf) -- exhausted sign flips, replay-speed
        variation, and all 6+3 gyro/accel axis permutations without finding
        a working configuration; root cause remains unidentified as a
        FAST-LIO2/EKF-internals issue, not a data problem.

    kiss-icp sidesteps both failure modes: no IMU coupling to get wrong
    (positions/orientations come purely from LiDAR scan-to-map registration,
    with the SAME adaptive-threshold robustness that the hand-rolled ICP
    attempt lacked), and it's a stable, widely-used, actively-maintained
    library rather than project-specific glue code.

    Returns a FramePose at one sample per LiDAR scan (~10 Hz, comparable
    density to the vendor's on-device frame_pose.txt) -- deliberately NOT
    IMU-rate, since there is no IMU coupling here.
    """
    from kiss_icp.config.config import DataConfig, MappingConfig
    from kiss_icp.config.parser import KISSConfig
    from kiss_icp.kiss_icp import KissICP

    workdir = Path(workdir)
    tmp_dir = workdir / "_kiss_icp_extract"
    bag_path = _resolve_bag_path(bag_zip_or_bag_path, tmp_dir)
    if bag_path is None or not bag_path.exists():
        raise ProcessingError(f"No .bag file found in {bag_zip_or_bag_path}")

    # voxel_size=None ("take it from data") is only resolved by kiss-icp's
    # own CLI config loader (config/parser.py: max_range/100.0 if unset) --
    # bypassed here since we build KISSConfig directly, so compute it the
    # same way ourselves or the pybind VoxelHashMap constructor rejects None.
    config = KISSConfig(
        data=DataConfig(min_range=min_range, max_range=max_range),
        mapping=MappingConfig(voxel_size=max_range / 100.0),
    )
    odometry = KissICP(config=config)

    times: list[float] = []
    tx: list[float] = []
    ty: list[float] = []
    tz: list[float] = []
    qx: list[float] = []
    qy: list[float] = []
    qz: list[float] = []
    qw: list[float] = []

    with _open_ros1_reader(bag_path) as reader:
        conn = next((c for c in reader.connections if c.topic in _LIDAR_TOPICS), None)
        if conn is None:
            available = sorted({c.topic for c in reader.connections})
            raise ProcessingError(f"No LiDAR topic found. Available: {available}")

        for _connection, msg_time_ns, rawdata in reader.messages(connections=[conn]):
            x, y, z, t, _intensity = _parse_custom_msg_lidar(bytes(rawdata), msg_time_ns)
            if len(x) < min_points:
                continue
            frame = np.column_stack([x, y, z]).astype(np.float64)
            # Per-point relative time for kiss-icp's motion-compensation
            # (deskew): the C++ preprocessor only needs consistent relative
            # ordering/spacing within this one scan, not any particular
            # absolute epoch or unit -- matches how kiss-icp's own dataset
            # loaders pass raw hardware time fields straight through
            # unnormalized (see kiss_icp/tools/point_cloud2.py).
            odometry.register_frame(frame, t)
            pose = odometry.last_pose
            qx_, qy_, qz_, qw_ = _quat_from_matrix(pose[:3, :3])
            times.append(msg_time_ns / 1e9)
            tx.append(float(pose[0, 3]))
            ty.append(float(pose[1, 3]))
            tz.append(float(pose[2, 3]))
            qx.append(qx_)
            qy.append(qy_)
            qz.append(qz_)
            qw.append(qw_)

    if not times:
        raise ProcessingError("No LiDAR scans processed by kiss-icp")

    return FramePose(
        times=np.array(times), tx=np.array(tx), ty=np.array(ty), tz=np.array(tz),
        qx=np.array(qx), qy=np.array(qy), qz=np.array(qz), qw=np.array(qw),
    )


def _estimate_ground_plane_normal(
    points: np.ndarray,
    up_hint: np.ndarray = np.array([0.0, 0.0, 1.0]),
    max_angle_deg: float = 60.0,
    dist_threshold: float = 0.15,
    n_iter: int = 500,
    max_points: int = 200_000,
    height_percentile: float = 30.0,
    seed: int = 0,
) -> np.ndarray:
    """RANSAC-fit the dominant near-horizontal plane (ground/floor) in a
    point cloud. Candidate planes are restricted to those whose normal lies
    within max_angle_deg of up_hint, so a large vertical wall (often more
    numerous in points than floor returns from a chest/head-mounted LiDAR)
    doesn't get picked over the actual ground. Returns a unit normal vector
    oriented to have positive dot product with up_hint.

    Additionally pre-filters to the lowest height_percentile of points by
    height along up_hint before RANSAC: walls/objects span the full height
    range while true floor/ground returns concentrate near the bottom, so
    this biases candidate-point sampling toward genuine ground even in
    smaller (e.g. single time-window) point sets where a wall or other
    large flat surface could otherwise dominate by sheer point count.
    """
    height = points @ (up_hint / np.linalg.norm(up_hint))
    thresh = np.percentile(height, height_percentile)
    points = points[height <= thresh]

    rng = np.random.default_rng(seed)
    if len(points) > max_points:
        idx = rng.choice(len(points), max_points, replace=False)
        points = points[idx]
    n = len(points)
    if n < 3:
        return up_hint / np.linalg.norm(up_hint)
    cos_thresh = np.cos(np.radians(max_angle_deg))
    best_inliers = -1
    best_normal = up_hint / np.linalg.norm(up_hint)
    for _ in range(n_iter):
        i, j, k = rng.choice(n, 3, replace=False)
        p0, p1, p2 = points[i], points[j], points[k]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        if normal @ up_hint < 0:
            normal = -normal
        if (normal @ up_hint) < cos_thresh:
            continue
        d = -(normal @ p0)
        dist = np.abs(points @ normal + d)
        inliers = int((dist < dist_threshold).sum())
        if inliers > best_inliers:
            best_inliers = inliers
            best_normal = normal
    return best_normal


def _rotation_aligning_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation matrix R (3x3) such that R @ a ~= b, for unit vectors a, b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))


def level_frame_pose_to_ground(frame_pose: FramePose, sample_points: np.ndarray) -> FramePose:
    """Correct a whole trajectory's global tilt by leveling the point
    cloud's dominant ground plane to true horizontal (world Z).

    Needed specifically for kiss-icp (bag_to_kiss_icp_trajectory): pure
    LiDAR odometry has no gravity reference at all (no IMU), so its local
    "world" frame's Z-axis is whatever the first scan's frame happened to
    be, and slow ICP registration bias across ~5000+ scans can accumulate
    real tilt over a multi-minute recording (empirically observed on scan
    e6b4bbe7: 144m of Z-span for a person walking on ~flat ground, vs a
    single-digit-meter span expected -- see session notes). The existing
    georeference_from_slam step only does a yaw-only similarity transform
    against RTK (deliberately -- RTK position alone can't observe roll/
    pitch, see its own docstring), so it does NOT and cannot fix this; it
    assumes the input trajectory's Z is already gravity-aligned, which used
    to be guaranteed by the vendor's own IMU-fused on-device SLAM and is
    simply not true for kiss-icp's IMU-free output.

    One GLOBAL rotation (not per-frame) is fitted from the whole
    accumulated local-frame point cloud and applied to every pose --
    correcting a single net accumulated tilt. If the true drift is
    significantly time-varying (not a roughly-constant per-registration
    bias), this will only partially correct it; that would show up as
    still-elevated roughness after this correction and point toward needing
    a time-varying (piecewise) leveling instead.
    """
    normal = _estimate_ground_plane_normal(sample_points)
    R_level = _rotation_aligning_vectors(normal, np.array([0.0, 0.0, 1.0]))

    pos = np.column_stack([frame_pose.tx, frame_pose.ty, frame_pose.tz])
    pos_new = (R_level @ pos.T).T

    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    new_q = np.zeros_like(q)
    for k in range(len(frame_pose.times)):
        qx_, qy_, qz_, qw_ = q[k]
        n_ = np.sqrt(qx_ * qx_ + qy_ * qy_ + qz_ * qz_ + qw_ * qw_)
        if n_ < 1e-9:
            new_q[k] = q[k]
            continue
        qx_, qy_, qz_, qw_ = qx_ / n_, qy_ / n_, qz_ / n_, qw_ / n_
        x2, y2, z2 = qx_ * qx_, qy_ * qy_, qz_ * qz_
        wx, wy, wz = qw_ * qx_, qw_ * qy_, qw_ * qz_
        xy, xz, yz = qx_ * qy_, qx_ * qz_, qy_ * qz_
        R_orig = np.array([
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ])
        R_corr = R_level @ R_orig
        new_q[k] = _quat_from_matrix(R_corr)

    return FramePose(
        times=frame_pose.times,
        tx=pos_new[:, 0], ty=pos_new[:, 1], tz=pos_new[:, 2],
        qx=new_q[:, 0], qy=new_q[:, 1], qz=new_q[:, 2], qw=new_q[:, 3],
    )


def level_frame_pose_to_ground_timevarying(
    frame_pose: FramePose,
    local_points: np.ndarray,
    local_point_times: np.ndarray,
    window_s: float = 60.0,
    min_window_points: int = 2000,
) -> FramePose:
    """Time-varying version of level_frame_pose_to_ground: a single global
    leveling rotation only removes the AVERAGE tilt over the whole
    recording, but kiss-icp's Z-drift (no gravity reference at all -- see
    level_frame_pose_to_ground's docstring) accumulates non-uniformly across
    a multi-minute walk. Empirically confirmed on scan e6b4bbe7: a single
    global correction nearly halved roughness (20.6m -> 10.2m) but left it
    still 24x worse than the vendor reference (0.42m) -- exactly the
    signature of a residual time-varying component a single rotation can't
    reach.

    Splits the recording into window_s-second windows, estimates the ground
    plane independently per window (from the ORIGINAL, unleveled local-frame
    point cloud), and builds a smoothly-interpolated per-pose correction the
    same way visually_correct_frame_pose/_fit_pose_graph_correction already
    do elsewhere in this module: piecewise-linear in rotation-vector space
    between window-center control points, then converted to a rotation
    matrix per pose. Applied to orientation directly (R_new = R_level(t) @
    R_orig(t), world-frame left-multiplicative, the same convention as
    visually_correct_frame_pose) and to position INCREMENTALLY -- each
    consecutive-pose displacement is re-rotated by the LOCAL correction at
    that time before being re-accumulated, rather than rotating absolute
    positions by a single global rotation (which would tear the path apart
    given the correction now varies over time).
    """
    t0, t1 = float(frame_pose.times[0]), float(frame_pose.times[-1])
    grid = np.arange(t0, t1 + window_s, window_s)
    if len(grid) < 2:
        return frame_pose

    ctrl_t: list[float] = []
    ctrl_normal: list[np.ndarray] = []
    prev_normal = np.array([0.0, 0.0, 1.0])
    for k in range(len(grid) - 1):
        wa, wb = grid[k], grid[k + 1]
        mask = (local_point_times >= wa) & (local_point_times < wb)
        if int(mask.sum()) < min_window_points:
            continue
        normal = _estimate_ground_plane_normal(local_points[mask], up_hint=prev_normal)
        ctrl_t.append(float((wa + wb) / 2))
        ctrl_normal.append(normal)
        prev_normal = normal

    if len(ctrl_t) < 2:
        # Not enough windows with sufficient ground coverage -- fall back to
        # the single global correction rather than no correction at all.
        return level_frame_pose_to_ground(frame_pose, local_points)

    ctrl_t = np.array(ctrl_t)
    ctrl_rotvec = np.array([
        _rotvec_from_matrix(_rotation_aligning_vectors(n, np.array([0.0, 0.0, 1.0])))
        for n in ctrl_normal
    ])

    t_clip = np.clip(frame_pose.times, ctrl_t[0], ctrl_t[-1])
    delta = np.column_stack([
        np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 0]),
        np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 1]),
        np.interp(t_clip, ctrl_t, ctrl_rotvec[:, 2]),
    ])

    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    pos_orig = np.column_stack([frame_pose.tx, frame_pose.ty, frame_pose.tz])
    pos_new = np.zeros_like(pos_orig)
    pos_new[0] = pos_orig[0]
    new_q = np.zeros_like(q)

    for k in range(len(frame_pose.times)):
        R_level_k = _matrix_from_rotvec(delta[k])

        qx_, qy_, qz_, qw_ = q[k]
        n_ = np.sqrt(qx_ * qx_ + qy_ * qy_ + qz_ * qz_ + qw_ * qw_)
        if n_ < 1e-9:
            new_q[k] = q[k]
        else:
            qx_, qy_, qz_, qw_ = qx_ / n_, qy_ / n_, qz_ / n_, qw_ / n_
            x2, y2, z2 = qx_ * qx_, qy_ * qy_, qz_ * qz_
            wx, wy, wz = qw_ * qx_, qw_ * qy_, qw_ * qz_
            xy, xz, yz = qx_ * qy_, qx_ * qz_, qy_ * qz_
            R_orig = np.array([
                [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
            ])
            new_q[k] = _quat_from_matrix(R_level_k @ R_orig)

        if k > 0:
            delta_orig = pos_orig[k] - pos_orig[k - 1]
            pos_new[k] = pos_new[k - 1] + R_level_k @ delta_orig

    return FramePose(
        times=frame_pose.times,
        tx=pos_new[:, 0], ty=pos_new[:, 1], tz=pos_new[:, 2],
        qx=new_q[:, 0], qy=new_q[:, 1], qz=new_q[:, 2], qw=new_q[:, 3],
    )


def compute_kiss_icp_leveled_trajectory(
    bag_zip_or_bag_path: str | Path,
    workdir: str | Path,
    window_s: float = 60.0,
) -> FramePose:
    """One-call wrapper: kiss-icp trajectory + ground-plane leveling
    (bag_to_kiss_icp_trajectory + level_frame_pose_to_ground_timevarying),
    used in place of the vendor's on-device frame_pose.txt for BAG-path
    scans. See both functions' docstrings for the full rationale/history;
    validated on scan e6b4bbe7 (median cell Z-range roughness): 13.3m
    vision-only-corrected vendor trajectory -> 9.6m kiss-icp + leveled.

    Leveling needs a first-pass local-frame point cloud to estimate the
    ground plane against, so this internally does two bag_lidar_to_laz
    passes: one with the raw (unleveled) kiss-icp trajectory just to get
    points+times for the ground-plane fit, discarded afterward -- callers
    should still do their own (real, kept) bag_lidar_to_laz call with the
    returned, already-leveled FramePose.
    """
    workdir = Path(workdir)
    kiss_fp = bag_to_kiss_icp_trajectory(bag_zip_or_bag_path, workdir)

    unleveled_laz = workdir / "_kiss_icp_unleveled_probe.laz"
    bag_lidar_to_laz(bag_zip_or_bag_path, unleveled_laz, kiss_fp)
    las = laspy.read(unleveled_laz)
    local_points = np.column_stack([
        np.asarray(las.x, dtype=np.float64),
        np.asarray(las.y, dtype=np.float64),
        np.asarray(las.z, dtype=np.float64),
    ])
    local_point_times = np.asarray(las.gps_time, dtype=np.float64)

    return level_frame_pose_to_ground_timevarying(
        kiss_fp, local_points, local_point_times, window_s=window_s
    )


def _collect_kiss_icp_loop_closure_observations(
    local_points: np.ndarray,
    local_point_times: np.ndarray,
    frame_pose: FramePose,
    step_s: float = 5.0,
    window_s: float = 2.0,
    min_gap_s: float = 20.0,
    max_pos_dist_m: float = 3.0,
    max_pairs: int = 40,
    min_points: int = 500,
    max_corr_dist: float = 0.5,
) -> list[tuple[float, float, np.ndarray, np.ndarray, float]]:
    """Find genuine revisits in a kiss-icp trajectory and directly measure
    the WORLD-FRAME drift between them via point-cloud ICP.

    Structurally different from the earlier (reverted) hand-rolled
    loop-closure attempt in this module
    (_collect_lidar_loop_closure_observations) in the two ways that
    directly address why that one failed (converged to wrong local minima
    on this device's real geometry, made results worse: 13.3m -> 27.2m
    roughness -- see that function's docstring/session notes):

      1. local_points/local_point_times come from bag_lidar_to_laz with the
         kiss-icp trajectory already applied -- points are already in a
         common WORLD frame, not raw body frame, so ICP(window_j ->
         window_i) directly yields the world-frame correction needed at a
         revisit; no body-frame conjugation formula required.
      2. The ICP initial guess is IDENTITY. kiss-icp's own trajectory is
         already locally self-consistent (unlike the vendor's on-device
         trajectory that fed the earlier attempt), so there's no large,
         frequently-wrong a-priori guess steering ICP into a bad local
         minimum. Windows are also wider (2s vs 0.5s) for more
         distinguishing geometric structure.

    Returns (t_i, t_j, rotvec_world, translation_world, weight) tuples: the
    correction that should be applied starting at t_j (the later visit) to
    bring it into alignment with t_i (the earlier, assumed-trustworthy
    anchor) -- fed into _fit_pose_graph_correction (rotation) and
    _fit_translation_correction (translation), which jointly resolve
    multiple such pairwise observations against a smooth, anchored
    correction curve rather than trusting any single pair in isolation.
    """
    def sensor_pos(t: float) -> np.ndarray:
        return np.array([
            np.interp(t, frame_pose.times, frame_pose.tx),
            np.interp(t, frame_pose.times, frame_pose.ty),
            np.interp(t, frame_pose.times, frame_pose.tz),
        ])

    t0, t1 = float(frame_pose.times[0]), float(frame_pose.times[-1])
    if t1 <= t0:
        return []
    grid = np.arange(t0, t1, step_s)
    positions = {k: sensor_pos(float(g)) for k, g in enumerate(grid)}

    pairs: list[tuple[int, int]] = []
    for a in range(len(grid)):
        for b in range(a + 1, len(grid)):
            if grid[b] - grid[a] < min_gap_s:
                continue
            if np.linalg.norm(positions[b] - positions[a]) > max_pos_dist_m:
                continue
            pairs.append((a, b))
    if not pairs:
        return []
    if len(pairs) > max_pairs:
        idx = np.linspace(0, len(pairs) - 1, max_pairs).astype(int)
        pairs = [pairs[i] for i in idx]

    # Downsample per-window points before ICP -- a 2s window over a ~34M-
    # point full-resolution recording can hold hundreds of thousands of
    # points; building a cKDTree and running up to 30 ICP iterations on
    # that many points per pair, times up to max_pairs pairs, made this
    # take tens of minutes in practice (found empirically: two competing
    # runs still hadn't finished after ~50+ CPU-minutes each -- see session
    # notes). Loop-closure registration only needs enough points to
    # constrain a rigid transform, not the full-resolution scan.
    rng = np.random.default_rng(0)

    def window_points(t_center: float, max_window_points: int = 15_000) -> np.ndarray:
        mask = np.abs(local_point_times - t_center) <= window_s
        pts = local_points[mask]
        if len(pts) > max_window_points:
            idx = rng.choice(len(pts), max_window_points, replace=False)
            pts = pts[idx]
        return pts

    observations: list[tuple[float, float, np.ndarray, np.ndarray, float]] = []
    for a, b in pairs:
        ta, tb = float(grid[a]), float(grid[b])
        pts_i = window_points(ta)
        pts_j = window_points(tb)
        if len(pts_i) < min_points or len(pts_j) < min_points:
            continue

        result = _icp_align(
            pts_j, pts_i, init_R=np.eye(3), init_t=np.zeros(3), max_corr_dist=max_corr_dist
        )
        if result is None:
            continue
        R_corr, t_corr, rms, inlier_ratio = result
        if inlier_ratio < 0.3 or rms > max_corr_dist:
            continue

        rotvec = _rotvec_from_matrix(R_corr)
        weight = 2.0 * inlier_ratio
        observations.append((ta, tb, rotvec, t_corr, weight))

    return observations


def _fit_translation_correction(
    observations: list[tuple[float, float, np.ndarray, float]],
    t_start: float,
    t_end: float,
    ctrl_step_s: float = 30.0,
    smooth_weight: float = 0.5,
    anchor_weight: float = 1.0,
    huber_scale_m: float = 0.3,
):
    """Translation twin of _fit_pose_graph_correction: same joint robust
    (Huber) least-squares architecture -- smoothness prior between
    control points, anchor at t_start ~= 0 -- fitting a smooth,
    piecewise-linear WORLD-FRAME position correction curve delta_pos(t)
    from sparse pairwise observations, instead of a rotation-vector one.
    """
    ctrl_t = np.arange(t_start, t_end + ctrl_step_s, ctrl_step_s)
    n_ctrl = len(ctrl_t)
    if n_ctrl < 2 or not observations:
        return np.array([t_start, t_end]), np.zeros((2, 3))

    def bracket(t: float):
        idx = int(np.searchsorted(ctrl_t, t)) - 1
        idx = int(np.clip(idx, 0, n_ctrl - 2))
        t0_, t1_ = ctrl_t[idx], ctrl_t[idx + 1]
        w = 0.0 if t1_ == t0_ else (t - t0_) / (t1_ - t0_)
        return idx, w

    obs_brackets = [(bracket(ti), bracket(tj), v, w) for ti, tj, v, w in observations]

    def residuals(x: np.ndarray) -> np.ndarray:
        delta = x.reshape(n_ctrl, 3)
        rows = []
        for (ai, wi), (aj, wj), v, w in obs_brackets:
            d_i = (1 - wi) * delta[ai] + wi * delta[ai + 1]
            d_j = (1 - wj) * delta[aj] + wj * delta[aj + 1]
            rows.append(w * ((d_j - d_i) - v))
        for k in range(n_ctrl - 1):
            rows.append(smooth_weight * (delta[k + 1] - delta[k]))
        rows.append(anchor_weight * delta[0])
        return np.concatenate(rows)

    x0 = np.zeros(n_ctrl * 3)
    result = least_squares(
        residuals, x0, loss="huber", f_scale=huber_scale_m, max_nfev=20000
    )
    return ctrl_t, result.x.reshape(n_ctrl, 3)


def apply_kiss_icp_loop_closure_correction(
    frame_pose: FramePose,
    local_points: np.ndarray,
    local_point_times: np.ndarray,
    min_observations: int = 3,
) -> FramePose:
    """Collect kiss-icp loop-closure observations and, if there are enough
    to trust a correction, jointly fit and apply a smooth world-frame
    rotation + translation drift correction. Best-effort: returns
    frame_pose unchanged if there aren't enough genuine revisits (min 3)
    to fit a trustworthy correction.
    """
    obs = _collect_kiss_icp_loop_closure_observations(local_points, local_point_times, frame_pose)
    if len(obs) < min_observations:
        return frame_pose

    rot_obs = [(ti, tj, rv, w) for ti, tj, rv, _tc, w in obs]
    trans_obs = [(ti, tj, tc, w) for ti, tj, _rv, tc, w in obs]

    t_start, t_end = float(frame_pose.times[0]), float(frame_pose.times[-1])
    ctrl_t_rot, ctrl_rotvec = _fit_pose_graph_correction(rot_obs, t_start, t_end)
    ctrl_t_pos, ctrl_pos = _fit_translation_correction(trans_obs, t_start, t_end)

    q = np.column_stack([frame_pose.qx, frame_pose.qy, frame_pose.qz, frame_pose.qw])
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    t_clip_rot = np.clip(frame_pose.times, ctrl_t_rot[0], ctrl_t_rot[-1])
    delta_rot = np.column_stack([
        np.interp(t_clip_rot, ctrl_t_rot, ctrl_rotvec[:, 0]),
        np.interp(t_clip_rot, ctrl_t_rot, ctrl_rotvec[:, 1]),
        np.interp(t_clip_rot, ctrl_t_rot, ctrl_rotvec[:, 2]),
    ])
    t_clip_pos = np.clip(frame_pose.times, ctrl_t_pos[0], ctrl_t_pos[-1])
    delta_pos = np.column_stack([
        np.interp(t_clip_pos, ctrl_t_pos, ctrl_pos[:, 0]),
        np.interp(t_clip_pos, ctrl_t_pos, ctrl_pos[:, 1]),
        np.interp(t_clip_pos, ctrl_t_pos, ctrl_pos[:, 2]),
    ])

    pos_orig = np.column_stack([frame_pose.tx, frame_pose.ty, frame_pose.tz])
    pos_new = pos_orig + delta_pos

    new_q = np.zeros_like(q)
    for k in range(len(frame_pose.times)):
        qx_, qy_, qz_, qw_ = q[k]
        n_ = np.sqrt(qx_ * qx_ + qy_ * qy_ + qz_ * qz_ + qw_ * qw_)
        if n_ < 1e-9:
            new_q[k] = q[k]
            continue
        qx_, qy_, qz_, qw_ = qx_ / n_, qy_ / n_, qz_ / n_, qw_ / n_
        x2, y2, z2 = qx_ * qx_, qy_ * qy_, qz_ * qz_
        wx, wy, wz = qw_ * qx_, qw_ * qy_, qw_ * qz_
        xy, xz, yz = qx_ * qy_, qx_ * qz_, qy_ * qz_
        R_orig = np.array([
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ])
        R_corr = _matrix_from_rotvec(delta_rot[k]) @ R_orig
        new_q[k] = _quat_from_matrix(R_corr)

    return FramePose(
        times=frame_pose.times,
        tx=pos_new[:, 0], ty=pos_new[:, 1], tz=pos_new[:, 2],
        qx=new_q[:, 0], qy=new_q[:, 1], qz=new_q[:, 2], qw=new_q[:, 3],
    )
