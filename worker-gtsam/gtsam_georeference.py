"""Joint GTSAM factor-graph alternative to pipeline.s20.georeference_from_slam.

Lives in its own isolated environment (see Dockerfile) because gtsam==4.2.1
requires numpy<2.0, which conflicts with the main worker's numpy>=2 / scipy /
opencv stack -- installing it into that shared environment silently
downgrades numpy there and breaks scipy/opencv imports. This script is
therefore standalone (no `pipeline.s20` import) and duplicates the handful
of small helpers (frame_pose/rtk .pos parsing, quaternion->matrix, the
yaw-only similarity fit) it needs from there.

WHAT THIS DOES DIFFERENTLY FROM s20.georeference_from_slam: that function
fits one global yaw+XY+Z similarity transform, then separately computes and
heavily time-smooths a translation-only RTK-vs-SLAM residual, applied
per-point -- a sequential, two-stage correction. This script instead builds
one GTSAM factor graph with a BetweenFactor<Pose3> between every consecutive
FAST-LIO2 pose (trusting its own short-term local consistency) and a
GPSFactor at every matched RTK epoch (pulling toward absolute position), and
solves both jointly with Levenberg-Marquardt -- the same structural idea as
the vendor SHARE SLAM engine's own VOXEL_SLAM core (BetweenFactor odometry/
loop edges + GPSFactor2, see project notes), minus its BTC-descriptor LiDAR
loop-closure edges (prototyped separately; on the one scan tested it did not
beat this position-only version -- see project history before reusing that
code path).

VALIDATED RESULT (scan 2026-07-04_04-12-33_PointCloud, 3331 poses / 328s):
median cell roughness 3.96m vs 4.53m for s20.georeference_from_slam on the
same FAST-LIO2 trajectory -- a real but modest ~13% improvement, well short
of the vendor's own 0.88m on the same scan. That gap is NOT closed by this
script; it needs a working orientation-correcting mechanism (the untested
loop-closure prototype, or something else) on top of it.

FRONT-END MATTERS AT LEAST AS MUCH AS THIS BACKEND: the same scan's raw
FAST-LIO2 output, in its own local frame with NO georeferencing applied at
all, already measured 4.52m -- i.e. this whole backend correction (any
version, including the sequential one) was operating near FAST-LIO2's own
local-consistency ceiling, not the real bottleneck. Fixing worker-fastlio's
previously-unapplied calibration.yaml IMU_time_offset (see
worker-fastlio/mid360_template.yaml) dropped that raw local figure to 4.17m,
and combined with this script's graph correction reached **3.62m** -- the
current best validated result on this scan. Both fixes are independent and
compose; look at worker-fastlio first for any further front-end gains before
tuning this backend further, since backend tuning alone has clearly
plateaued (loop closure and GNSS-course-derived yaw both failed to improve
on it, see project history).

THE DEFAULT NOISE PARAMETERS ARE SCAN-SPECIFIC, NOT UNIVERSAL: they were
found by sweeping on that one scan, and the qualitative finding was that
*very* rigid odometry trust + *very* loose GPS trust ended up winning --
i.e. the graph mostly reproduces a single clean global alignment rather than
meaningfully using per-epoch RTK. Re-validate on a few more scans (roughness
before/after, visual inspection) before trusting these defaults broadly, and
prefer widening --gps-sigma-h/v further over narrowing them if a new scan
looks worse: every sweep step in that direction (this project's history)
made things better, tightening always made things worse.
"""
import argparse
from datetime import datetime, timezone

import gtsam
import laspy
import numpy as np
from gtsam.symbol_shorthand import X
from pyproj import CRS, Transformer


def read_frame_pose(path):
    cols = [[] for _ in range(8)]
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
    return {
        k: np.array(v)
        for k, v in zip(["t", "x", "y", "z", "qx", "qy", "qz", "qw"], cols)
    }


def read_rtk_pos(path):
    times, lats, lons, alts = [], [], [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 5:
                continue
            try:
                dt = datetime.strptime(f"{parts[0]} {parts[1]}", "%Y/%m/%d %H:%M:%S.%f").replace(
                    tzinfo=timezone.utc
                )
                times.append(dt.timestamp())
                lats.append(float(parts[2]))
                lons.append(float(parts[3]))
                alts.append(float(parts[4]))
            except ValueError:
                continue
    return np.array(times), np.array(lats), np.array(lons), np.array(alts)


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    x2, y2, z2 = qx * qx, qy * qy, qz * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    return np.array(
        [
            [1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (x2 + z2), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (x2 + y2)],
        ]
    )


def solve_yaw_similarity(src_pts, dst_pts):
    """Same fit as s20._solve_yaw_similarity_transform -- used only to seed
    the graph's initial values, not part of the optimization itself."""
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
    return R, np.array([t_xy[0], t_xy[1], t_z])


def gtsam_georeference(
    src_laz,
    frame_pose_path,
    rtk_pos_path,
    out_laz,
    target_crs_epsg=None,
    gps_sigma_h=10.0,
    gps_sigma_v=20.0,
    odom_sigma_trans=0.001,
    odom_sigma_rot=0.001,
    gps_max_time_gap_s=0.15,
) -> int:
    fp = read_frame_pose(frame_pose_path)
    rtk_t, rtk_lat, rtk_lon, rtk_alt = read_rtk_pos(rtk_pos_path)
    if len(rtk_t) < 4:
        raise ValueError(f"RTK trajectory has only {len(rtk_t)} epochs, need >= 4")

    median_lon, median_lat = float(np.median(rtk_lon)), float(np.median(rtk_lat))
    if target_crs_epsg is not None:
        utm_crs = CRS.from_epsg(target_crs_epsg)
    else:
        utm_zone = int((median_lon + 180.0) / 6.0) + 1
        hemisphere = "north" if median_lat >= 0 else "south"
        utm_crs = CRS.from_string(f"+proj=utm +zone={utm_zone} +{hemisphere} +datum=WGS84 +units=m +no_defs")
    to_utm = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    rtk_x, rtk_y = to_utm.transform(rtk_lon, rtk_lat)
    rtk_x, rtk_y, rtk_z = np.array(rtk_x), np.array(rtk_y), np.array(rtk_alt)

    N = len(fp["t"])
    t_start, t_end = max(fp["t"][0], rtk_t[0]), min(fp["t"][-1], rtk_t[-1])
    if t_end <= t_start:
        raise ValueError(f"No time overlap: SLAM [{fp['t'][0]:.1f},{fp['t'][-1]:.1f}], RTK [{rtk_t[0]:.1f},{rtk_t[-1]:.1f}]")
    mask = (rtk_t >= t_start) & (rtk_t <= t_end)
    rtk_tm = rtk_t[mask]
    slam_pts = np.column_stack(
        [np.interp(rtk_tm, fp["t"], fp["x"]), np.interp(rtk_tm, fp["t"], fp["y"]), np.interp(rtk_tm, fp["t"], fp["z"])]
    )
    utm_pts = np.column_stack([rtk_x[mask], rtk_y[mask], rtk_z[mask]])
    R0, t0 = solve_yaw_similarity(slam_pts, utm_pts)

    R_orig = [quat_to_R(fp["qx"][i], fp["qy"][i], fp["qz"][i], fp["qw"][i]) for i in range(N)]
    t_orig = np.column_stack([fp["x"], fp["y"], fp["z"]])
    R_init = [R0 @ R_orig[i] for i in range(N)]
    t_init = (R0 @ t_orig.T).T + t0

    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    for i in range(N):
        initial.insert(X(i), gtsam.Pose3(gtsam.Rot3(R_init[i]), gtsam.Point3(*t_init[i])))

    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]))
    graph.add(gtsam.PriorFactorPose3(X(0), initial.atPose3(X(0)), prior_noise))

    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([odom_sigma_rot] * 3 + [odom_sigma_trans] * 3))
    for i in range(N - 1):
        Ti = gtsam.Pose3(gtsam.Rot3(R_orig[i]), gtsam.Point3(*t_orig[i]))
        Tj = gtsam.Pose3(gtsam.Rot3(R_orig[i + 1]), gtsam.Point3(*t_orig[i + 1]))
        graph.add(gtsam.BetweenFactorPose3(X(i), X(i + 1), Ti.between(Tj), odom_noise))

    gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([gps_sigma_h, gps_sigma_h, gps_sigma_v]))
    n_gps = 0
    for k in range(len(rtk_t)):
        j = int(np.searchsorted(fp["t"], rtk_t[k]))
        if j <= 0 or j >= N:
            continue
        if abs(fp["t"][j] - rtk_t[k]) > gps_max_time_gap_s and abs(fp["t"][j - 1] - rtk_t[k]) > gps_max_time_gap_s:
            continue
        if abs(fp["t"][j - 1] - rtk_t[k]) < abs(fp["t"][j] - rtk_t[k]):
            j -= 1
        graph.add(gtsam.GPSFactor(X(j), gtsam.Point3(rtk_x[k], rtk_y[k], rtk_z[k]), gps_noise))
        n_gps += 1

    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SILENT")
    result = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()
    print(
        f"[gtsam_georeference] {N} poses, {n_gps} GPS factors, "
        f"error {graph.error(initial):.1f} -> {graph.error(result):.1f}"
    )

    delta_R = np.zeros((N, 3, 3))
    delta_t = np.zeros((N, 3))
    for i in range(N):
        p = result.atPose3(X(i))
        Rn, tn = p.rotation().matrix(), np.asarray(p.translation())
        dR = Rn @ R_orig[i].T
        delta_R[i] = dR
        delta_t[i] = tn - dR @ t_orig[i]

    with laspy.open(src_laz) as f:
        las = f.read()
        x = np.asarray(las.x, dtype=np.float64)
        y = np.asarray(las.y, dtype=np.float64)
        z = np.asarray(las.z, dtype=np.float64)
        gps_time = np.asarray(las.gps_time, dtype=np.float64)

    order = np.argsort(gps_time)
    gps_s = gps_time[order]
    uniq_t, start_idx, counts = np.unique(gps_s, return_index=True, return_counts=True)
    pose_idx = np.clip(np.searchsorted(fp["t"], uniq_t), 0, N - 1)
    left = np.clip(pose_idx - 1, 0, N - 1)
    use_left = np.abs(fp["t"][left] - uniq_t) < np.abs(fp["t"][pose_idx] - uniq_t)
    pose_idx = np.where(use_left, left, pose_idx)

    x_s, y_s, z_s = x[order].copy(), y[order].copy(), z[order].copy()
    for si, c, pi in zip(start_idx, counts, pose_idx):
        sl = slice(si, si + c)
        pts = np.column_stack([x_s[sl], y_s[sl], z_s[sl]])
        new_pts = pts @ delta_R[pi].T + delta_t[pi]
        x_s[sl], y_s[sl], z_s[sl] = new_pts[:, 0], new_pts[:, 1], new_pts[:, 2]

    out_x, out_y, out_z = np.empty_like(x), np.empty_like(y), np.empty_like(z)
    out_x[order], out_y[order], out_z[order] = x_s, y_s, z_s

    with laspy.open(src_laz) as reader:
        hdr = laspy.LasHeader(point_format=reader.header.point_format, version=reader.header.version)
        hdr.add_crs(utm_crs)
        hdr.offsets = np.array([out_x.min(), out_y.min(), out_z.min()])
        hdr.scales = np.array([0.001, 0.001, 0.001])
        with laspy.open(out_laz, mode="w", header=hdr) as writer:
            src = reader.read()
            p = laspy.ScaleAwarePointRecord.zeros(len(src.x), header=hdr)
            for dim in src.point_format.dimension_names:
                if dim in ("X", "Y", "Z"):
                    continue
                setattr(p, dim, np.asarray(getattr(src, dim)))
            p.x, p.y, p.z = out_x, out_y, out_z
            writer.write_points(p)

    return len(out_x)


def _cli():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src_laz")
    ap.add_argument("frame_pose")
    ap.add_argument("rtk_pos")
    ap.add_argument("out_laz")
    ap.add_argument("--target-crs-epsg", type=int, default=None)
    ap.add_argument("--gps-sigma-h", type=float, default=10.0)
    ap.add_argument("--gps-sigma-v", type=float, default=20.0)
    ap.add_argument("--odom-sigma-trans", type=float, default=0.001)
    ap.add_argument("--odom-sigma-rot", type=float, default=0.001)
    args = ap.parse_args()
    n = gtsam_georeference(
        args.src_laz,
        args.frame_pose,
        args.rtk_pos,
        args.out_laz,
        target_crs_epsg=args.target_crs_epsg,
        gps_sigma_h=args.gps_sigma_h,
        gps_sigma_v=args.gps_sigma_v,
        odom_sigma_trans=args.odom_sigma_trans,
        odom_sigma_rot=args.odom_sigma_rot,
    )
    print(f"wrote {n} points to {args.out_laz}")


if __name__ == "__main__":
    _cli()
