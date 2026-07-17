"""SHARE S20 scanner-specific processing: bag extraction, camera colorisation.

The S20 backpack is a Livox MID-360 LiDAR + 3 cameras:
  /usb_cam/image_raw/compressed   -- nav camera, 20 Hz, 640x480 pinhole
  /camera_agent/img_left/compressed  -- left fisheye, 0.5 Hz, 3504x4672
  /camera_agent/img_right/compressed -- right fisheye, 0.5 Hz, 3504x4672

Data can be uploaded as:
  - A ZIP of PCD folders (default, bag_lidar_enabled=False)
  - A ZIP containing a .bag file (bag_lidar_enabled=True)
"""

import io
import struct
import zipfile
from dataclasses import dataclass
from pathlib import Path

import laspy
import numpy as np

from pipeline.processing import ProcessingError

# ROS2 CompressedImage topic aliases used to name extracted files.
_CAM_TOPICS: dict[str, str] = {
    "/usb_cam/image_raw/compressed": "nav",
    "/camera_agent/img_left/compressed": "left",
    "/camera_agent/img_right/compressed": "right",
}


# ---------------------------------------------------------------------------
# Frame pose (SLAM body trajectory)
# ---------------------------------------------------------------------------

@dataclass
class FramePose:
    """Interpolatable body-frame trajectory from frame_pose.txt."""
    times: np.ndarray   # float64, seconds
    tx: np.ndarray      # float64, metres
    ty: np.ndarray
    tz: np.ndarray
    qx: np.ndarray      # float64, quaternion components
    qy: np.ndarray
    qz: np.ndarray
    qw: np.ndarray


def read_frame_pose(path: str | Path) -> FramePose:
    """Parse frame_pose.txt: tab/space separated  t x y z qx qy qz qw  per line."""
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


# ---------------------------------------------------------------------------
# Camera frame extraction from bag
# ---------------------------------------------------------------------------

def _parse_compressed_image_jpeg(raw: bytes) -> bytes | None:
    """Extract JPEG bytes from a raw ROS2 CompressedImage CDR message.

    CDR layout (little-endian):
      4 bytes  encapsulation header
      4 bytes  header.stamp.sec
      4 bytes  header.stamp.nanosec
      4 bytes  frame_id string length (N)
      N bytes  frame_id
      4 bytes  format string length (M)
      M bytes  format (e.g. "jpeg")
      4 bytes  data byte array length (L)
      L bytes  JPEG data
    """
    try:
        off = 4  # skip encapsulation header
        off += 4 + 4  # stamp sec + nanosec
        fid_len = struct.unpack_from("<I", raw, off)[0]
        off += 4 + fid_len
        # align to 4 bytes
        if off % 4:
            off += 4 - (off % 4)
        fmt_len = struct.unpack_from("<I", raw, off)[0]
        off += 4 + fmt_len
        if off % 4:
            off += 4 - (off % 4)
        data_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        return raw[off: off + data_len]
    except Exception:
        return None


def extract_camera_frames_from_bag(bag_zip_or_bag_path: str | Path, out_dir: str | Path) -> int:
    """Extract compressed camera frames from a .bag file (inside a ZIP or directly).

    Writes  {alias}_{ts_ns:019d}.jpg  files into out_dir.
    Returns total number of frames written.
    """
    bag_zip_or_bag_path = Path(bag_zip_or_bag_path)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Find the .bag file
    if bag_zip_or_bag_path.suffix.lower() == ".zip":
        bag_path = _extract_bag_from_zip(bag_zip_or_bag_path, out_dir)
    else:
        bag_path = bag_zip_or_bag_path

    if bag_path is None or not bag_path.exists():
        return 0

    return _extract_frames_from_bag_file(bag_path, out_dir)


def _extract_bag_from_zip(zip_path: Path, out_dir: Path) -> Path | None:
    with zipfile.ZipFile(zip_path) as zf:
        for name in zf.namelist():
            if name.endswith(".bag") and not name.startswith("__MACOSX"):
                target = out_dir / Path(name).name
                with zf.open(name) as src, open(target, "wb") as dst:
                    while chunk := src.read(8 * 1024 * 1024):
                        dst.write(chunk)
                return target
    return None


def _extract_frames_from_bag_file(bag_path: Path, out_dir: Path) -> int:
    """Walk a ROS2 bag SQLite3 database and extract CompressedImage messages."""
    import sqlite3

    db_file = bag_path
    if bag_path.is_dir():
        # ROS2 bag is a directory; find the .db3 inside
        dbs = list(bag_path.rglob("*.db3"))
        if not dbs:
            return 0
        db_file = dbs[0]

    n_written = 0
    topic_map: dict[int, str] = {}  # topic_id -> alias

    with sqlite3.connect(str(db_file)) as conn:
        cur = conn.cursor()
        try:
            cur.execute("SELECT id, name FROM topics")
        except sqlite3.OperationalError:
            return 0
        for tid, name in cur.fetchall():
            if name in _CAM_TOPICS:
                topic_map[tid] = _CAM_TOPICS[name]

        if not topic_map:
            return 0

        placeholders = ",".join("?" * len(topic_map))
        cur.execute(
            f"SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN ({placeholders}) ORDER BY timestamp",
            list(topic_map.keys()),
        )
        for topic_id, ts_ns, data in cur.fetchall():
            alias = topic_map[topic_id]
            if isinstance(data, str):
                data = data.encode("latin-1")
            jpeg = _parse_compressed_image_jpeg(bytes(data))
            if jpeg and len(jpeg) > 100:
                fname = out_dir / f"{alias}_{ts_ns:019d}.jpg"
                fname.write_bytes(jpeg)
                n_written += 1

    return n_written


# ---------------------------------------------------------------------------
# LiDAR extraction from bag
# ---------------------------------------------------------------------------

def bag_lidar_to_laz(bag_zip_or_bag_path: str | Path, out_laz: str | Path) -> int:
    """Extract LiDAR point cloud from a ROS2 bag and write LAZ.

    Reads /livox/lidar (or /livox/lidar_node) PointCloud2 messages,
    applies per-point timestamp for motion compensation.
    Returns number of points written.
    """
    import sqlite3

    bag_zip_or_bag_path = Path(bag_zip_or_bag_path)
    out_laz = Path(out_laz)
    tmp_dir = out_laz.parent / "_bag_extract"
    tmp_dir.mkdir(exist_ok=True)

    if bag_zip_or_bag_path.suffix.lower() == ".zip":
        bag_path = _extract_bag_from_zip(bag_zip_or_bag_path, tmp_dir)
    else:
        bag_path = bag_zip_or_bag_path

    if bag_path is None or not bag_path.exists():
        raise ProcessingError(f"No .bag file found in {bag_zip_or_bag_path}")

    db_file = bag_path
    if bag_path.is_dir():
        dbs = list(bag_path.rglob("*.db3"))
        if not dbs:
            raise ProcessingError(f"No .db3 in bag directory {bag_path}")
        db_file = dbs[0]

    lidar_topics = ["/livox/lidar", "/livox/lidar_node", "/points"]
    topic_id: int | None = None
    point_step = 0
    fields: list[dict] = []

    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    zs: list[np.ndarray] = []
    ts_list: list[np.ndarray] = []
    intensities: list[np.ndarray] = []

    with sqlite3.connect(str(db_file)) as conn:
        cur = conn.cursor()
        cur.execute("SELECT id, name FROM topics")
        topic_rows = cur.fetchall()

        for tid, name in topic_rows:
            if name in lidar_topics:
                topic_id = tid
                break

        if topic_id is None:
            raise ProcessingError(
                f"No LiDAR topic found in bag. Topics: {[r[1] for r in topic_rows]}"
            )

        cur.execute(
            "SELECT data FROM messages WHERE topic_id = ? LIMIT 1", (topic_id,)
        )
        row = cur.fetchone()
        if row is None:
            raise ProcessingError("LiDAR topic has no messages")

        # Parse the first PointCloud2 to get field layout
        data0 = bytes(row[0]) if not isinstance(row[0], bytes) else row[0]
        fields, point_step = _parse_pc2_header(data0)

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,),
        )
        for ts_ns, raw in cur.fetchall():
            raw = bytes(raw) if not isinstance(raw, bytes) else raw
            x, y, z, t, intensity = _decode_pc2_points(raw, fields, point_step, float(ts_ns) / 1e9)
            if len(x):
                xs.append(x)
                ys.append(y)
                zs.append(z)
                ts_list.append(t)
                intensities.append(intensity)

    if not xs:
        raise ProcessingError("No LiDAR points extracted from bag")

    all_x = np.concatenate(xs)
    all_y = np.concatenate(ys)
    all_z = np.concatenate(zs)
    all_t = np.concatenate(ts_list)
    all_i = np.concatenate(intensities)

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


def _parse_pc2_header(raw: bytes) -> tuple[list[dict], int]:
    """Parse PointCloud2 CDR header, return (fields, point_step)."""
    off = 4  # encapsulation
    off += 4 + 4  # stamp
    fid_len = struct.unpack_from("<I", raw, off)[0]
    off += 4 + fid_len
    if off % 4:
        off += 4 - (off % 4)
    # height, width
    off += 8
    # fields array
    n_fields = struct.unpack_from("<I", raw, off)[0]
    off += 4
    fields = []
    for _ in range(n_fields):
        name_len = struct.unpack_from("<I", raw, off)[0]
        off += 4
        name = raw[off: off + name_len].decode("utf-8", errors="replace").rstrip("\x00")
        off += name_len
        if off % 4:
            off += 4 - (off % 4)
        field_offset, datatype, count = struct.unpack_from("<IBI", raw, off)
        off += 9
        if off % 4:
            off += 4 - (off % 4)
        fields.append({"name": name, "offset": field_offset, "datatype": datatype, "count": count})
    is_bigendian = struct.unpack_from("B", raw, off)[0]
    off += 1
    point_step, row_step = struct.unpack_from("<II", raw, off)
    off += 8
    return fields, point_step


_PC2_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def _decode_pc2_points(
    raw: bytes, fields: list[dict], point_step: int, msg_time: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Decode a PointCloud2 message body into x,y,z,time,intensity arrays."""
    # Find data start (after header)
    # Scan for the data length field at the known location
    # Simpler: just try to find the point data by walking the CDR
    try:
        # Skip CDR header bytes to find data array start
        # We locate it by searching from the end of the message header
        # For robustness, try both: using numpy structured array or byte-by-byte
        field_map = {f["name"]: f for f in fields}

        # Find data array (last uint32 before the actual point data)
        # The data starts after all header fields; we look for the data_length marker
        # by trying offsets until we get reasonable XYZ values
        for data_start in _find_data_start(raw, point_step, field_map):
            break
        else:
            return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)

        n_pts = (len(raw) - data_start) // point_step
        if n_pts <= 0:
            return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)

        buf = np.frombuffer(raw[data_start: data_start + n_pts * point_step], dtype=np.uint8)
        buf = buf.reshape(n_pts, point_step)

        def _extract(name: str, default: np.ndarray) -> np.ndarray:
            if name not in field_map:
                return default
            f = field_map[name]
            dt = np.dtype("<" + _PC2_DTYPES.get(f["datatype"], "f4"))
            nbytes = dt.itemsize
            col = buf[:, f["offset"]: f["offset"] + nbytes]
            return col.copy().view(dt).reshape(-1).astype(np.float64)

        x = _extract("x", np.zeros(n_pts))
        y = _extract("y", np.zeros(n_pts))
        z = _extract("z", np.zeros(n_pts))
        intensity = _extract("intensity", np.ones(n_pts))
        # Livox per-point timestamp (offset_time in ns from msg stamp)
        t_offset = _extract("offset_time", np.zeros(n_pts))
        t = msg_time + t_offset / 1e9

        # Filter invalid points
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        return x[valid], y[valid], z[valid], t[valid], intensity[valid]
    except Exception:
        return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)


def _find_data_start(raw: bytes, point_step: int, field_map: dict):
    """Yield a candidate data_start offset by scanning for the data length marker."""
    # The data array in CDR is preceded by a uint32 length (number of bytes).
    # We search backwards from the end of the buffer.
    for off in range(max(0, len(raw) - point_step * 4), 4, -4):
        try:
            n = struct.unpack_from("<I", raw, off - 4)[0]
            if n > 0 and n % point_step == 0 and off + n <= len(raw):
                yield off
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Camera colorisation
# ---------------------------------------------------------------------------

def _load_calibration_camera(cal_path: str | Path | None):
    """Load usb_cam / nav camera intrinsics from calibration.yaml.
    Returns (fx, fy, cx, cy, R_cam_in_body, t_cam_in_body) or None.
    """
    if cal_path is None or not Path(cal_path).exists():
        return None
    try:
        import yaml
        with open(cal_path, encoding="utf-8") as f:
            d = yaml.safe_load(f)
        if not d:
            return None
        cam = None
        for key in ("usb_cam", "nav_cam", "camera", "cam0"):
            if key in d:
                cam = d[key]
                break
        if cam is None and "camera_matrix" in d:
            cam = d
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
        return (fx, fy, cx, cy, np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    except Exception:
        return None


def colorize_laz(
    laz_path: str | Path,
    cam_zip_path: str | Path,
    frame_pose: FramePose,
    out_laz: str | Path,
    calibration_path: str | Path | None = None,
) -> int:
    """Colorize a SLAM-frame LAZ by projecting nav-cam frames onto LiDAR points.

    laz_path: input LAZ in SLAM body-frame (gps_time per point).
    cam_zip_path: camera_frames.zip with nav_*.jpg (Unix ns timestamp in name).
    frame_pose: FramePose with body poses over time.
    out_laz: output LAS-7 file with RGB channels.
    calibration_path: optional calibration.yaml with usb_cam intrinsics.

    Returns number of points coloured.
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
    R_bic = R_cam_in_body.T  # rotation body-in-camera

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
            nr = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
            if nr < 1e-9:
                continue
            qx /= nr; qy /= nr; qz /= nr; qw /= nr

            R_bw = np.array([
                [1 - 2*(qy*qy + qz*qz),  2*(qx*qy - qw*qz),  2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz),  1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy),  2*(qy*qz + qw*qx),  1 - 2*(qx*qx + qy*qy)],
            ], dtype=np.float64)
            t_bw = np.array([tx, ty, tz], dtype=np.float64)

            pts_w = np.column_stack([x_arr[mask], y_arr[mask], z_arr[mask]])
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