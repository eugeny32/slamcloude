"""PPK GNSS post-processing: trajectory parsing, RTKLIB solving, and applying
the trajectory correction to a point cloud.

The PPK solution itself (carrier-phase, ambiguity resolution) is delegated to
RTKLIB's rnx2rtkp (present in the worker Docker image). Everything else —
.pos parsing, per-epoch offset computation, per-point application via
gps_time — is pure numpy/pyproj and unit-tested locally.

Time systems note: point gps_time and trajectory epochs must share one time
base. Synthetic tests guarantee it; for real SHARE S20 data the decode step
is responsible for normalizing LAS gps_time to the trajectory's GPST epoch.
"""

import copy
import shutil
import subprocess
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import laspy
import numpy as np
from pyproj import Transformer

from pipeline.processing import ProcessingError

# RTKLIB .pos solution quality flags.
Q_FIXED = 1
Q_FLOAT = 2


class TrajectoryError(ProcessingError):
    pass


@dataclass
class Trajectory:
    times: np.ndarray  # POSIX seconds (GPST parsed as UTC-naive)
    lats: np.ndarray  # degrees
    lons: np.ndarray  # degrees
    heights: np.ndarray  # meters (ellipsoidal)
    quality: np.ndarray  # RTKLIB Q flags

    def __len__(self) -> int:
        return len(self.times)


def parse_pos(path: str | Path) -> Trajectory:
    """Parse an RTKLIB-style .pos file: '%'-comments, then
    'YYYY/MM/DD HH:MM:SS.fff  lat  lon  height  Q  ns ...' rows."""
    times: list[float] = []
    lats: list[float] = []
    lons: list[float] = []
    heights: list[float] = []
    quality: list[int] = []

    with open(path, encoding="utf-8", errors="replace") as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith("%") or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 6:
                raise TrajectoryError(f"{path}: line {lineno}: expected >= 6 columns")
            try:
                stamp = datetime.strptime(
                    f"{parts[0]} {parts[1]}", "%Y/%m/%d %H:%M:%S.%f"
                ).replace(tzinfo=UTC)
                times.append(stamp.timestamp())
                lats.append(float(parts[2]))
                lons.append(float(parts[3]))
                heights.append(float(parts[4]))
                quality.append(int(parts[5]))
            except ValueError as exc:
                raise TrajectoryError(f"{path}: line {lineno}: {exc}") from exc

    if not times:
        raise TrajectoryError(f"{path}: no trajectory epochs found")

    order = np.argsort(times)
    return Trajectory(
        times=np.asarray(times)[order],
        lats=np.asarray(lats)[order],
        lons=np.asarray(lons)[order],
        heights=np.asarray(heights)[order],
        quality=np.asarray(quality, dtype=int)[order],
    )


def fixed_ratio(traj: Trajectory) -> float:
    """Fraction of epochs with a fixed (Q=1) solution."""
    return float(np.mean(traj.quality == Q_FIXED))


def rnx2rtkp_available() -> bool:
    return shutil.which("rnx2rtkp") is not None


def run_rnx2rtkp(
    rover_obs: str | Path,
    base_obs: str | Path,
    out_pos: str | Path,
    nav: str | Path | None = None,
    timeout: int = 3600,
) -> None:
    """PPK (kinematic) solution: rover raw obs corrected by base station obs."""
    cmd = ["rnx2rtkp", "-p", "2", "-o", str(out_pos), str(rover_obs), str(base_obs)]
    if nav is not None:
        cmd.append(str(nav))
    result = subprocess.run(cmd, capture_output=True, timeout=timeout)
    out = Path(out_pos)
    if result.returncode != 0 or not out.exists() or out.stat().st_size == 0:
        raise ProcessingError(
            f"rnx2rtkp failed (rc={result.returncode}): "
            f"{result.stderr.decode(errors='replace')[:2000]}"
        )


@dataclass
class CorrectionResult:
    points: int
    mean_shift_m: float
    max_shift_m: float


def compute_offsets(
    original: Trajectory, corrected: Trajectory, transformer: Transformer
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Per-epoch (dx, dy, dz) in the projected CRS at the original epochs.

    The corrected trajectory is time-interpolated onto the original's epochs,
    so differing solution rates are fine.
    """
    ox, oy = transformer.transform(original.lons, original.lats)
    cx, cy = transformer.transform(corrected.lons, corrected.lats)
    cxi = np.interp(original.times, corrected.times, cx)
    cyi = np.interp(original.times, corrected.times, cy)
    chi = np.interp(original.times, corrected.times, corrected.heights)
    return (
        original.times,
        cxi - np.asarray(ox),
        cyi - np.asarray(oy),
        chi - original.heights,
    )


def apply_trajectory_correction(
    src: str | Path,
    dst: str | Path,
    original: Trajectory,
    corrected: Trajectory,
) -> CorrectionResult:
    """Shift every point by the trajectory correction interpolated at the
    point's gps_time. Chunked — constant memory for arbitrarily large clouds."""
    with laspy.open(src) as reader:
        crs = reader.header.parse_crs()
        if crs is None:
            raise TrajectoryError(
                "point cloud carries no CRS; cannot apply trajectory correction"
            )
        if "gps_time" not in reader.header.point_format.dimension_names:
            raise TrajectoryError(
                "point format has no gps_time; cannot map points onto the trajectory"
            )
        transformer = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
        times, dx, dy, dz = compute_offsets(original, corrected, transformer)

        points = 0
        sum_shift = 0.0
        max_shift = 0.0
        header = copy.deepcopy(reader.header)
        with laspy.open(dst, mode="w", header=header) as writer:
            from pipeline.processing import CHUNK_POINTS

            for chunk in reader.chunk_iterator(CHUNK_POINTS):
                t = np.asarray(chunk.gps_time)
                sx = np.interp(t, times, dx)
                sy = np.interp(t, times, dy)
                sz = np.interp(t, times, dz)
                chunk.x = np.asarray(chunk.x) + sx
                chunk.y = np.asarray(chunk.y) + sy
                chunk.z = np.asarray(chunk.z) + sz
                writer.write_points(chunk)

                shift = np.sqrt(sx**2 + sy**2 + sz**2)
                points += len(t)
                sum_shift += float(shift.sum())
                max_shift = max(max_shift, float(shift.max(initial=0.0)))

    return CorrectionResult(
        points=points,
        mean_shift_m=(sum_shift / points) if points else 0.0,
        max_shift_m=max_shift,
    )
