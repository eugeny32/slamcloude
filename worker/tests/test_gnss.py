"""PPK GNSS core tests: .pos parsing and applying trajectory corrections
to a point cloud with known, verifiable shifts. No RTKLIB/S3/DB needed."""

from datetime import UTC, datetime
from pathlib import Path

import laspy
import numpy as np
import pytest
from pyproj import CRS, Transformer

from pipeline import gnss

UTM_EPSG = 32637
CX, CY, CZ = 410_000.0, 6_170_000.0, 150.0
T0, T1 = 100_000.0, 100_010.0  # trajectory time window (POSIX seconds)

# Known correction the "PPK solution" applies to the original trajectory.
SHIFT = np.array([0.5, -0.3, 0.2])

_TO_WGS = Transformer.from_crs(CRS.from_epsg(UTM_EPSG), "EPSG:4326", always_xy=True)


def _pos_text(times: np.ndarray, utm_xyz: np.ndarray, quality: int = 1) -> str:
    lines = ["% synthetic RTKLIB-style pos file"]
    lons, lats = _TO_WGS.transform(utm_xyz[:, 0], utm_xyz[:, 1])
    for t, lat, lon, h in zip(times, lats, lons, utm_xyz[:, 2], strict=True):
        stamp = datetime.fromtimestamp(t, UTC).strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
        lines.append(f"{stamp}  {lat:.9f}  {lon:.9f}  {h:.4f}  {quality}  14")
    return "\n".join(lines) + "\n"


@pytest.fixture
def trajectories(tmp_path: Path) -> tuple[Path, Path]:
    """(original.pos, corrected.pos): straight path, constant known shift."""
    times = np.linspace(T0, T1, 11)
    xyz = np.column_stack(
        (
            np.linspace(CX, CX + 10, 11),
            np.full(11, CY),
            np.full(11, CZ),
        )
    )
    original = tmp_path / "original.pos"
    corrected = tmp_path / "corrected.pos"
    original.write_text(_pos_text(times, xyz, quality=2))
    corrected.write_text(_pos_text(times, xyz + SHIFT, quality=1))
    return original, corrected


@pytest.fixture
def las_with_gps_time(tmp_path: Path) -> Path:
    rng = np.random.default_rng(3)
    n = 400
    pts = rng.normal(0.0, 2.0, size=(n, 3)) + (CX + 5, CY, CZ)
    path = tmp_path / "cloud.las"
    header = laspy.LasHeader(version="1.4", point_format=3)  # pf3 has gps_time
    header.scales = (0.001, 0.001, 0.001)
    header.offsets = (CX, CY, CZ)
    header.add_crs(CRS.from_epsg(UTM_EPSG))
    las = laspy.LasData(header)
    las.x, las.y, las.z = pts[:, 0], pts[:, 1], pts[:, 2]
    las.gps_time = rng.uniform(T0, T1, size=n)
    las.write(str(path))
    return path


class TestParsePos:
    def test_roundtrip(self, trajectories: tuple[Path, Path]) -> None:
        traj = gnss.parse_pos(trajectories[0])
        assert len(traj) == 11
        assert np.all(np.diff(traj.times) > 0)
        assert traj.times[0] == pytest.approx(T0, abs=0.001)
        assert 50 < traj.lats[0] < 60 and 30 < traj.lons[0] < 45  # UTM 37N

    def test_fixed_ratio(self, trajectories: tuple[Path, Path]) -> None:
        assert gnss.fixed_ratio(gnss.parse_pos(trajectories[0])) == 0.0  # all float
        assert gnss.fixed_ratio(gnss.parse_pos(trajectories[1])) == 1.0  # all fixed

    def test_empty_rejected(self, tmp_path: Path) -> None:
        p = tmp_path / "empty.pos"
        p.write_text("% only comments\n")
        with pytest.raises(gnss.TrajectoryError, match="no trajectory"):
            gnss.parse_pos(p)

    def test_garbage_line_rejected(self, tmp_path: Path) -> None:
        p = tmp_path / "bad.pos"
        p.write_text("2026/07/16 10:00:00.000 not-a-number 37.0 100.0 1 5\n")
        with pytest.raises(gnss.TrajectoryError, match="line 1"):
            gnss.parse_pos(p)


class TestApplyCorrection:
    def test_points_shift_by_trajectory_delta(
        self, las_with_gps_time: Path, trajectories: tuple[Path, Path], tmp_path: Path
    ) -> None:
        original = gnss.parse_pos(trajectories[0])
        corrected = gnss.parse_pos(trajectories[1])
        dst = tmp_path / "corrected_cloud.laz"

        result = gnss.apply_trajectory_correction(
            las_with_gps_time, dst, original=original, corrected=corrected
        )

        expected_shift = float(np.linalg.norm(SHIFT))
        assert result.points == 400
        # Constant shift: mean == max == |SHIFT| (mm tolerance: scale +
        # lat/lon roundtrip through 9-decimal pos formatting).
        assert result.mean_shift_m == pytest.approx(expected_shift, abs=0.005)
        assert result.max_shift_m == pytest.approx(expected_shift, abs=0.005)

        with laspy.open(las_with_gps_time) as before, laspy.open(dst) as after:
            src = before.read()
            out = after.read()
        dx = np.asarray(out.x) - np.asarray(src.x)
        dy = np.asarray(out.y) - np.asarray(src.y)
        dz = np.asarray(out.z) - np.asarray(src.z)
        assert np.allclose(dx, SHIFT[0], atol=0.005)
        assert np.allclose(dy, SHIFT[1], atol=0.005)
        assert np.allclose(dz, SHIFT[2], atol=0.005)

    def test_cloud_without_crs_rejected(
        self, trajectories: tuple[Path, Path], tmp_path: Path
    ) -> None:
        path = tmp_path / "nocrs.las"
        header = laspy.LasHeader(version="1.4", point_format=3)
        las = laspy.LasData(header)
        las.x, las.y, las.z = [0.0], [0.0], [0.0]
        las.write(str(path))
        with pytest.raises(gnss.TrajectoryError, match="no CRS"):
            gnss.apply_trajectory_correction(
                path,
                tmp_path / "out.laz",
                original=gnss.parse_pos(trajectories[0]),
                corrected=gnss.parse_pos(trajectories[1]),
            )


def test_math_note_pdal_style_gate() -> None:
    # rnx2rtkp is exercised in the worker image / e2e; locally just the gate.
    assert isinstance(gnss.rnx2rtkp_available(), bool)
