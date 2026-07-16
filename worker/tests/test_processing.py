"""Processing core tests on real (synthetic) LAS data — run locally, no S3/DB."""

import hashlib
from pathlib import Path

import laspy
import numpy as np
import pytest
from pyproj import CRS, Transformer

from pipeline import processing

UTM_EPSG = 32637  # UTM 37N (Moscow area)
CLUSTER_CENTER = (410_000.0, 6_170_000.0, 150.0)
NUM_CLUSTER_POINTS = 500
NUM_OUTLIERS = 5


def _make_las(path: Path, *, with_crs: bool = True, with_outliers: bool = True) -> None:
    rng = np.random.default_rng(42)
    cx, cy, cz = CLUSTER_CENTER
    pts = rng.normal(0.0, 2.0, size=(NUM_CLUSTER_POINTS, 3)) + (cx, cy, cz)
    if with_outliers:
        # Points hundreds of meters away from a 2 m-sigma cluster.
        outliers = np.array(
            [(cx + 500 * (i + 1), cy - 400 * (i + 1), cz + 200) for i in range(NUM_OUTLIERS)]
        )
        pts = np.vstack([pts, outliers])

    header = laspy.LasHeader(version="1.4", point_format=3)
    header.scales = (0.001, 0.001, 0.001)
    header.offsets = (cx, cy, cz)
    if with_crs:
        header.add_crs(CRS.from_epsg(UTM_EPSG))
    las = laspy.LasData(header)
    las.x, las.y, las.z = pts[:, 0], pts[:, 1], pts[:, 2]
    las.write(str(path))


@pytest.fixture
def las_file(tmp_path: Path) -> Path:
    path = tmp_path / "scan.las"
    _make_las(path)
    return path


class TestChecksum:
    def test_sha256_matches_hashlib(self, tmp_path: Path) -> None:
        f = tmp_path / "data.bin"
        f.write_bytes(b"slamcloude" * 1000)
        assert processing.sha256_of_file(f) == hashlib.sha256(b"slamcloude" * 1000).hexdigest()

    def test_verify_mismatch_raises(self, tmp_path: Path) -> None:
        f = tmp_path / "data.bin"
        f.write_bytes(b"payload")
        with pytest.raises(processing.ChecksumMismatchError):
            processing.verify_checksum(f, "0" * 64)


class TestDecode:
    def test_las_to_laz_preserves_points_and_crs(self, las_file: Path, tmp_path: Path) -> None:
        dst = tmp_path / "decoded.laz"
        result = processing.decode_to_laz(las_file, dst, source_name=las_file.name)
        assert result.num_points == NUM_CLUSTER_POINTS + NUM_OUTLIERS
        assert result.source_format == "las"
        assert result.crs_epsg == UTM_EPSG
        with laspy.open(dst) as reader:
            assert reader.header.point_count == result.num_points

    def test_unsupported_format_rejected(self, tmp_path: Path) -> None:
        src = tmp_path / "scan.s20raw"
        src.write_bytes(b"\x00" * 128)
        with pytest.raises(processing.UnsupportedFormatError, match="s20raw"):
            processing.decode_to_laz(src, tmp_path / "out.laz", source_name=src.name)


class TestFilterOutliers:
    def test_removes_far_points_keeps_cluster(self, las_file: Path, tmp_path: Path) -> None:
        dst = tmp_path / "filtered.laz"
        result = processing.filter_outliers(las_file, dst)
        assert result.points_in == NUM_CLUSTER_POINTS + NUM_OUTLIERS
        # All planted outliers gone, the vast majority of the cluster kept.
        assert result.points_out <= NUM_CLUSTER_POINTS
        assert result.points_out >= int(NUM_CLUSTER_POINTS * 0.9)
        with laspy.open(dst) as reader:
            assert reader.header.point_count == result.points_out
            # Remaining extent is the tight cluster, not the 2.5 km outlier spread.
            assert reader.header.maxs[0] - reader.header.mins[0] < 50


class TestGeoreference:
    def test_wgs84_bbox_matches_pyproj(self, las_file: Path) -> None:
        bbox = processing.wgs84_bbox(las_file)
        assert bbox is not None
        min_lon, min_lat, max_lon, max_lat = bbox
        assert min_lon < max_lon and min_lat < max_lat

        # Cluster center must fall inside the bbox.
        t = Transformer.from_crs(CRS.from_epsg(UTM_EPSG), "EPSG:4326", always_xy=True)
        lon, lat = t.transform(CLUSTER_CENTER[0], CLUSTER_CENTER[1])
        assert min_lon <= lon <= max_lon
        assert min_lat <= lat <= max_lat
        # UTM 37N around Moscow: sanity-check the absolute location.
        assert 30 < min_lon < 45 and 50 < min_lat < 60

    def test_no_crs_returns_none(self, tmp_path: Path) -> None:
        path = tmp_path / "nocrs.las"
        _make_las(path, with_crs=False)
        assert processing.wgs84_bbox(path) is None

    def test_bbox_polygon_ewkt(self) -> None:
        ewkt = processing.bbox_polygon_ewkt((37.0, 55.0, 38.0, 56.0))
        assert ewkt.startswith("SRID=4326;POLYGON((")
        assert ewkt.count(",") == 4  # closed ring: 5 vertices
        assert "37.0 55.0" in ewkt and "38.0 56.0" in ewkt


class TestCopc:
    def test_build_copc_when_pdal_present(self, las_file: Path, tmp_path: Path) -> None:
        if not processing.pdal_available():
            pytest.skip("pdal binary not installed (available in worker Docker image)")
        dst = tmp_path / "out.copc.laz"
        processing.build_copc(las_file, dst)
        assert dst.stat().st_size > 0
