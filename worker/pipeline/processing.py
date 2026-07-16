"""Point cloud processing core — pure functions over local files, no S3/DB.

Stack: laspy(+lazrs)/numpy/scipy/pyproj — pip wheels on every platform, so
this module runs and is unit-tested on dev machines too. PDAL is used
opportunistically (subprocess) for COPC octree generation when the binary is
present (it is in the worker Docker image); Open3D/GPU implementations can
replace individual functions later without touching orchestration.

Memory: files are processed in chunks of CHUNK_POINTS, never fully loaded.
"""

import copy
import hashlib
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import laspy
import numpy as np
from pyproj import Transformer
from scipy.spatial import cKDTree

CHUNK_POINTS = 2_000_000
SUPPORTED_FORMATS = {".las", ".laz"}


class ProcessingError(RuntimeError):
    pass


class UnsupportedFormatError(ProcessingError):
    pass


class ChecksumMismatchError(ProcessingError):
    pass


def sha256_of_file(path: str | Path, chunk_size: int = 8 * 1024 * 1024) -> str:
    hasher = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(chunk_size):
            hasher.update(chunk)
    return hasher.hexdigest()


def verify_checksum(path: str | Path, expected_sha256: str) -> None:
    actual = sha256_of_file(path)
    if actual != expected_sha256.lower():
        raise ChecksumMismatchError(
            f"raw file checksum mismatch: declared {expected_sha256}, actual {actual}"
        )


@dataclass
class DecodeResult:
    num_points: int
    source_format: str
    crs_epsg: int | None


def decode_to_laz(src: str | Path, dst: str | Path, source_name: str) -> DecodeResult:
    """Normalize supported input into LAZ. SHARE S20 proprietary raw stream
    decoding plugs in here once the vendor format spec / SDK is available."""
    ext = Path(source_name).suffix.lower()
    if ext not in SUPPORTED_FORMATS:
        raise UnsupportedFormatError(
            f"unsupported raw format {ext!r}; supported: {sorted(SUPPORTED_FORMATS)} "
            "(SHARE S20 native stream support pending vendor SDK)"
        )
    with laspy.open(src) as reader:
        crs = reader.header.parse_crs()
        header = copy.deepcopy(reader.header)
        with laspy.open(dst, mode="w", header=header) as writer:
            for chunk in reader.chunk_iterator(CHUNK_POINTS):
                writer.write_points(chunk)
        num_points = reader.header.point_count
    return DecodeResult(
        num_points=num_points,
        source_format=ext.lstrip("."),
        crs_epsg=crs.to_epsg() if crs is not None else None,
    )


@dataclass
class FilterResult:
    points_in: int
    points_out: int


def filter_outliers(
    src: str | Path, dst: str | Path, k: int = 8, std_ratio: float = 2.0
) -> FilterResult:
    """Statistical outlier removal: drop points whose mean distance to their k
    nearest neighbours exceeds mean + std_ratio * std.

    kNN is computed per chunk (bounded memory); for chunks of millions of
    points this approximates global SOR well.
    """
    points_in = 0
    points_out = 0
    with laspy.open(src) as reader:
        header = copy.deepcopy(reader.header)
        with laspy.open(dst, mode="w", header=header) as writer:
            for chunk in reader.chunk_iterator(CHUNK_POINTS):
                n = len(chunk)
                points_in += n
                if n <= k + 1:
                    writer.write_points(chunk)
                    points_out += n
                    continue
                pts = np.column_stack(
                    (np.asarray(chunk.x), np.asarray(chunk.y), np.asarray(chunk.z))
                )
                tree = cKDTree(pts)
                # k+1 because the nearest neighbour of a point is itself.
                dists, _ = tree.query(pts, k=k + 1)
                mean_dist = dists[:, 1:].mean(axis=1)
                threshold = mean_dist.mean() + std_ratio * mean_dist.std()
                mask = mean_dist <= threshold
                writer.write_points(chunk[mask])
                points_out += int(mask.sum())
    return FilterResult(points_in=points_in, points_out=points_out)


def wgs84_bbox(src: str | Path) -> tuple[float, float, float, float] | None:
    """(minLon, minLat, maxLon, maxLat) of the file extent, or None if the
    file carries no CRS (nothing to georeference against)."""
    with laspy.open(src) as reader:
        crs = reader.header.parse_crs()
        mins = reader.header.mins
        maxs = reader.header.maxs
    if crs is None:
        return None
    transformer = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    corners_x = [mins[0], mins[0], maxs[0], maxs[0]]
    corners_y = [mins[1], maxs[1], mins[1], maxs[1]]
    lons, lats = transformer.transform(corners_x, corners_y)
    return (min(lons), min(lats), max(lons), max(lats))


def bbox_polygon_ewkt(bbox: tuple[float, float, float, float]) -> str:
    """EWKT polygon for scans.bbox (geometry(POLYGON, 4326))."""
    min_lon, min_lat, max_lon, max_lat = bbox
    ring = (
        f"{min_lon} {min_lat}, {max_lon} {min_lat}, "
        f"{max_lon} {max_lat}, {min_lon} {max_lat}, {min_lon} {min_lat}"
    )
    return f"SRID=4326;POLYGON(({ring}))"


def pdal_available() -> bool:
    return shutil.which("pdal") is not None


def build_copc(src: str | Path, dst: str | Path) -> None:
    """Multi-resolution COPC octree via PDAL (present in the worker image)."""
    pipeline = json.dumps(
        {
            "pipeline": [
                {"type": "readers.las", "filename": str(src)},
                {"type": "writers.copc", "filename": str(dst)},
            ]
        }
    )
    result = subprocess.run(
        ["pdal", "pipeline", "--stdin"],
        input=pipeline.encode(),
        capture_output=True,
        timeout=6 * 3600,
    )
    if result.returncode != 0:
        raise ProcessingError(
            f"pdal pipeline failed (rc={result.returncode}): {result.stderr.decode()[:2000]}"
        )
