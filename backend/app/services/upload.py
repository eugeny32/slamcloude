"""Pure upload-planning logic, unit-tested without I/O."""

import math
import re

S3_MIN_PART_SIZE = 5 * 1024 * 1024
S3_MAX_PARTS = 10_000

_FILENAME_SAFE = re.compile(r"[^A-Za-z0-9._-]+")


def compute_num_parts(file_size: int, part_size: int) -> int:
    """Number of multipart parts for a file, validating S3 constraints."""
    if file_size <= 0:
        raise ValueError("file_size must be positive")
    if part_size < S3_MIN_PART_SIZE:
        raise ValueError(f"part_size must be >= {S3_MIN_PART_SIZE} bytes")
    num_parts = math.ceil(file_size / part_size)
    if num_parts > S3_MAX_PARTS:
        raise ValueError(
            f"file of {file_size} bytes needs {num_parts} parts of {part_size} bytes; "
            f"S3 allows at most {S3_MAX_PARTS} — increase part_size"
        )
    return num_parts


def sanitize_filename(filename: str) -> str:
    """Strip any path components and unsafe characters; never returns empty."""
    # Take the basename regardless of separator style the client used.
    base = filename.replace("\\", "/").rsplit("/", 1)[-1].strip()
    safe = _FILENAME_SAFE.sub("_", base).strip("._")
    if not safe:
        raise ValueError("filename resolves to an empty name")
    return safe


def raw_object_key(scan_id: str, filename: str) -> str:
    return f"{scan_id}/{sanitize_filename(filename)}"


def raw_file_path(bucket: str, key: str) -> str:
    return f"s3://{bucket}/{key}"


def key_from_raw_path(path: str, bucket: str) -> str:
    prefix = f"s3://{bucket}/"
    if not path.startswith(prefix):
        raise ValueError(f"raw_file_path {path!r} is not in bucket {bucket!r}")
    return path[len(prefix):]
