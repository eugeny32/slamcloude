import pytest

from app.services.upload import (
    S3_MIN_PART_SIZE,
    compute_num_parts,
    key_from_raw_path,
    raw_file_path,
    raw_object_key,
    sanitize_filename,
)

MIB = 1024 * 1024


class TestComputeNumParts:
    def test_exact_multiple(self) -> None:
        assert compute_num_parts(128 * MIB, 64 * MIB) == 2

    def test_remainder_adds_part(self) -> None:
        assert compute_num_parts(128 * MIB + 1, 64 * MIB) == 3

    def test_file_smaller_than_part(self) -> None:
        assert compute_num_parts(1, 64 * MIB) == 1

    def test_50_gb_file(self) -> None:
        assert compute_num_parts(50 * 1024 * MIB, 64 * MIB) == 800

    def test_zero_size_rejected(self) -> None:
        with pytest.raises(ValueError, match="positive"):
            compute_num_parts(0, 64 * MIB)

    def test_part_below_s3_minimum_rejected(self) -> None:
        with pytest.raises(ValueError, match=str(S3_MIN_PART_SIZE)):
            compute_num_parts(100 * MIB, MIB)

    def test_too_many_parts_rejected(self) -> None:
        with pytest.raises(ValueError, match="10000|10_000|at most"):
            compute_num_parts(10_001 * 5 * MIB, 5 * MIB)


class TestSanitizeFilename:
    def test_plain_name_unchanged(self) -> None:
        assert sanitize_filename("scan_2026-07-16.s20raw") == "scan_2026-07-16.s20raw"

    def test_path_traversal_stripped(self) -> None:
        assert sanitize_filename("../../etc/passwd") == "passwd"
        assert sanitize_filename("..\\..\\windows\\cmd.exe") == "cmd.exe"

    def test_unsafe_chars_replaced(self) -> None:
        # Non-ASCII/specials collapse to "_"; leading "._" are trimmed
        # so a name can never become a hidden dotfile.
        assert sanitize_filename("мой скан (1).las") == "1_.las"

    def test_empty_after_sanitize_rejected(self) -> None:
        with pytest.raises(ValueError):
            sanitize_filename("///")


def test_key_roundtrip() -> None:
    key = raw_object_key("abc-123", "scan.raw")
    path = raw_file_path("raw-scans", key)
    assert path == "s3://raw-scans/abc-123/scan.raw"
    assert key_from_raw_path(path, "raw-scans") == key


def test_key_from_wrong_bucket_rejected() -> None:
    with pytest.raises(ValueError):
        key_from_raw_path("s3://other/abc/scan.raw", "raw-scans")
