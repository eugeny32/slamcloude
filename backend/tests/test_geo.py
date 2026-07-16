import pytest

from app.services.geo import BBoxError, parse_bbox


def test_valid_bbox() -> None:
    assert parse_bbox("37.5,55.6,37.8,55.8") == (37.5, 55.6, 37.8, 55.8)


def test_negative_coordinates() -> None:
    assert parse_bbox("-122.5,-37.9,-122.3,-37.7") == (-122.5, -37.9, -122.3, -37.7)


@pytest.mark.parametrize(
    "value",
    [
        "37.5,55.6,37.8",  # too few
        "37.5,55.6,37.8,55.8,1",  # too many
        "a,55.6,37.8,55.8",  # non-numeric
        "181,55.6,37.8,55.8",  # lon out of range
        "37.5,91,37.8,55.8",  # lat out of range
        "37.8,55.6,37.5,55.8",  # min_lon >= max_lon
        "37.5,55.8,37.8,55.6",  # min_lat >= max_lat
        "37.5,55.6,37.5,55.8",  # degenerate (zero width)
    ],
)
def test_invalid_bbox_rejected(value: str) -> None:
    with pytest.raises(BBoxError):
        parse_bbox(value)
