"""Coordinate parsing/validation — critical logic, covered by unit tests."""


class BBoxError(ValueError):
    pass


def parse_bbox(value: str) -> tuple[float, float, float, float]:
    """Parse "minLon,minLat,maxLon,maxLat" (EPSG:4326) with range validation."""
    parts = value.split(",")
    if len(parts) != 4:
        raise BBoxError("bbox must have exactly 4 comma-separated numbers")
    try:
        min_lon, min_lat, max_lon, max_lat = (float(p) for p in parts)
    except ValueError as exc:
        raise BBoxError(f"bbox contains a non-numeric value: {value!r}") from exc

    if not (-180.0 <= min_lon <= 180.0 and -180.0 <= max_lon <= 180.0):
        raise BBoxError("longitude out of range [-180, 180]")
    if not (-90.0 <= min_lat <= 90.0 and -90.0 <= max_lat <= 90.0):
        raise BBoxError("latitude out of range [-90, 90]")
    if min_lon >= max_lon or min_lat >= max_lat:
        raise BBoxError("bbox min corner must be strictly south-west of max corner")
    return (min_lon, min_lat, max_lon, max_lat)
