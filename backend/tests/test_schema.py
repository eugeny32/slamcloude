"""Schema sanity checks against SQLAlchemy metadata (no DB required)."""

from geoalchemy2 import Geometry

import app.models  # noqa: F401
from app.db import Base


def test_all_tables_defined() -> None:
    assert set(Base.metadata.tables) == {
        "users",
        "projects",
        "scans",
        "scan_inputs",
        "jobs",
        "processed_assets",
    }


def test_scans_bbox_is_wgs84_polygon_with_gist_index() -> None:
    scans = Base.metadata.tables["scans"]
    bbox = scans.c.bbox
    assert isinstance(bbox.type, Geometry)
    assert bbox.type.srid == 4326
    assert bbox.type.geometry_type == "POLYGON"

    gist = next(ix for ix in scans.indexes if ix.name == "ix_scans_bbox")
    assert gist.dialect_options["postgresql"]["using"] == "gist"


def test_processed_assets_file_size_is_bigint() -> None:
    # 50+ GB raw files: a 32-bit int would overflow at 2 GB.
    col = Base.metadata.tables["processed_assets"].c.file_size
    assert "BIGINT" in str(col.type).upper()
