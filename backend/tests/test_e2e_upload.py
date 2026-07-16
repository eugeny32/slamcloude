"""End-to-end flow against the real docker-compose stack.

Skipped unless SLAMCLOUDE_E2E=1 and the stack is up:
    docker compose -f infra/docker-compose.yml up -d
    # create a user, then:
    $env:SLAMCLOUDE_E2E="1"; $env:SLAMCLOUDE_E2E_API_KEY="sk_..."
    uv run pytest backend/tests/test_e2e_upload.py
"""

import hashlib
import io
import os
import time

import httpx
import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SLAMCLOUDE_E2E") != "1",
    reason="requires running docker-compose stack (set SLAMCLOUDE_E2E=1)",
)

BASE = os.environ.get("SLAMCLOUDE_E2E_URL", "http://localhost:8000")


def _make_las_bytes() -> bytes:
    """Small real LAS file: point cluster in UTM 37N with a few outliers."""
    import laspy
    import numpy as np
    from pyproj import CRS

    rng = np.random.default_rng(7)
    cx, cy, cz = 410_000.0, 6_170_000.0, 150.0
    pts = rng.normal(0.0, 2.0, size=(300, 3)) + (cx, cy, cz)
    pts = np.vstack([pts, [[cx + 900, cy - 900, cz + 300]]])

    header = laspy.LasHeader(version="1.4", point_format=3)
    header.scales = (0.001, 0.001, 0.001)
    header.offsets = (cx, cy, cz)
    header.add_crs(CRS.from_epsg(32637))
    las = laspy.LasData(header)
    las.x, las.y, las.z = pts[:, 0], pts[:, 1], pts[:, 2]
    buf = io.BytesIO()
    las.write(buf)
    return buf.getvalue()


@pytest.fixture
def client() -> httpx.Client:
    api_key = os.environ["SLAMCLOUDE_E2E_API_KEY"]
    return httpx.Client(base_url=BASE, headers={"X-API-Key": api_key}, timeout=120)


def test_full_upload_and_processing_flow(client: httpx.Client) -> None:
    project = client.post("/projects", json={"name": "e2e"})
    assert project.status_code == 201, project.text
    project_id = project.json()["id"]

    payload = _make_las_bytes()
    init = client.post(
        "/scans/upload",
        json={
            "project_id": project_id,
            "filename": "e2e-scan.las",
            "file_size": len(payload),
            "checksum_sha256": hashlib.sha256(payload).hexdigest(),
        },
    )
    assert init.status_code == 201, init.text
    scan_id = init.json()["scan_id"]
    upload_id = init.json()["upload_id"]

    part = client.put(
        f"/scans/{scan_id}/upload/parts/1",
        params={"upload_id": upload_id},
        content=payload,
        headers={
            "Content-Type": "application/octet-stream",
            "X-Part-SHA256": hashlib.sha256(payload).hexdigest(),
        },
    )
    assert part.status_code == 200, part.text
    etag = part.json()["etag"]

    done = client.post(
        f"/scans/{scan_id}/upload/complete",
        json={"upload_id": upload_id, "parts": [{"part_number": 1, "etag": etag}]},
    )
    assert done.status_code == 200, done.text
    assert done.json()["status"] == "uploaded"
    assert len(done.json()["jobs"]) == 5

    # Pipeline is auto-enqueued; wait for real processing to finish.
    deadline = time.monotonic() + 180
    body = {}
    while time.monotonic() < deadline:
        st = client.get(f"/scans/{scan_id}/status")
        assert st.status_code == 200
        body = st.json()
        if body["status"] in ("completed", "failed"):
            break
        time.sleep(2)

    assert body["status"] == "completed", body
    assert all(j["status"] == "completed" for j in body["jobs"])

    # Real metadata extracted by the pipeline.
    assert body["source_format"] == "las"
    assert body["crs_epsg"] == 32637
    assert 250 <= body["num_points"] <= 301  # outliers filtered out
    assert body["bbox"] is not None
    min_lon, min_lat, max_lon, max_lat = body["bbox"]
    assert 30 < min_lon < 45 and 50 < min_lat < 60  # UTM 37N — Moscow area

    # Assets: LAS always; COPC when pdal is in the worker image (it is).
    types = {a["asset_type"] for a in body["assets"]}
    assert "las" in types
    assert "copc" in types

    # Geospatial search finds the scan by its bbox.
    found = client.get(
        f"/projects/{project_id}/scans", params={"bbox": "30,50,45,60"}
    )
    assert found.status_code == 200
    assert scan_id in {s["id"] for s in found.json()}

    # Download redirects to a presigned URL that actually serves the file.
    dl = client.get(f"/scans/{scan_id}/download", params={"format": "las"})
    assert dl.status_code == 307
    content = httpx.get(dl.headers["location"], timeout=60)
    assert content.status_code == 200
    assert len(content.content) > 0

    # Preview metadata for the web viewer.
    preview = client.get(f"/scans/{scan_id}/preview")
    assert preview.status_code == 200
    assert preview.json()["copc_url"]
