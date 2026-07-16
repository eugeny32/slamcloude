from fastapi.testclient import TestClient

from app.main import app

client = TestClient(app)


def test_health() -> None:
    resp = client.get("/health")
    assert resp.status_code == 200
    assert resp.json() == {"status": "ok"}


def test_stub_endpoints_registered() -> None:
    paths = set(app.openapi()["paths"])
    assert {
        "/scans/upload",
        "/scans/{scan_id}/status",
        "/scans/{scan_id}/download",
        "/scans/{scan_id}/preview",
        "/projects/{project_id}/scans",
    } <= paths
