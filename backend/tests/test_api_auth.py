"""Auth behavior without a real database (session dependency overridden)."""

from collections.abc import Iterator
from typing import Any

import pytest
from fastapi.testclient import TestClient

from app.db import get_session
from app.main import app

_VALID_INIT_BODY = {
    "project_id": "00000000-0000-0000-0000-000000000001",
    "filename": "scan.raw",
    "file_size": 1024,
}


class _EmptyResult:
    def scalar_one_or_none(self) -> None:
        return None


class _FakeSession:
    async def execute(self, *args: Any, **kwargs: Any) -> _EmptyResult:
        return _EmptyResult()


@pytest.fixture
def client() -> Iterator[TestClient]:
    app.dependency_overrides[get_session] = lambda: _FakeSession()
    with TestClient(app) as c:
        yield c
    app.dependency_overrides.clear()


def test_health_needs_no_auth(client: TestClient) -> None:
    assert client.get("/health").status_code == 200


def test_upload_without_key_is_401(client: TestClient) -> None:
    resp = client.post("/scans/upload", json=_VALID_INIT_BODY)
    assert resp.status_code == 401


def test_upload_with_invalid_key_is_401(client: TestClient) -> None:
    resp = client.post(
        "/scans/upload", json=_VALID_INIT_BODY, headers={"X-API-Key": "sk_invalid"}
    )
    assert resp.status_code == 401
    assert resp.json()["detail"] == "Invalid API key"


def test_projects_without_key_is_401(client: TestClient) -> None:
    assert client.get("/projects").status_code == 401
