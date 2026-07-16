"""Thin sync wrapper over boto3.

All methods block; call them via fastapi.concurrency.run_in_threadpool from
async handlers (worker tasks are sync and call them directly). Kept as a
class so tests can substitute a fake.
"""

from functools import lru_cache
from pathlib import Path
from typing import Any, cast

import boto3
from botocore.config import Config

from app.config import get_settings

# MinIO requires path-style addressing (bucket in the path, not the host).
_S3_CONFIG = Config(s3={"addressing_style": "path"})


def parse_storage_path(path: str) -> tuple[str, str]:
    """Split "s3://bucket/key..." into (bucket, key)."""
    if not path.startswith("s3://"):
        raise ValueError(f"not an s3 path: {path!r}")
    bucket, _, key = path[len("s3://"):].partition("/")
    if not bucket or not key:
        raise ValueError(f"malformed s3 path: {path!r}")
    return bucket, key


class S3Storage:
    def __init__(
        self,
        endpoint_url: str,
        access_key: str,
        secret_key: str,
        region: str,
        public_endpoint_url: str | None = None,
    ) -> None:
        def _client(endpoint: str) -> Any:
            return boto3.client(
                "s3",
                endpoint_url=endpoint,
                aws_access_key_id=access_key,
                aws_secret_access_key=secret_key,
                region_name=region,
                config=_S3_CONFIG,
            )

        self._client: Any = _client(endpoint_url)
        # Presigned URLs are consumed by browsers outside the cluster/compose
        # network, so they must be signed against the public endpoint.
        self._presign_client: Any = (
            _client(public_endpoint_url) if public_endpoint_url else self._client
        )

    def create_multipart_upload(self, bucket: str, key: str) -> str:
        resp = self._client.create_multipart_upload(Bucket=bucket, Key=key)
        return cast(str, resp["UploadId"])

    def upload_part(
        self, bucket: str, key: str, upload_id: str, part_number: int, data: bytes
    ) -> str:
        resp = self._client.upload_part(
            Bucket=bucket, Key=key, UploadId=upload_id, PartNumber=part_number, Body=data
        )
        return cast(str, resp["ETag"])

    def complete_multipart_upload(
        self, bucket: str, key: str, upload_id: str, parts: list[tuple[int, str]]
    ) -> None:
        self._client.complete_multipart_upload(
            Bucket=bucket,
            Key=key,
            UploadId=upload_id,
            MultipartUpload={
                "Parts": [{"PartNumber": n, "ETag": etag} for n, etag in parts]
            },
        )

    def abort_multipart_upload(self, bucket: str, key: str, upload_id: str) -> None:
        self._client.abort_multipart_upload(Bucket=bucket, Key=key, UploadId=upload_id)

    def put_object(self, bucket: str, key: str, data: bytes) -> None:
        self._client.put_object(Bucket=bucket, Key=key, Body=data)

    def object_size(self, bucket: str, key: str) -> int:
        resp = self._client.head_object(Bucket=bucket, Key=key)
        return cast(int, resp["ContentLength"])

    def object_exists(self, bucket: str, key: str) -> bool:
        try:
            self._client.head_object(Bucket=bucket, Key=key)
            return True
        except self._client.exceptions.ClientError:
            return False

    def download_file(self, bucket: str, key: str, path: str | Path) -> None:
        """Streams to disk (boto3 TransferManager) — constant memory."""
        self._client.download_file(bucket, key, str(path))

    def upload_file(self, bucket: str, key: str, path: str | Path) -> None:
        """Streams from disk, multipart for large files — constant memory."""
        self._client.upload_file(str(path), bucket, key)

    def copy_object(
        self, src_bucket: str, src_key: str, dst_bucket: str, dst_key: str
    ) -> None:
        """Server-side copy — no data passes through the worker."""
        self._client.copy(
            {"Bucket": src_bucket, "Key": src_key}, dst_bucket, dst_key
        )

    def presign_get(self, bucket: str, key: str, expires_seconds: int = 900) -> str:
        url = self._presign_client.generate_presigned_url(
            "get_object",
            Params={"Bucket": bucket, "Key": key},
            ExpiresIn=expires_seconds,
        )
        return cast(str, url)


@lru_cache
def get_storage() -> S3Storage:
    s = get_settings()
    return S3Storage(
        endpoint_url=s.s3_endpoint_url,
        access_key=s.s3_access_key,
        secret_key=s.s3_secret_key,
        region=s.s3_region,
        public_endpoint_url=s.s3_public_endpoint_url,
    )
