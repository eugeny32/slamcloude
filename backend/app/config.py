from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

    # Defaults target the local docker-compose stack (infra/docker-compose.yml).
    database_url: str = "postgresql+psycopg://slam:slam@localhost:5432/slamcloude"
    redis_url: str = "redis://localhost:6379/0"

    s3_endpoint_url: str = "http://localhost:9000"
    # Endpoint reachable by end-user browsers (for presigned URLs); in
    # docker-compose the internal endpoint http://minio:9000 is not routable
    # from the host, so this is set to http://localhost:9000 there.
    s3_public_endpoint_url: str | None = None
    s3_access_key: str = "minioadmin"
    s3_secret_key: str = "minioadmin"
    s3_bucket_raw: str = "raw-scans"
    s3_bucket_processed: str = "processed-assets"
    s3_region: str = "us-east-1"

    # Chunked upload: 64 MiB parts keep memory flat for 50+ GB files.
    # S3 multipart limits: part >= 5 MiB (except last), <= 10 000 parts.
    upload_part_size: int = 64 * 1024 * 1024
    max_upload_size_bytes: int = 500 * 1024**3

    # Simulated duration of each stub pipeline step (roadmap step 4).
    pipeline_stub_seconds: float = 0.5

    # Frontend origins allowed to call the API from the browser.
    cors_origins: list[str] = ["http://localhost:5173", "http://localhost:3000"]

    rate_limit_enabled: bool = True
    # Fixed window per API key; generous defaults for MVP.
    rate_limit_requests: int = 300
    rate_limit_window_seconds: int = 60


@lru_cache
def get_settings() -> Settings:
    return Settings()
