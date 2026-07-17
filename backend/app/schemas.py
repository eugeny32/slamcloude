import uuid
from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field

from app.models import AssetType, JobStatus, PipelineStep, ScanInputKind, ScanStatus


class ProjectCreate(BaseModel):
    name: str = Field(min_length=1, max_length=255)


class ProjectOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: uuid.UUID
    name: str
    created_at: datetime


class UploadInitRequest(BaseModel):
    project_id: uuid.UUID
    filename: str = Field(min_length=1, max_length=512)
    file_size: int = Field(gt=0)
    captured_at: datetime | None = None
    # SHA-256 of the whole file, hex. Stored and verified by the pipeline
    # during decode (streaming); per-part integrity is checked at upload time.
    checksum_sha256: str | None = Field(None, pattern=r"^[0-9a-fA-F]{64}$")


class UploadInitResponse(BaseModel):
    scan_id: uuid.UUID
    upload_id: str
    part_size: int
    num_parts: int


class UploadPartResponse(BaseModel):
    part_number: int
    etag: str
    sha256: str


class PartETag(BaseModel):
    part_number: int = Field(ge=1, le=10_000)
    etag: str = Field(min_length=1)


class UploadCompleteRequest(BaseModel):
    upload_id: str = Field(min_length=1)
    parts: list[PartETag] = Field(min_length=1)


class ScanInputOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    kind: ScanInputKind
    storage_path: str
    file_size: int
    uploaded_at: datetime


class ReprocessRequest(BaseModel):
    # Default: everything downstream of the (possibly new) PPK correction is
    # recomputed; decode/filter intermediates in S3 are reused as-is.
    from_step: PipelineStep = PipelineStep.PPK_CORRECTION


class JobOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: uuid.UUID
    pipeline_step: PipelineStep
    status: JobStatus
    started_at: datetime | None
    finished_at: datetime | None
    error_message: str | None


class AssetOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: uuid.UUID
    asset_type: AssetType
    storage_path: str
    file_size: int
    version: int
    created_at: datetime


class ScanStatusOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: uuid.UUID
    project_id: uuid.UUID
    status: ScanStatus
    raw_file_path: str | None
    size_bytes: int | None
    checksum_sha256: str | None
    captured_at: datetime | None
    rtk_fixed: bool
    num_points: int | None
    source_format: str | None
    crs_epsg: int | None
    # (minLon, minLat, maxLon, maxLat) in EPSG:4326, set by georeference step.
    bbox: tuple[float, float, float, float] | None
    created_at: datetime
    jobs: list[JobOut]
    assets: list[AssetOut] = []


class ScanOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: uuid.UUID
    status: ScanStatus
    captured_at: datetime | None
    rtk_fixed: bool
    bag_lidar_enabled: bool
    size_bytes: int | None
    created_at: datetime
