import enum
import uuid
from datetime import datetime

from geoalchemy2 import Geometry
from sqlalchemy import (
    BigInteger,
    Boolean,
    DateTime,
    Enum,
    ForeignKey,
    Index,
    Integer,
    String,
    Text,
    func,
)
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from app.db import Base


class PlanTier(enum.StrEnum):
    FREE = "free"
    PRO = "pro"
    ENTERPRISE = "enterprise"


class ScanStatus(enum.StrEnum):
    UPLOADING = "uploading"
    UPLOADED = "uploaded"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


class PipelineStep(enum.StrEnum):
    DECODE_RAW = "decode_raw"
    FILTER_OUTLIERS = "filter_outliers"
    PPK_CORRECTION = "ppk_correction"
    GEOREFERENCE = "georeference"
    COLORIZE = "colorize"
    BUILD_OCTREE = "build_octree"


class JobStatus(enum.StrEnum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


class AssetType(enum.StrEnum):
    LAS = "las"
    COPC = "copc"
    MESH = "mesh"
    SPLAT = "splat"


# Canonical execution order; one Job row per step per scan.
PIPELINE_ORDER: tuple[PipelineStep, ...] = (
    PipelineStep.DECODE_RAW,
    PipelineStep.FILTER_OUTLIERS,
    PipelineStep.PPK_CORRECTION,
    PipelineStep.GEOREFERENCE,
    PipelineStep.COLORIZE,
    PipelineStep.BUILD_OCTREE,
)


class ScanInputKind(enum.StrEnum):
    """Auxiliary source files accompanying a scan's raw point stream."""

    TRAJECTORY = "trajectory"  # PPK/SLAM trajectory from the scanner (.pos)
    ROVER_OBS = "rover_obs"  # raw GNSS observations from the scanner (RINEX)
    BASE_RINEX = "base_rinex"  # base station observations (RINEX) for PPK
    NAV = "nav"  # broadcast ephemerides (RINEX nav)


def _enum(e: type[enum.Enum], name: str) -> Enum:
    # Store enum *values* as plain VARCHAR (validated at the app level):
    # adding a member is free, no ALTER TYPE as with native PG enums.
    return Enum(e, name=name, native_enum=False, values_callable=lambda x: [i.value for i in x])


class User(Base):
    __tablename__ = "users"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    email: Mapped[str] = mapped_column(String(320), unique=True, nullable=False)
    api_key_hash: Mapped[str] = mapped_column(String(128), unique=True, nullable=False)
    plan_tier: Mapped[PlanTier] = mapped_column(
        _enum(PlanTier, "plan_tier"), nullable=False, default=PlanTier.FREE
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    projects: Mapped[list["Project"]] = relationship(back_populates="owner")


class Project(Base):
    __tablename__ = "projects"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    owner_id: Mapped[uuid.UUID] = mapped_column(
        ForeignKey("users.id", ondelete="CASCADE"), nullable=False, index=True
    )
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    owner: Mapped[User] = relationship(back_populates="projects")
    scans: Mapped[list["Scan"]] = relationship(back_populates="project")


class Scan(Base):
    __tablename__ = "scans"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    project_id: Mapped[uuid.UUID] = mapped_column(
        ForeignKey("projects.id", ondelete="CASCADE"), nullable=False, index=True
    )
    raw_file_path: Mapped[str | None] = mapped_column(String(1024))
    status: Mapped[ScanStatus] = mapped_column(
        _enum(ScanStatus, "scan_status"), nullable=False, default=ScanStatus.UPLOADING, index=True
    )
    captured_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    # Declared by the client at upload init; verified after multipart complete
    # (size) and during pipeline decode (checksum, streaming).
    size_bytes: Mapped[int | None] = mapped_column(BigInteger)
    checksum_sha256: Mapped[str | None] = mapped_column(String(64))
    # Filled by the pipeline: decode_raw (points/format/CRS), georeference (bbox).
    num_points: Mapped[int | None] = mapped_column(BigInteger)
    source_format: Mapped[str | None] = mapped_column(String(16))
    crs_epsg: Mapped[int | None] = mapped_column(Integer)
    # WGS84 footprint of the scan; GiST index (below) serves "scans in this area" queries.
    bbox = mapped_column(Geometry(geometry_type="POLYGON", srid=4326, spatial_index=False))
    rtk_fixed: Mapped[bool] = mapped_column(Boolean, nullable=False, default=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    project: Mapped[Project] = relationship(back_populates="scans")
    jobs: Mapped[list["Job"]] = relationship(back_populates="scan")
    assets: Mapped[list["ProcessedAsset"]] = relationship(back_populates="scan")
    inputs: Mapped[list["ScanInput"]] = relationship(back_populates="scan")

    __table_args__ = (Index("ix_scans_bbox", "bbox", postgresql_using="gist"),)


class ScanInput(Base):
    __tablename__ = "scan_inputs"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    scan_id: Mapped[uuid.UUID] = mapped_column(
        ForeignKey("scans.id", ondelete="CASCADE"), nullable=False, index=True
    )
    kind: Mapped[ScanInputKind] = mapped_column(
        _enum(ScanInputKind, "scan_input_kind"), nullable=False
    )
    storage_path: Mapped[str] = mapped_column(String(1024), nullable=False)
    file_size: Mapped[int] = mapped_column(BigInteger, nullable=False)
    uploaded_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    scan: Mapped[Scan] = relationship(back_populates="inputs")

    # One file per kind per scan; re-upload replaces it.
    __table_args__ = (Index("uq_scan_inputs_scan_kind", "scan_id", "kind", unique=True),)


class Job(Base):
    __tablename__ = "jobs"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    scan_id: Mapped[uuid.UUID] = mapped_column(
        ForeignKey("scans.id", ondelete="CASCADE"), nullable=False, index=True
    )
    pipeline_step: Mapped[PipelineStep] = mapped_column(
        _enum(PipelineStep, "pipeline_step"), nullable=False
    )
    status: Mapped[JobStatus] = mapped_column(
        _enum(JobStatus, "job_status"), nullable=False, default=JobStatus.PENDING, index=True
    )
    started_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    finished_at: Mapped[datetime | None] = mapped_column(DateTime(timezone=True))
    error_message: Mapped[str | None] = mapped_column(Text)

    scan: Mapped[Scan] = relationship(back_populates="jobs")


class ProcessedAsset(Base):
    __tablename__ = "processed_assets"

    id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True), primary_key=True, default=uuid.uuid4
    )
    scan_id: Mapped[uuid.UUID] = mapped_column(
        ForeignKey("scans.id", ondelete="CASCADE"), nullable=False, index=True
    )
    asset_type: Mapped[AssetType] = mapped_column(_enum(AssetType, "asset_type"), nullable=False)
    storage_path: Mapped[str] = mapped_column(String(1024), nullable=False)
    file_size: Mapped[int] = mapped_column(BigInteger, nullable=False)
    version: Mapped[int] = mapped_column(Integer, nullable=False, default=1)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), nullable=False, server_default=func.now()
    )

    scan: Mapped[Scan] = relationship(back_populates="assets")
