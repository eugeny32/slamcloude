"""initial schema: users, projects, scans, jobs, processed_assets

Revision ID: 0001
Revises:
Create Date: 2026-07-16

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op
from geoalchemy2 import Geometry

revision: str = "0001"
down_revision: str | None = None
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None

_plan_tier = sa.Enum("free", "pro", "enterprise", name="plan_tier", native_enum=False)
_scan_status = sa.Enum(
    "uploading", "uploaded", "processing", "completed", "failed",
    name="scan_status", native_enum=False,
)
_pipeline_step = sa.Enum(
    "decode_raw", "filter_outliers", "georeference", "colorize", "build_octree",
    name="pipeline_step", native_enum=False,
)
_job_status = sa.Enum(
    "pending", "running", "completed", "failed", name="job_status", native_enum=False
)
_asset_type = sa.Enum("las", "copc", "mesh", "splat", name="asset_type", native_enum=False)


def upgrade() -> None:
    op.execute("CREATE EXTENSION IF NOT EXISTS postgis")

    op.create_table(
        "users",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column("email", sa.String(320), nullable=False, unique=True),
        sa.Column("api_key_hash", sa.String(128), nullable=False, unique=True),
        sa.Column("plan_tier", _plan_tier, nullable=False, server_default="free"),
        sa.Column(
            "created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()
        ),
    )

    op.create_table(
        "projects",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "owner_id",
            sa.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("name", sa.String(255), nullable=False),
        sa.Column(
            "created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()
        ),
    )
    op.create_index("ix_projects_owner_id", "projects", ["owner_id"])

    op.create_table(
        "scans",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "project_id",
            sa.UUID(as_uuid=True),
            sa.ForeignKey("projects.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("raw_file_path", sa.String(1024)),
        sa.Column("status", _scan_status, nullable=False, server_default="uploading"),
        sa.Column("captured_at", sa.DateTime(timezone=True)),
        sa.Column("bbox", Geometry(geometry_type="POLYGON", srid=4326, spatial_index=False)),
        sa.Column("rtk_fixed", sa.Boolean(), nullable=False, server_default=sa.false()),
        sa.Column(
            "created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()
        ),
    )
    op.create_index("ix_scans_project_id", "scans", ["project_id"])
    op.create_index("ix_scans_status", "scans", ["status"])
    op.create_index("ix_scans_bbox", "scans", ["bbox"], postgresql_using="gist")

    op.create_table(
        "jobs",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "scan_id",
            sa.UUID(as_uuid=True),
            sa.ForeignKey("scans.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("pipeline_step", _pipeline_step, nullable=False),
        sa.Column("status", _job_status, nullable=False, server_default="pending"),
        sa.Column("started_at", sa.DateTime(timezone=True)),
        sa.Column("finished_at", sa.DateTime(timezone=True)),
        sa.Column("error_message", sa.Text()),
    )
    op.create_index("ix_jobs_scan_id", "jobs", ["scan_id"])
    op.create_index("ix_jobs_status", "jobs", ["status"])

    op.create_table(
        "processed_assets",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "scan_id",
            sa.UUID(as_uuid=True),
            sa.ForeignKey("scans.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("asset_type", _asset_type, nullable=False),
        sa.Column("storage_path", sa.String(1024), nullable=False),
        sa.Column("file_size", sa.BigInteger(), nullable=False),
        sa.Column("version", sa.Integer(), nullable=False, server_default="1"),
        sa.Column(
            "created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()
        ),
    )
    op.create_index("ix_processed_assets_scan_id", "processed_assets", ["scan_id"])


def downgrade() -> None:
    op.drop_table("processed_assets")
    op.drop_table("jobs")
    op.drop_table("scans")
    op.drop_table("projects")
    op.drop_table("users")
