"""scan_inputs: auxiliary source files (trajectory, GNSS obs, base RINEX)

Revision ID: 0004
Revises: 0003
Create Date: 2026-07-16

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "0004"
down_revision: str | None = "0003"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None

_kind = sa.Enum(
    "trajectory", "rover_obs", "base_rinex", "nav",
    name="scan_input_kind", native_enum=False,
)


def upgrade() -> None:
    op.create_table(
        "scan_inputs",
        sa.Column("id", sa.UUID(as_uuid=True), primary_key=True),
        sa.Column(
            "scan_id",
            sa.UUID(as_uuid=True),
            sa.ForeignKey("scans.id", ondelete="CASCADE"),
            nullable=False,
        ),
        sa.Column("kind", _kind, nullable=False),
        sa.Column("storage_path", sa.String(1024), nullable=False),
        sa.Column("file_size", sa.BigInteger(), nullable=False),
        sa.Column(
            "uploaded_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()
        ),
    )
    op.create_index("ix_scan_inputs_scan_id", "scan_inputs", ["scan_id"])
    op.create_index(
        "uq_scan_inputs_scan_kind", "scan_inputs", ["scan_id", "kind"], unique=True
    )


def downgrade() -> None:
    op.drop_table("scan_inputs")
