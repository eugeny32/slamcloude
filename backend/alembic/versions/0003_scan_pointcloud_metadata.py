"""scans: point cloud metadata filled by the pipeline

Revision ID: 0003
Revises: 0002
Create Date: 2026-07-16

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "0003"
down_revision: str | None = "0002"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    op.add_column("scans", sa.Column("num_points", sa.BigInteger(), nullable=True))
    op.add_column("scans", sa.Column("source_format", sa.String(16), nullable=True))
    op.add_column("scans", sa.Column("crs_epsg", sa.Integer(), nullable=True))


def downgrade() -> None:
    op.drop_column("scans", "crs_epsg")
    op.drop_column("scans", "source_format")
    op.drop_column("scans", "num_points")
