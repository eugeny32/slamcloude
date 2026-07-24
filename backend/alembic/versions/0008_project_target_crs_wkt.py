"""add projects.target_crs_wkt

Revision ID: 0008
Revises: 0007
Create Date: 2026-07-23

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "0008"
down_revision: str | None = "0007"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    op.add_column("projects", sa.Column("target_crs_wkt", sa.String(), nullable=True))


def downgrade() -> None:
    op.drop_column("projects", "target_crs_wkt")
