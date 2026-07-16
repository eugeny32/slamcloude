"""scans: declared size and checksum for upload validation

Revision ID: 0002
Revises: 0001
Create Date: 2026-07-16

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "0002"
down_revision: str | None = "0001"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    op.add_column("scans", sa.Column("size_bytes", sa.BigInteger(), nullable=True))
    op.add_column("scans", sa.Column("checksum_sha256", sa.String(64), nullable=True))


def downgrade() -> None:
    op.drop_column("scans", "checksum_sha256")
    op.drop_column("scans", "size_bytes")
