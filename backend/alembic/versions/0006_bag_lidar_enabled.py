"""rename photogrammetry_enabled to bag_lidar_enabled; remove dense_stereo jobs

Revision ID: 0006
Revises: 0005
Create Date: 2026-07-17

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

revision: str = "0006"
down_revision: str | None = "0005"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    # Rename column (PostgreSQL supports this directly)
    op.alter_column("scans", "photogrammetry_enabled", new_column_name="bag_lidar_enabled")

    # Remove jobs for the removed dense_stereo step
    op.execute(
        "DELETE FROM jobs WHERE pipeline_step = 'dense_stereo'"
    )

    # Update scan_input_kind enum: rename stereo_images -> camera_frames for existing rows
    op.execute(
        "UPDATE scan_inputs SET kind = 'camera_frames' WHERE kind = 'stereo_images'"
    )


def downgrade() -> None:
    op.execute(
        "UPDATE scan_inputs SET kind = 'stereo_images' WHERE kind = 'camera_frames'"
    )
    op.alter_column("scans", "bag_lidar_enabled", new_column_name="photogrammetry_enabled")