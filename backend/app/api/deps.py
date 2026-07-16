import uuid
from typing import Annotated

from fastapi import Depends, HTTPException, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.db import get_session
from app.models import Project, Scan, User
from app.security import get_current_user

SessionDep = Annotated[AsyncSession, Depends(get_session)]
CurrentUser = Annotated[User, Depends(get_current_user)]


async def get_owned_project(
    session: AsyncSession, project_id: uuid.UUID, user: User
) -> Project:
    project = await session.get(Project, project_id)
    if project is None or project.owner_id != user.id:
        # 404 (not 403) so we don't leak which project ids exist.
        raise HTTPException(status.HTTP_404_NOT_FOUND, "Project not found")
    return project


async def get_owned_scan(
    session: AsyncSession, scan_id: uuid.UUID, user: User, *, with_details: bool = False
) -> Scan:
    stmt = (
        select(Scan)
        .join(Project, Scan.project_id == Project.id)
        .where(Scan.id == scan_id, Project.owner_id == user.id)
    )
    if with_details:
        stmt = stmt.options(selectinload(Scan.jobs), selectinload(Scan.assets))
    scan = (await session.execute(stmt)).scalar_one_or_none()
    if scan is None:
        raise HTTPException(status.HTTP_404_NOT_FOUND, "Scan not found")
    return scan
