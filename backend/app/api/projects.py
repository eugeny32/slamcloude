import uuid
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy import func, select

from app.api.deps import CurrentUser, SessionDep, get_owned_project
from app.models import Project, Scan
from app.ratelimit import RateLimiter
from app.schemas import ProjectCreate, ProjectOut, ScanOut
from app.services.geo import BBoxError, parse_bbox

router = APIRouter(prefix="/projects", tags=["projects"])

_default_limit = RateLimiter("api")


@router.post(
    "",
    response_model=ProjectOut,
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(_default_limit)],
)
async def create_project(
    body: ProjectCreate, user: CurrentUser, session: SessionDep
) -> Project:
    project = Project(id=uuid.uuid4(), owner_id=user.id, name=body.name)
    session.add(project)
    await session.commit()
    return project


@router.get("", response_model=list[ProjectOut], dependencies=[Depends(_default_limit)])
async def list_projects(user: CurrentUser, session: SessionDep) -> list[Project]:
    result = await session.execute(
        select(Project).where(Project.owner_id == user.id).order_by(Project.created_at)
    )
    return list(result.scalars().all())


@router.get(
    "/{project_id}/scans",
    response_model=list[ScanOut],
    dependencies=[Depends(_default_limit)],
)
async def list_project_scans(
    project_id: uuid.UUID,
    user: CurrentUser,
    session: SessionDep,
    bbox: Annotated[
        str | None, Query(description="minLon,minLat,maxLon,maxLat (EPSG:4326)")
    ] = None,
) -> list[Scan]:
    await get_owned_project(session, project_id, user)

    stmt = select(Scan).where(Scan.project_id == project_id).order_by(Scan.created_at)
    if bbox is not None:
        try:
            min_lon, min_lat, max_lon, max_lat = parse_bbox(bbox)
        except BBoxError as exc:
            raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, str(exc)) from exc
        envelope = func.ST_MakeEnvelope(min_lon, min_lat, max_lon, max_lat, 4326)
        stmt = stmt.where(Scan.bbox.isnot(None), func.ST_Intersects(Scan.bbox, envelope))

    result = await session.execute(stmt)
    return list(result.scalars().all())
