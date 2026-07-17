from fastapi import APIRouter

from app.api import projects, scans, users

router = APIRouter()
router.include_router(projects.router)
router.include_router(scans.router)
router.include_router(users.router)


@router.get("/health", tags=["service"])
async def health() -> dict[str, str]:
    return {"status": "ok"}
