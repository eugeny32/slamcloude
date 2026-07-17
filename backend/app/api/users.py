"""User profile and API-key management endpoints."""
import uuid

from fastapi import APIRouter, Depends, status
from pydantic import BaseModel

from app.api.deps import CurrentUser, SessionDep
from app.ratelimit import RateLimiter
from app.security import generate_api_key, hash_api_key

router = APIRouter(prefix="/users", tags=["users"])
_default_limit = RateLimiter("api")


class UserOut(BaseModel):
    id: uuid.UUID
    email: str


class RotateKeyOut(BaseModel):
    api_key: str


@router.get("/me", response_model=UserOut, dependencies=[Depends(_default_limit)])
async def get_me(user: CurrentUser) -> UserOut:
    return UserOut(id=user.id, email=user.email)


@router.post(
    "/me/rotate-key",
    response_model=RotateKeyOut,
    status_code=status.HTTP_200_OK,
    dependencies=[Depends(_default_limit)],
)
async def rotate_api_key(user: CurrentUser, session: SessionDep) -> RotateKeyOut:
    """Generate a new API key. The old key is immediately invalidated."""
    new_key = generate_api_key()
    user.api_key_hash = hash_api_key(new_key)
    await session.commit()
    return RotateKeyOut(api_key=new_key)
