import hashlib
import secrets
from typing import Annotated

from fastapi import Depends, HTTPException, Security, status
from fastapi.security import APIKeyHeader
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from app.db import get_session
from app.models import User

API_KEY_PREFIX = "sk_"

_api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)


def generate_api_key() -> str:
    return API_KEY_PREFIX + secrets.token_urlsafe(32)


def hash_api_key(api_key: str) -> str:
    # Keys are high-entropy random strings, so a fast unsalted hash is fine
    # (unlike passwords, they cannot be dictionary-attacked).
    return hashlib.sha256(api_key.encode()).hexdigest()


async def get_current_user(
    api_key: Annotated[str | None, Security(_api_key_header)],
    session: Annotated[AsyncSession, Depends(get_session)],
) -> User:
    if not api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing X-API-Key header",
        )
    result = await session.execute(select(User).where(User.api_key_hash == hash_api_key(api_key)))
    user = result.scalar_one_or_none()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key",
        )
    return user
