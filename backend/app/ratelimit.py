import logging
from typing import Annotated

import redis.asyncio as aioredis
from fastapi import Depends, HTTPException, status

from app.config import get_settings
from app.models import User
from app.security import get_current_user

logger = logging.getLogger(__name__)

_redis: aioredis.Redis | None = None


def _get_redis() -> aioredis.Redis:
    global _redis
    if _redis is None:
        _redis = aioredis.from_url(  # type: ignore[no-untyped-call]
            get_settings().redis_url
        )
    return _redis


class RateLimiter:
    """Fixed-window limiter per API key backed by Redis.

    Fails open: an unreachable Redis must not take the API down with it.
    """

    def __init__(self, scope: str, limit: int | None = None, window_seconds: int | None = None):
        self.scope = scope
        self.limit = limit
        self.window_seconds = window_seconds

    async def __call__(self, user: Annotated[User, Depends(get_current_user)]) -> None:
        settings = get_settings()
        if not settings.rate_limit_enabled:
            return
        limit = self.limit or settings.rate_limit_requests
        window = self.window_seconds or settings.rate_limit_window_seconds
        key = f"ratelimit:{self.scope}:{user.id}"
        try:
            r = _get_redis()
            count = await r.incr(key)
            if count == 1:
                await r.expire(key, window)
        except Exception:
            logger.warning("Rate limiter Redis unavailable, failing open", exc_info=True)
            return
        if count > limit:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Rate limit exceeded: {limit} requests per {window}s",
            )
