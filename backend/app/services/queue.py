"""Enqueue pipeline runs by task name — the API never imports worker code."""

import logging
import uuid
from functools import lru_cache

from celery import Celery

from app.config import get_settings

logger = logging.getLogger(__name__)

PIPELINE_RUN_TASK = "pipeline.run"


@lru_cache
def get_celery() -> Celery:
    settings = get_settings()
    return Celery("slamcloude-api", broker=settings.redis_url, backend=settings.redis_url)


def enqueue_pipeline(scan_id: uuid.UUID) -> None:
    """Blocking (broker round-trip): call via run_in_threadpool from handlers."""
    get_celery().send_task(PIPELINE_RUN_TASK, args=[str(scan_id)])


def try_enqueue_pipeline(scan_id: uuid.UUID) -> bool:
    """Best-effort enqueue: an unreachable broker must not fail a finished
    upload — the client can retry via POST /scans/{id}/process."""
    try:
        enqueue_pipeline(scan_id)
        return True
    except Exception:
        logger.warning("Failed to enqueue pipeline for scan %s", scan_id, exc_info=True)
        return False
