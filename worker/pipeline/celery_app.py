from celery import Celery

from app.config import get_settings

settings = get_settings()

celery_app = Celery(
    "slamcloude",
    broker=settings.redis_url,
    backend=settings.redis_url,
    include=["pipeline.tasks"],
)

celery_app.conf.update(
    task_serializer="json",
    result_serializer="json",
    accept_content=["json"],
    # Point cloud steps are long and heavy: one task per worker process at a
    # time, and re-deliver on worker crash so a step is never silently lost.
    task_acks_late=True,
    worker_prefetch_multiplier=1,
)
