import hashlib
import tempfile
import uuid
from datetime import UTC, datetime
from pathlib import Path as FsPath
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException, Path, Query, Request, status
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import RedirectResponse
from geoalchemy2.shape import to_shape
from sqlalchemy import select

from app.api.deps import CurrentUser, SessionDep, get_owned_project, get_owned_scan
from app.config import get_settings
from app.models import (
    PIPELINE_ORDER,
    AssetType,
    Job,
    JobStatus,
    ProcessedAsset,
    Scan,
    ScanInput,
    ScanInputKind,
    ScanStatus,
)
from app.ratelimit import RateLimiter
from app.schemas import (
    AssetOut,
    JobOut,
    ReprocessRequest,
    ScanInputOut,
    ScanStatusOut,
    UploadCompleteRequest,
    UploadInitRequest,
    UploadInitResponse,
    UploadPartResponse,
)
from app.services.queue import enqueue_pipeline, try_enqueue_pipeline
from app.services.s3 import get_storage, parse_storage_path
from app.services.upload import (
    compute_num_parts,
    key_from_raw_path,
    raw_file_path,
    raw_object_key,
)

router = APIRouter(prefix="/scans", tags=["scans"])

# Uploads are heavier than plain API calls: tighter window.
_upload_limit = RateLimiter("upload", limit=60, window_seconds=60)
_default_limit = RateLimiter("api")


def _scan_bbox(scan: Scan) -> tuple[float, float, float, float] | None:
    if scan.bbox is None:
        return None
    minx, miny, maxx, maxy = to_shape(scan.bbox).bounds
    return (minx, miny, maxx, maxy)


def _scan_status_out(
    scan: Scan, jobs: list[Job], assets: list[ProcessedAsset] | None = None
) -> ScanStatusOut:
    order = {step: i for i, step in enumerate(PIPELINE_ORDER)}
    return ScanStatusOut(
        id=scan.id,
        project_id=scan.project_id,
        status=scan.status,
        raw_file_path=scan.raw_file_path,
        size_bytes=scan.size_bytes,
        checksum_sha256=scan.checksum_sha256,
        captured_at=scan.captured_at,
        rtk_fixed=scan.rtk_fixed,
        num_points=scan.num_points,
        source_format=scan.source_format,
        crs_epsg=scan.crs_epsg,
        bbox=_scan_bbox(scan),
        created_at=scan.created_at,
        jobs=[
            JobOut.model_validate(j)
            for j in sorted(jobs, key=lambda j: order[j.pipeline_step])
        ],
        assets=[AssetOut.model_validate(a) for a in assets or []],
    )


@router.post(
    "/upload",
    response_model=UploadInitResponse,
    status_code=status.HTTP_201_CREATED,
    dependencies=[Depends(_upload_limit)],
)
async def initiate_upload(
    body: UploadInitRequest, user: CurrentUser, session: SessionDep
) -> UploadInitResponse:
    settings = get_settings()
    await get_owned_project(session, body.project_id, user)

    if body.file_size > settings.max_upload_size_bytes:
        raise HTTPException(
            status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
            f"file_size exceeds limit of {settings.max_upload_size_bytes} bytes",
        )
    try:
        num_parts = compute_num_parts(body.file_size, settings.upload_part_size)
        scan_id = uuid.uuid4()
        key = raw_object_key(str(scan_id), body.filename)
    except ValueError as exc:
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, str(exc)) from exc

    storage = get_storage()
    upload_id: str = await run_in_threadpool(
        storage.create_multipart_upload, settings.s3_bucket_raw, key
    )

    scan = Scan(
        id=scan_id,
        project_id=body.project_id,
        raw_file_path=raw_file_path(settings.s3_bucket_raw, key),
        status=ScanStatus.UPLOADING,
        captured_at=body.captured_at,
        size_bytes=body.file_size,
        checksum_sha256=body.checksum_sha256.lower() if body.checksum_sha256 else None,
    )
    session.add(scan)
    await session.commit()

    return UploadInitResponse(
        scan_id=scan_id,
        upload_id=upload_id,
        part_size=settings.upload_part_size,
        num_parts=num_parts,
    )


@router.put(
    "/{scan_id}/upload/parts/{part_number}",
    response_model=UploadPartResponse,
    dependencies=[Depends(_upload_limit)],
    openapi_extra={
        "requestBody": {
            "content": {
                "application/octet-stream": {"schema": {"type": "string", "format": "binary"}}
            },
            "required": True,
        }
    },
)
async def upload_part(
    request: Request,
    scan_id: uuid.UUID,
    part_number: Annotated[int, Path(ge=1, le=10_000)],
    upload_id: Annotated[str, Query(min_length=1)],
    user: CurrentUser,
    session: SessionDep,
) -> UploadPartResponse:
    """Receive one raw body part and relay it to S3 multipart.

    The part is buffered (bounded by 2x configured part size, ~128 MiB) —
    never the whole file. Optional X-Part-SHA256 header is verified.
    """
    settings = get_settings()
    scan = await get_owned_scan(session, scan_id, user)
    if scan.status is not ScanStatus.UPLOADING:
        raise HTTPException(status.HTTP_409_CONFLICT, f"Scan is in status '{scan.status}'")

    max_part = settings.upload_part_size * 2
    hasher = hashlib.sha256()
    buf = bytearray()
    async for chunk in request.stream():
        buf.extend(chunk)
        hasher.update(chunk)
        if len(buf) > max_part:
            raise HTTPException(
                status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                f"part exceeds maximum size of {max_part} bytes",
            )
    if not buf:
        raise HTTPException(status.HTTP_400_BAD_REQUEST, "empty part body")

    digest = hasher.hexdigest()
    declared = request.headers.get("X-Part-SHA256")
    if declared and declared.lower() != digest:
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            "part checksum mismatch: data corrupted in transit, retry this part",
        )

    storage = get_storage()
    key = key_from_raw_path(scan.raw_file_path or "", settings.s3_bucket_raw)
    etag: str = await run_in_threadpool(
        storage.upload_part, settings.s3_bucket_raw, key, upload_id, part_number, bytes(buf)
    )
    return UploadPartResponse(part_number=part_number, etag=etag, sha256=digest)


@router.post(
    "/{scan_id}/upload/complete",
    response_model=ScanStatusOut,
    dependencies=[Depends(_upload_limit)],
)
async def complete_upload(
    scan_id: uuid.UUID,
    body: UploadCompleteRequest,
    user: CurrentUser,
    session: SessionDep,
) -> ScanStatusOut:
    settings = get_settings()
    scan = await get_owned_scan(session, scan_id, user)
    if scan.status is not ScanStatus.UPLOADING:
        raise HTTPException(status.HTTP_409_CONFLICT, f"Scan is in status '{scan.status}'")

    part_numbers = [p.part_number for p in body.parts]
    if len(set(part_numbers)) != len(part_numbers):
        raise HTTPException(status.HTTP_422_UNPROCESSABLE_ENTITY, "duplicate part numbers")
    parts = sorted(((p.part_number, p.etag) for p in body.parts), key=lambda x: x[0])

    storage = get_storage()
    key = key_from_raw_path(scan.raw_file_path or "", settings.s3_bucket_raw)
    await run_in_threadpool(
        storage.complete_multipart_upload, settings.s3_bucket_raw, key, body.upload_id, parts
    )

    actual_size: int = await run_in_threadpool(
        storage.object_size, settings.s3_bucket_raw, key
    )
    if scan.size_bytes is not None and actual_size != scan.size_bytes:
        scan.status = ScanStatus.FAILED
        await session.commit()
        raise HTTPException(
            status.HTTP_400_BAD_REQUEST,
            f"size mismatch: declared {scan.size_bytes}, stored {actual_size}; "
            "scan marked failed, start a new upload",
        )

    scan.status = ScanStatus.UPLOADED
    jobs = [
        Job(id=uuid.uuid4(), scan_id=scan.id, pipeline_step=step, status=JobStatus.PENDING)
        for step in PIPELINE_ORDER
    ]
    session.add_all(jobs)
    await session.commit()

    # Best-effort: if the broker is down the upload is still intact and the
    # client restarts processing via POST /scans/{id}/process.
    await run_in_threadpool(try_enqueue_pipeline, scan.id)
    return _scan_status_out(scan, jobs)


@router.put(
    "/{scan_id}/inputs/{kind}",
    response_model=ScanInputOut,
    dependencies=[Depends(_upload_limit)],
    openapi_extra={
        "requestBody": {
            "content": {
                "application/octet-stream": {"schema": {"type": "string", "format": "binary"}}
            },
            "required": True,
        }
    },
)
async def upload_scan_input(
    request: Request,
    scan_id: uuid.UUID,
    kind: ScanInputKind,
    user: CurrentUser,
    session: SessionDep,
    filename: Annotated[str | None, Query(max_length=512)] = None,
) -> ScanInput:
    """Attach an auxiliary source file to a scan (raw body): PPK trajectory
    (.pos), rover RINEX obs, base station RINEX, ephemerides. Re-upload of
    the same kind replaces the previous file."""
    settings = get_settings()
    scan = await get_owned_scan(session, scan_id, user)

    suffix = FsPath(filename).suffix.lower()[:16] if filename else ""
    key = f"{scan.id}/inputs/{kind.value}{suffix}"

    # Stream to a temp file (bounded memory), then stream to S3.
    size = 0
    with tempfile.NamedTemporaryFile(delete=False) as tmp:
        tmp_path = FsPath(tmp.name)
        async for chunk in request.stream():
            size += len(chunk)
            if size > settings.max_upload_size_bytes:
                raise HTTPException(
                    status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                    f"input file exceeds limit of {settings.max_upload_size_bytes} bytes",
                )
            tmp.write(chunk)
    try:
        if size == 0:
            raise HTTPException(status.HTTP_400_BAD_REQUEST, "empty input file body")
        await run_in_threadpool(
            get_storage().upload_file, settings.s3_bucket_raw, key, tmp_path
        )
    finally:
        tmp_path.unlink(missing_ok=True)

    storage_path = f"s3://{settings.s3_bucket_raw}/{key}"
    existing = (
        await session.execute(
            select(ScanInput).where(ScanInput.scan_id == scan.id, ScanInput.kind == kind)
        )
    ).scalar_one_or_none()
    if existing is not None:
        existing.storage_path = storage_path
        existing.file_size = size
        existing.uploaded_at = datetime.now(UTC)
        record = existing
    else:
        record = ScanInput(
            id=uuid.uuid4(),
            scan_id=scan.id,
            kind=kind,
            storage_path=storage_path,
            file_size=size,
            uploaded_at=datetime.now(UTC),
        )
        session.add(record)
    await session.commit()
    return record


@router.get(
    "/{scan_id}/inputs",
    response_model=list[ScanInputOut],
    dependencies=[Depends(_default_limit)],
)
async def list_scan_inputs(
    scan_id: uuid.UUID, user: CurrentUser, session: SessionDep
) -> list[ScanInput]:
    scan = await get_owned_scan(session, scan_id, user)
    result = await session.execute(
        select(ScanInput).where(ScanInput.scan_id == scan.id).order_by(ScanInput.kind)
    )
    return list(result.scalars().all())


@router.post(
    "/{scan_id}/reprocess",
    status_code=status.HTTP_202_ACCEPTED,
    dependencies=[Depends(_default_limit)],
)
async def reprocess_scan(
    scan_id: uuid.UUID,
    body: ReprocessRequest,
    user: CurrentUser,
    session: SessionDep,
) -> dict[str, str]:
    """Recompute from a given step, reusing upstream intermediates in S3 —
    e.g. after uploading a base station RINEX: PPK + re-georeference without
    re-decoding/re-filtering the raw data."""
    scan = await get_owned_scan(session, scan_id, user, with_details=True)
    if scan.status is ScanStatus.UPLOADING:
        raise HTTPException(status.HTTP_409_CONFLICT, "Upload is not complete yet")
    if scan.status is ScanStatus.PROCESSING:
        raise HTTPException(status.HTTP_409_CONFLICT, "Pipeline is already running")

    from_idx = PIPELINE_ORDER.index(body.from_step)
    jobs_by_step = {j.pipeline_step: j for j in scan.jobs}
    for step in PIPELINE_ORDER[:from_idx]:
        job = jobs_by_step.get(step)
        if job is None or job.status is not JobStatus.COMPLETED:
            raise HTTPException(
                status.HTTP_409_CONFLICT,
                f"upstream step '{step.value}' is not completed — its intermediate "
                "result is missing; use POST /scans/{id}/process for a full run",
            )
    for step in PIPELINE_ORDER[from_idx:]:
        job = jobs_by_step.get(step)
        if job is None:
            # Scans processed before this step existed get the row on demand.
            session.add(
                Job(id=uuid.uuid4(), scan_id=scan.id, pipeline_step=step,
                    status=JobStatus.PENDING)
            )
        else:
            job.status = JobStatus.PENDING
            job.started_at = None
            job.finished_at = None
            job.error_message = None
    scan.status = ScanStatus.UPLOADED
    await session.commit()

    await run_in_threadpool(enqueue_pipeline, scan.id)
    return {"detail": f"reprocess from '{body.from_step.value}' enqueued"}


@router.post(
    "/{scan_id}/process",
    status_code=status.HTTP_202_ACCEPTED,
    dependencies=[Depends(_default_limit)],
)
async def process_scan(
    scan_id: uuid.UUID, user: CurrentUser, session: SessionDep
) -> dict[str, str]:
    """(Re)start the pipeline: resumes from the failed step, completed steps
    are skipped by the worker."""
    scan = await get_owned_scan(session, scan_id, user)
    if scan.status in (ScanStatus.UPLOADING,):
        raise HTTPException(status.HTTP_409_CONFLICT, "Upload is not complete yet")
    if scan.status is ScanStatus.PROCESSING:
        raise HTTPException(status.HTTP_409_CONFLICT, "Pipeline is already running")
    await run_in_threadpool(enqueue_pipeline, scan.id)
    return {"detail": "pipeline enqueued"}


@router.get(
    "/{scan_id}/status",
    response_model=ScanStatusOut,
    dependencies=[Depends(_default_limit)],
)
async def scan_status(
    scan_id: uuid.UUID, user: CurrentUser, session: SessionDep
) -> ScanStatusOut:
    scan = await get_owned_scan(session, scan_id, user, with_details=True)
    return _scan_status_out(scan, list(scan.jobs), list(scan.assets))


def _latest_asset(scan: Scan, asset_type: AssetType) -> ProcessedAsset | None:
    matching = [a for a in scan.assets if a.asset_type is asset_type]
    return max(matching, key=lambda a: a.version, default=None)


@router.get(
    "/{scan_id}/download",
    status_code=status.HTTP_307_TEMPORARY_REDIRECT,
    response_class=RedirectResponse,
    dependencies=[Depends(_default_limit)],
)
async def download_scan(
    scan_id: uuid.UUID,
    user: CurrentUser,
    session: SessionDep,
    format: Annotated[str, Query(pattern="^(las|e57|ply)$")] = "las",
) -> RedirectResponse:
    """Redirect to a short-lived presigned URL for the processed point cloud."""
    if format != "las":
        raise HTTPException(
            status.HTTP_501_NOT_IMPLEMENTED,
            f"export to {format} is not available yet; use format=las",
        )
    scan = await get_owned_scan(session, scan_id, user, with_details=True)
    asset = _latest_asset(scan, AssetType.LAS)
    if asset is None:
        raise HTTPException(
            status.HTTP_404_NOT_FOUND,
            f"no processed point cloud yet (scan status: '{scan.status}')",
        )
    bucket, key = parse_storage_path(asset.storage_path)
    url = await run_in_threadpool(get_storage().presign_get, bucket, key, 900)
    return RedirectResponse(url, status_code=status.HTTP_307_TEMPORARY_REDIRECT)


@router.get("/{scan_id}/preview", dependencies=[Depends(_default_limit)])
async def scan_preview(
    scan_id: uuid.UUID, user: CurrentUser, session: SessionDep
) -> dict[str, object]:
    """Metadata for the streaming web viewer: COPC URL (LOD octree, ranged
    reads) plus extent/point count. The frontend (Potree/Cesium) consumes this."""
    scan = await get_owned_scan(session, scan_id, user, with_details=True)
    copc = _latest_asset(scan, AssetType.COPC)
    copc_url: str | None = None
    if copc is not None:
        bucket, key = parse_storage_path(copc.storage_path)
        copc_url = await run_in_threadpool(get_storage().presign_get, bucket, key, 3600)
    return {
        "scan_id": str(scan.id),
        "status": scan.status.value,
        "num_points": scan.num_points,
        "bbox": _scan_bbox(scan),
        "crs_epsg": scan.crs_epsg,
        "copc_url": copc_url,
    }
