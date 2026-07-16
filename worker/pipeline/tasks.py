"""Pipeline tasks: one Celery task per step, statuses persisted in `jobs`.

Steps run as a chain; a failure stops the chain with the failed job recorded,
and re-sending `pipeline.run` resumes from the failed step (completed steps
are skipped).

Data flow between steps goes through S3 (steps may run on different worker
pods): raw bucket holds `{scan_id}/intermediate/<step>.laz`, the processed
bucket receives final assets. Files are streamed to/from local temp storage
and processed in chunks — a 50 GB cloud never sits in memory.
"""

import shutil
import tempfile
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from celery import chain
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from app.config import get_settings
from app.models import (
    PIPELINE_ORDER,
    AssetType,
    Job,
    JobStatus,
    PipelineStep,
    ProcessedAsset,
    Scan,
    ScanInput,
    ScanInputKind,
    ScanStatus,
)
from app.services.s3 import get_storage, parse_storage_path
from pipeline import gnss, processing
from pipeline.celery_app import celery_app
from pipeline.db import SessionLocal


@dataclass
class StepOutcome:
    """What a step reports back to be persisted on the scan."""

    num_points: int | None = None
    source_format: str | None = None
    crs_epsg: int | None = None
    bbox_ewkt: str | None = None
    rtk_fixed: bool | None = None
    # (asset_type, storage_path, file_size)
    assets: list[tuple[AssetType, str, int]] = field(default_factory=list)


def intermediate_key(scan_id: str, step: PipelineStep) -> str:
    return f"{scan_id}/intermediate/{step.value}.laz"


def corrected_trajectory_key(scan_id: str) -> str:
    return f"{scan_id}/intermediate/trajectory_corrected.pos"


def build_pipeline(scan_id: str) -> Any:
    """Chain of all steps in canonical order; completed steps skip themselves."""
    return chain(*(run_step.si(scan_id, step.value) for step in PIPELINE_ORDER))


@celery_app.task(name="pipeline.ping")
def ping() -> str:
    return "pong"


@celery_app.task(name="pipeline.run")
def run_pipeline(scan_id: str) -> None:
    build_pipeline(scan_id).apply_async()


@celery_app.task(name="pipeline.step")
def run_step(scan_id: str, step_value: str) -> str:
    step = PipelineStep(step_value)
    sid = uuid.UUID(scan_id)

    with SessionLocal() as session:
        job = _get_job(session, sid, step)
        if job.status is JobStatus.COMPLETED:
            return "skipped"
        job.status = JobStatus.RUNNING
        job.started_at = datetime.now(UTC)
        job.finished_at = None
        job.error_message = None
        scan = session.get_one(Scan, sid)
        scan.status = ScanStatus.PROCESSING
        session.commit()

    try:
        outcome = _execute_step(scan_id, step)
    except Exception as exc:
        with SessionLocal() as session:
            job = _get_job(session, sid, step)
            job.status = JobStatus.FAILED
            job.finished_at = datetime.now(UTC)
            job.error_message = f"{type(exc).__name__}: {exc}"[:2000]
            session.get_one(Scan, sid).status = ScanStatus.FAILED
            session.commit()
        raise

    with SessionLocal() as session:
        job = _get_job(session, sid, step)
        job.status = JobStatus.COMPLETED
        job.finished_at = datetime.now(UTC)
        scan = session.get_one(Scan, sid)
        if outcome.num_points is not None:
            scan.num_points = outcome.num_points
        if outcome.source_format is not None:
            scan.source_format = outcome.source_format
        if outcome.crs_epsg is not None:
            scan.crs_epsg = outcome.crs_epsg
        if outcome.bbox_ewkt is not None:
            scan.bbox = outcome.bbox_ewkt
        if outcome.rtk_fixed is not None:
            scan.rtk_fixed = outcome.rtk_fixed
        for asset_type, path, size in outcome.assets:
            # Reprocessing produces a new version; download/preview serve the latest.
            latest = session.execute(
                select(func.max(ProcessedAsset.version)).where(
                    ProcessedAsset.scan_id == sid,
                    ProcessedAsset.asset_type == asset_type,
                )
            ).scalar()
            session.add(
                ProcessedAsset(
                    id=uuid.uuid4(),
                    scan_id=sid,
                    asset_type=asset_type,
                    storage_path=path,
                    file_size=size,
                    version=(latest or 0) + 1,
                )
            )
        if step is PIPELINE_ORDER[-1]:
            scan.status = ScanStatus.COMPLETED
        session.commit()
    return "completed"


def _get_job(session: Session, scan_id: uuid.UUID, step: PipelineStep) -> Job:
    return session.execute(
        select(Job).where(Job.scan_id == scan_id, Job.pipeline_step == step)
    ).scalar_one()


def _load_inputs(scan_id: str) -> dict[ScanInputKind, str]:
    """kind -> storage_path for the scan's auxiliary input files."""
    with SessionLocal() as session:
        rows = session.execute(
            select(ScanInput).where(ScanInput.scan_id == uuid.UUID(scan_id))
        ).scalars()
        return {row.kind: row.storage_path for row in rows}


def _download_input(
    storage: Any, inputs: dict[ScanInputKind, str], kind: ScanInputKind, workdir: Path
) -> Path:
    bucket, key = parse_storage_path(inputs[kind])
    local = workdir / f"{kind.value}{Path(key).suffix}"
    storage.download_file(bucket, key, local)
    return local


def _execute_step(scan_id: str, step: PipelineStep) -> StepOutcome:
    settings = get_settings()
    storage = get_storage()
    raw_bucket = settings.s3_bucket_raw

    with SessionLocal() as session:
        scan = session.get_one(Scan, uuid.UUID(scan_id))
        raw_path = scan.raw_file_path or ""
        checksum = scan.checksum_sha256

    workdir = Path(tempfile.mkdtemp(prefix=f"slam-{step.value}-"))
    try:
        if step is PipelineStep.DECODE_RAW:
            src_bucket, src_key = parse_storage_path(raw_path)
            local_raw = workdir / Path(src_key).name
            storage.download_file(src_bucket, src_key, local_raw)
            if checksum:
                processing.verify_checksum(local_raw, checksum)
            local_out = workdir / "decoded.laz"
            decoded = processing.decode_to_laz(local_raw, local_out, source_name=src_key)
            storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
            return StepOutcome(
                num_points=decoded.num_points,
                source_format=decoded.source_format,
                crs_epsg=decoded.crs_epsg,
            )

        if step is PipelineStep.FILTER_OUTLIERS:
            local_in = workdir / "in.laz"
            local_out = workdir / "filtered.laz"
            storage.download_file(
                raw_bucket, intermediate_key(scan_id, PipelineStep.DECODE_RAW), local_in
            )
            result = processing.filter_outliers(local_in, local_out)
            storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
            return StepOutcome(num_points=result.points_out)

        if step is PipelineStep.PPK_CORRECTION:
            # PPK: solve the rover's raw GNSS observations against a base
            # station RINEX. Without a base file the step is a no-op — the
            # user uploads one later and hits POST /scans/{id}/reprocess.
            inputs = _load_inputs(scan_id)
            if ScanInputKind.BASE_RINEX not in inputs:
                return StepOutcome()
            for required in (ScanInputKind.ROVER_OBS, ScanInputKind.TRAJECTORY):
                if required not in inputs:
                    raise processing.ProcessingError(
                        f"PPK correction needs the '{required.value}' input file "
                        "alongside base_rinex"
                    )
            if not gnss.rnx2rtkp_available():
                raise processing.ProcessingError(
                    "rnx2rtkp (RTKLIB) is not installed on this worker; "
                    "PPK runs in the worker Docker image"
                )
            rover = _download_input(storage, inputs, ScanInputKind.ROVER_OBS, workdir)
            base = _download_input(storage, inputs, ScanInputKind.BASE_RINEX, workdir)
            nav = (
                _download_input(storage, inputs, ScanInputKind.NAV, workdir)
                if ScanInputKind.NAV in inputs
                else None
            )
            corrected_pos = workdir / "corrected.pos"
            gnss.run_rnx2rtkp(rover, base, corrected_pos, nav=nav)
            corrected = gnss.parse_pos(corrected_pos)
            storage.upload_file(raw_bucket, corrected_trajectory_key(scan_id), corrected_pos)
            return StepOutcome(rtk_fixed=gnss.fixed_ratio(corrected) >= 0.5)

        if step is PipelineStep.GEOREFERENCE:
            local_in = workdir / "in.laz"
            storage.download_file(
                raw_bucket,
                intermediate_key(scan_id, PipelineStep.FILTER_OUTLIERS),
                local_in,
            )

            corrected_key = corrected_trajectory_key(scan_id)
            inputs = _load_inputs(scan_id)
            if (
                ScanInputKind.TRAJECTORY in inputs
                and storage.object_exists(raw_bucket, corrected_key)
            ):
                # Shift the cloud by (corrected - original) trajectory delta,
                # interpolated at each point's gps_time.
                original_pos = _download_input(
                    storage, inputs, ScanInputKind.TRAJECTORY, workdir
                )
                corrected_pos = workdir / "corrected.pos"
                storage.download_file(raw_bucket, corrected_key, corrected_pos)
                local_out = workdir / "georef.laz"
                gnss.apply_trajectory_correction(
                    local_in,
                    local_out,
                    original=gnss.parse_pos(original_pos),
                    corrected=gnss.parse_pos(corrected_pos),
                )
                storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
                bbox = processing.wgs84_bbox(local_out)
            else:
                # No PPK correction: georeferencing relies on the CRS carried
                # by the file itself. Points unchanged — server-side copy.
                storage.copy_object(
                    raw_bucket,
                    intermediate_key(scan_id, PipelineStep.FILTER_OUTLIERS),
                    raw_bucket,
                    intermediate_key(scan_id, step),
                )
                bbox = processing.wgs84_bbox(local_in)
            return StepOutcome(
                bbox_ewkt=processing.bbox_polygon_ewkt(bbox) if bbox else None
            )

        if step is PipelineStep.COLORIZE:
            # Colorization needs the two 16 MP camera streams + calibration —
            # pending SHARE S20 camera data spec. Pass-through for now.
            storage.copy_object(
                raw_bucket,
                intermediate_key(scan_id, PipelineStep.GEOREFERENCE),
                raw_bucket,
                intermediate_key(scan_id, step),
            )
            return StepOutcome()

        if step is PipelineStep.BUILD_OCTREE:
            local_in = workdir / "final.laz"
            storage.download_file(
                raw_bucket, intermediate_key(scan_id, PipelineStep.COLORIZE), local_in
            )
            assets: list[tuple[AssetType, str, int]] = []

            las_key = f"{scan_id}/pointcloud.laz"
            storage.upload_file(settings.s3_bucket_processed, las_key, local_in)
            assets.append(
                (
                    AssetType.LAS,
                    f"s3://{settings.s3_bucket_processed}/{las_key}",
                    local_in.stat().st_size,
                )
            )

            if processing.pdal_available():
                local_copc = workdir / "pointcloud.copc.laz"
                processing.build_copc(local_in, local_copc)
                copc_key = f"{scan_id}/pointcloud.copc.laz"
                storage.upload_file(settings.s3_bucket_processed, copc_key, local_copc)
                assets.append(
                    (
                        AssetType.COPC,
                        f"s3://{settings.s3_bucket_processed}/{copc_key}",
                        local_copc.stat().st_size,
                    )
                )
            return StepOutcome(assets=assets)

        raise processing.ProcessingError(f"unknown step: {step}")
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
