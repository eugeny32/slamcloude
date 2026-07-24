"""Pipeline tasks: one Celery task per step, statuses persisted in `jobs`.

Steps run as a chain; a failure stops the chain with the failed job recorded,
and re-sending `pipeline.run` resumes from the failed step (completed steps
are skipped).

Data flow between steps goes through S3 (steps may run on different worker
pods): raw bucket holds `{scan_id}/intermediate/<step>.laz`, the processed
bucket receives final assets.
"""

import shutil
import tempfile
import uuid
import zipfile
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
from pipeline import gnss, processing, s20
from pipeline.celery_app import celery_app
from pipeline.db import SessionLocal


@dataclass
class StepOutcome:
    num_points: int | None = None
    source_format: str | None = None
    crs_epsg: int | None = None
    bbox_ewkt: str | None = None
    rtk_fixed: bool | None = None
    assets: list[tuple[AssetType, str, int]] = field(default_factory=list)


def intermediate_key(scan_id: str, step: PipelineStep) -> str:
    return f"{scan_id}/intermediate/{step.value}.laz"


def corrected_trajectory_key(scan_id: str) -> str:
    return f"{scan_id}/intermediate/trajectory_corrected.pos"


def bag_rtk_key(scan_id: str) -> str:
    return f"{scan_id}/intermediate/bag_rtk.pos"


def georef_transform_key(scan_id: str) -> str:
    return f"{scan_id}/intermediate/georef_transform.npz"


def build_pipeline(scan_id: str) -> Any:
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


def _save_inputs(
    storage: Any,
    raw_bucket: str,
    scan_id: str,
    extra: list[tuple[ScanInputKind, str, int]],
) -> None:
    with SessionLocal() as session:
        for kind, path, size in extra:
            existing = session.execute(
                select(ScanInput).where(
                    ScanInput.scan_id == uuid.UUID(scan_id),
                    ScanInput.kind == kind,
                )
            ).scalar_one_or_none()
            if existing:
                existing.storage_path = path
                existing.file_size = size
            else:
                session.add(
                    ScanInput(
                        id=uuid.uuid4(),
                        scan_id=uuid.UUID(scan_id),
                        kind=kind,
                        storage_path=path,
                        file_size=size,
                    )
                )
        session.commit()


def _execute_step(scan_id: str, step: PipelineStep) -> StepOutcome:
    settings = get_settings()
    storage = get_storage()
    raw_bucket = settings.s3_bucket_raw

    with SessionLocal() as session:
        scan = session.get_one(Scan, uuid.UUID(scan_id))
        raw_path = scan.raw_file_path or ""
        checksum = scan.checksum_sha256
        use_bag = bool(getattr(scan, "bag_lidar_enabled", False))
        target_crs_epsg = scan.project.target_crs_epsg
        target_crs_wkt = scan.project.target_crs_wkt

    workdir = Path(tempfile.mkdtemp(prefix=f"slam-{step.value}-"))
    try:
        # -------------------------------------------------------------------
        if step is PipelineStep.COMPUTE_SLAM:
            # Run the Voxel-SLAM LiDAR-inertial kernel over the bag to produce
            # a trajectory (frame_pose) + world-frame cloud, replacing reliance
            # on the vendor's on-device SLAM. No-op for non-bag scans (the
            # standard PCD path already ships a good cloud).
            if not use_bag:
                return StepOutcome()
            src_bucket, src_key = parse_storage_path(raw_path)
            local_raw = workdir / Path(src_key).name
            storage.download_file(src_bucket, src_key, local_raw)

            # Extract the main .bag + calibration.yaml from the ZIP and stage
            # them in S3 as stable keys the voxelslam worker can fetch.
            bag_local = workdir / "scan.bag"
            cal_local = workdir / "calibration.yaml"
            cal_key = ""
            with zipfile.ZipFile(local_raw) as zf:
                bag_candidates = [
                    n for n in zf.namelist()
                    if n.endswith(".bag") and not n.startswith("__MACOSX")
                ]
                main_bag = max(bag_candidates, key=lambda n: zf.getinfo(n).file_size, default=None)
                if main_bag is None:
                    return StepOutcome()  # no bag inside; nothing to compute
                with zf.open(main_bag) as src, open(bag_local, "wb") as dst:
                    shutil.copyfileobj(src, dst)
                cal_name = next((n for n in zf.namelist() if Path(n).name == "calibration.yaml"), None)
                if cal_name is not None:
                    with zf.open(cal_name) as src, open(cal_local, "wb") as dst:
                        dst.write(src.read())

            bag_key = f"{scan_id}/inputs/voxelslam_scan.bag"
            storage.upload_file(raw_bucket, bag_key, bag_local)
            if cal_local.exists():
                cal_key = f"{scan_id}/inputs/calibration.yaml"
                storage.upload_file(raw_bucket, cal_key, cal_local)

            fp_key = f"{scan_id}/inputs/frame_pose.txt"
            cloud_key = intermediate_key(scan_id, PipelineStep.COMPUTE_SLAM)
            # Best-effort (like PPK_CORRECTION): if the SLAM kernel times out or
            # diverges (e.g. degenerate narrow-corridor geometry), don't fail
            # the scan -- skip and let DECODE_RAW fall back to the vendor's
            # on-device frame_pose.txt from the ZIP.
            try:
                result = celery_app.send_task(
                    "voxelslam.process",
                    args=[bag_key, cal_key, fp_key, cloud_key, "scan"],
                    queue="voxelslam",
                ).get(timeout=7200)
                points = result.get("points") if isinstance(result, dict) else None
            except Exception:
                return StepOutcome()

            # Register the SLAM trajectory as the scan's FRAME_POSE input so
            # DECODE_RAW / COLORIZE / GEOREFERENCE pick it up in place of the
            # vendor's frame_pose.txt (_save_inputs upserts).
            _save_inputs(storage, raw_bucket, scan_id, [
                (ScanInputKind.FRAME_POSE, f"s3://{raw_bucket}/{fp_key}", 0),
            ])
            return StepOutcome(num_points=points)

        # -------------------------------------------------------------------
        if step is PipelineStep.DECODE_RAW:
            src_bucket, src_key = parse_storage_path(raw_path)
            local_raw = workdir / Path(src_key).name
            storage.download_file(src_bucket, src_key, local_raw)
            if checksum:
                processing.verify_checksum(local_raw, checksum)

            local_out = workdir / "decoded.laz"
            extra_inputs: list[tuple[ScanInputKind, str, int]] = []

            if not use_bag:
                # ---- PCD path: standard decode from PCD folders in ZIP ----
                decoded = processing.decode_to_laz(local_raw, local_out, source_name=src_key)
                storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
                return StepOutcome(
                    num_points=decoded.num_points,
                    source_format=decoded.source_format,
                    crs_epsg=decoded.crs_epsg,
                )

            # ---- BAG path: extract LiDAR + camera frames from ROS1 bag ZIP ----

            # Extract frame_pose.txt and .bag file from the ZIP
            bag_extract_dir = workdir / "bag_extracted"
            bag_extract_dir.mkdir(exist_ok=True)
            bag_path_in: Path | None = None
            frame_pose_bag: s20.FramePose | None = None
            fp_local: Path | None = None
            fp_key: str | None = None

            # Prefer the Voxel-SLAM trajectory from COMPUTE_SLAM if it produced
            # one; only fall back to the vendor's frame_pose.txt in the ZIP when
            # COMPUTE_SLAM was skipped (non-bag, or the kernel diverged).
            _existing_inputs = _load_inputs(scan_id)
            slam_fp_used = False
            if ScanInputKind.FRAME_POSE in _existing_inputs:
                try:
                    slam_fp_path = _download_input(
                        storage, _existing_inputs, ScanInputKind.FRAME_POSE, workdir
                    )
                    frame_pose_bag = s20.read_frame_pose(slam_fp_path)
                    slam_fp_used = True
                except Exception:
                    slam_fp_used = False

            try:
                with zipfile.ZipFile(local_raw) as zf:
                    # A ZIP may contain more than one .bag (e.g. the main
                    # recording plus a small info/*.bag metadata sidecar) --
                    # the real data bag is always the largest one.
                    bag_candidates = [
                        n for n in zf.namelist()
                        if n.endswith(".bag") and not n.startswith("__MACOSX")
                    ]
                    main_bag_name = max(
                        bag_candidates, key=lambda n: zf.getinfo(n).file_size, default=None
                    )
                    for name in zf.namelist():
                        pname = Path(name)
                        if pname.name == "frame_pose.txt" and not slam_fp_used:
                            # Only used when COMPUTE_SLAM didn't supply a
                            # trajectory (see slam_fp_used above).
                            fp_local = workdir / "frame_pose.txt"
                            with zf.open(name) as src, open(fp_local, "wb") as dst:
                                dst.write(src.read())
                            frame_pose_bag = s20.read_frame_pose(fp_local)
                            fp_key = f"{scan_id}/inputs/frame_pose.txt"
                            storage.upload_file(raw_bucket, fp_key, fp_local)
                            extra_inputs.append((
                                ScanInputKind.FRAME_POSE,
                                f"s3://{raw_bucket}/{fp_key}",
                                fp_local.stat().st_size,
                            ))
                        elif name == main_bag_name:
                            bag_local = bag_extract_dir / pname.name
                            with zf.open(name) as src, open(bag_local, "wb") as dst:
                                while chunk := src.read(8 * 1024 * 1024):
                                    dst.write(chunk)
                            bag_path_in = bag_local
            except Exception as exc:
                raise processing.ProcessingError(
                    f"Failed to extract BAG ZIP contents: {exc}"
                ) from exc

            if bag_path_in is None:
                raise processing.ProcessingError("No .bag file found in uploaded ZIP")

            # Extract camera frames (best-effort) -- done before bag_lidar_to_laz
            # so the visual attitude correction below (which needs them) can run
            # first, and the corrected frame_pose is what actually gets applied
            # to the LiDAR points.
            cam_zip: Path | None = None
            try:
                cam_dir = workdir / "camera_frames"
                cam_dir.mkdir(exist_ok=True)
                n_cam = s20.extract_camera_frames_from_bag(bag_path_in, cam_dir)
                if n_cam > 0:
                    cam_zip = workdir / "camera_frames.zip"
                    with zipfile.ZipFile(cam_zip, "w", zipfile.ZIP_STORED) as zf:
                        for img in sorted(cam_dir.iterdir()):
                            zf.write(img, img.name)
                    cam_key = f"{scan_id}/inputs/camera_frames.zip"
                    storage.upload_file(raw_bucket, cam_key, cam_zip)
                    extra_inputs.append((
                        ScanInputKind.CAMERA_FRAMES,
                        f"s3://{raw_bucket}/{cam_key}",
                        cam_zip.stat().st_size,
                    ))
            except Exception:
                cam_zip = None

            # Extract calibration.yaml (best-effort)
            cal_local: Path | None = None
            try:
                with zipfile.ZipFile(local_raw) as zf:
                    for name in zf.namelist():
                        if Path(name).name == "calibration.yaml":
                            cal_local = workdir / "calibration.yaml"
                            with zf.open(name) as src, open(cal_local, "wb") as dst:
                                dst.write(src.read())
                            cal_key = f"{scan_id}/inputs/calibration.yaml"
                            storage.upload_file(raw_bucket, cal_key, cal_local)
                            extra_inputs.append((
                                ScanInputKind.CALIBRATION,
                                f"s3://{raw_bucket}/{cal_key}",
                                cal_local.stat().st_size,
                            ))
                            break
            except Exception:
                cal_local = None

            # Trajectory computation (best-effort, kiss-icp): recompute the
            # sensor trajectory from raw LiDAR geometry alone rather than
            # trusting the vendor's on-device SLAM frame_pose.txt, whose
            # accumulated roll/pitch/yaw drift is what causes
            # range-dependent geometry warping downstream (RTK position
            # alone can't observe/correct attitude -- see
            # georeference_from_slam's docstring). Falls back to the
            # vendor's own frame_pose_bag unchanged if this fails for any
            # reason (e.g. very short recording, missing LiDAR topic),
            # since it's a refinement, not a hard requirement.
            #
            # This replaces two earlier, both-abandoned attempts at the
            # same goal (see session notes):
            #   - A hand-rolled ICP loop-closure corrector
            #     (_collect_lidar_loop_closure_observations, still present,
            #     unused): converged to wrong local minima on this device's
            #     real geometry, made results worse (13.3m -> 27.2m
            #     roughness).
            #   - A full ROS2 FAST-LIO2 integration (separate
            #     worker-fastlio service, since removed): built and ran
            #     end-to-end, but its IMU-coupled EKF diverged to
            #     multi-km trajectories within ~10-30s of real motion for
            #     reasons that survived exhaustive sign/axis-permutation
            #     testing -- root cause never identified.
            #
            # kiss-icp (PRBonn, pip-installable, no ROS/IMU coupling)
            # produces a bounded, physically plausible trajectory on its
            # own; compute_kiss_icp_leveled_trajectory additionally levels
            # it against the point cloud's own ground plane, since pure
            # LiDAR odometry has no gravity reference and its "up" axis can
            # slowly drift over a multi-minute recording. Validated on scan
            # e6b4bbe7 (median cell Z-range roughness): 13.3m
            # (vision-only-corrected vendor trajectory) -> 9.6m (kiss-icp +
            # leveling) -- the best result found this session, though still
            # well short of the vendor's own reference pipeline (0.42m).
            # Only recompute the trajectory with kiss-icp when COMPUTE_SLAM did
            # NOT already supply a (better) Voxel-SLAM one -- otherwise this
            # would clobber the Voxel-SLAM poses used for the LiDAR->LAZ build
            # while COLORIZE/GEOREFERENCE keep reading the Voxel-SLAM frame_pose
            # from S3, producing an inconsistent cloud.
            if frame_pose_bag is not None and not slam_fp_used:
                try:
                    kiss_fp = s20.compute_kiss_icp_leveled_trajectory(bag_path_in, workdir)
                    frame_pose_bag = kiss_fp
                    if fp_local is not None and fp_key is not None:
                        s20.write_frame_pose(frame_pose_bag, fp_local)
                        storage.upload_file(raw_bucket, fp_key, fp_local)
                except Exception:
                    pass

            # Convert LiDAR from bag to LAZ, applying frame_pose for world_slam coords
            n_pts = s20.bag_lidar_to_laz(bag_path_in, local_out, frame_pose_bag)
            storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)

            # Extract RTK trajectory from bag (for georeferencing in later step)
            try:
                rtk_local = workdir / "bag_rtk.pos"
                n_rtk = s20.bag_to_rtk_pos(bag_path_in, rtk_local)
                if n_rtk > 0:
                    storage.upload_file(raw_bucket, bag_rtk_key(scan_id), rtk_local)
            except Exception:
                pass

            if extra_inputs:
                _save_inputs(storage, raw_bucket, scan_id, extra_inputs)

            return StepOutcome(num_points=n_pts, source_format="bag")

        # -------------------------------------------------------------------
        if step is PipelineStep.FILTER_OUTLIERS:
            local_in = workdir / "in.laz"
            local_out = workdir / "filtered.laz"
            storage.download_file(
                raw_bucket, intermediate_key(scan_id, PipelineStep.DECODE_RAW), local_in
            )
            result = processing.filter_outliers(local_in, local_out)
            storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
            return StepOutcome(num_points=result.points_out)

        # -------------------------------------------------------------------
        if step is PipelineStep.BIN_TO_RINEX:
            inputs = _load_inputs(scan_id)
            if ScanInputKind.ROVER_PPKRAW_BIN not in inputs and ScanInputKind.BASE_BIN not in inputs:
                return StepOutcome()
            return StepOutcome()

        # -------------------------------------------------------------------
        if step is PipelineStep.PPK_CORRECTION:
            inputs = _load_inputs(scan_id)
            # PPK correction is optional, best-effort refinement -- georeference
            # already falls back cleanly when it's unavailable (BAG path uses
            # the bag's own onboard RTK; PCD path falls through to the raw,
            # uncorrected trajectory). It genuinely needs BOTH rover and base
            # observations to produce anything meaningful, so any missing
            # piece (base_rinex, rover_obs, the original trajectory, or the
            # rnx2rtkp binary itself) skips this step rather than failing the
            # whole pipeline chain over an optional refinement.
            missing = [
                required.value
                for required in (
                    ScanInputKind.BASE_RINEX,
                    ScanInputKind.ROVER_OBS,
                    ScanInputKind.TRAJECTORY,
                )
                if required not in inputs
            ]
            if missing or not gnss.rnx2rtkp_available():
                return StepOutcome()
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

        # -------------------------------------------------------------------
        # The real, deployed PIPELINE_ORDER (site-packages app/models.py --
        # the copy actually resolved by the celery worker's default import
        # path; /srv/backend/app/models.py is a different, non-authoritative
        # copy) runs COLORIZE before GEOREFERENCE. colorize_laz's camera-
        # projection math needs points in the local SLAM frame frame_pose was
        # recorded in, so it must run on FILTER_OUTLIERS' still-local-frame
        # output, before georeferencing moves points into UTM.
        if step is PipelineStep.COLORIZE:
            inputs = _load_inputs(scan_id)
            local_in = workdir / "filtered.laz"
            storage.download_file(
                raw_bucket, intermediate_key(scan_id, PipelineStep.FILTER_OUTLIERS), local_in
            )
            if ScanInputKind.CAMERA_FRAMES not in inputs:
                # PCD path: point cloud already carries RGB from SLAM output; pass-through.
                storage.copy_object(
                    raw_bucket,
                    intermediate_key(scan_id, PipelineStep.FILTER_OUTLIERS),
                    raw_bucket,
                    intermediate_key(scan_id, step),
                )
                return StepOutcome()
            # BAG path: project nav-cam frames onto LiDAR points in world_slam frame.
            local_out = workdir / "coloured.laz"
            fp_path = _download_input(storage, inputs, ScanInputKind.FRAME_POSE, workdir)
            frame_pose = s20.read_frame_pose(fp_path)
            cam_zip = _download_input(storage, inputs, ScanInputKind.CAMERA_FRAMES, workdir)
            cal_path = None
            if ScanInputKind.CALIBRATION in inputs:
                cal_path = _download_input(storage, inputs, ScanInputKind.CALIBRATION, workdir)
            s20.colorize_laz(local_in, cam_zip, frame_pose, local_out, cal_path)
            storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
            return StepOutcome()

        # -------------------------------------------------------------------
        if step is PipelineStep.GEOREFERENCE:
            local_in = workdir / "in.laz"
            storage.download_file(
                raw_bucket,
                intermediate_key(scan_id, PipelineStep.COLORIZE),
                local_in,
            )

            inputs = _load_inputs(scan_id)
            corrected_key = corrected_trajectory_key(scan_id)
            rtk_key = bag_rtk_key(scan_id)

            # BAG path: georeference using RTK trajectory extracted from bag
            if use_bag and ScanInputKind.FRAME_POSE in inputs and storage.object_exists(raw_bucket, rtk_key):
                fp_path = _download_input(storage, inputs, ScanInputKind.FRAME_POSE, workdir)
                frame_pose = s20.read_frame_pose(fp_path)
                rtk_pos = workdir / "bag_rtk.pos"
                storage.download_file(raw_bucket, rtk_key, rtk_pos)
                local_out = workdir / "georef.laz"
                s20.georeference_from_slam(
                    local_in, frame_pose, rtk_pos, local_out,
                    target_crs_epsg=target_crs_epsg, target_crs_wkt=target_crs_wkt,
                )
                storage.upload_file(raw_bucket, intermediate_key(scan_id, step), local_out)
                bbox = processing.wgs84_bbox(local_out)

            # PPK path: shift already-georeferenced cloud by trajectory correction
            elif (
                ScanInputKind.TRAJECTORY in inputs
                and storage.object_exists(raw_bucket, corrected_key)
            ):
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
                # No georeferencing data — pass-through
                storage.copy_object(
                    raw_bucket,
                    intermediate_key(scan_id, PipelineStep.COLORIZE),
                    raw_bucket,
                    intermediate_key(scan_id, step),
                )
                local_out = local_in
                bbox = processing.wgs84_bbox(local_in)

            return StepOutcome(
                bbox_ewkt=processing.bbox_polygon_ewkt(bbox) if bbox else None,
                crs_epsg=processing.crs_epsg(local_out) if bbox else None,
            )

        # -------------------------------------------------------------------
        if step is PipelineStep.BUILD_OCTREE:
            local_in = workdir / "final.laz"
            storage.download_file(
                raw_bucket, intermediate_key(scan_id, PipelineStep.GEOREFERENCE), local_in
            )
            assets: list[tuple[AssetType, str, int]] = []

            las_key = f"{scan_id}/pointcloud.laz"
            storage.upload_file(settings.s3_bucket_processed, las_key, local_in)
            assets.append((
                AssetType.LAS,
                f"s3://{settings.s3_bucket_processed}/{las_key}",
                local_in.stat().st_size,
            ))

            if processing.pdal_available():
                local_copc = workdir / "pointcloud.copc.laz"
                processing.build_copc(local_in, local_copc)
                copc_key = f"{scan_id}/pointcloud.copc.laz"
                storage.upload_file(settings.s3_bucket_processed, copc_key, local_copc)
                assets.append((
                    AssetType.COPC,
                    f"s3://{settings.s3_bucket_processed}/{copc_key}",
                    local_copc.stat().st_size,
                ))
            return StepOutcome(assets=assets)

        raise processing.ProcessingError(f"unknown step: {step}")
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
