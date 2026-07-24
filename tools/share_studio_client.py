"""HTTP client for SHARE PointClouds Studio's local processing engine (SAE / core.exe).

Reverse-engineered (legitimate: extracted the Electron app's own app.asar with
`asar extract`, then read the resulting plain JS -- no DRM/licensing bypass
involved) from SHARE PointClouds Studio 2.5.0/2.5.1. The desktop app is just a
thin Electron UI; all real processing happens in a local HTTP server
(`core.exe`, internally called "SAE" / ShareAlgoEngine) that the UI drives over
`http://localhost:{port}`. This module talks to that same local server
directly, so it only works ON THE MACHINE where SHARE PointClouds Studio is
installed and running (confirmed port: 8000, logged as
"后处理引擎启动: success Port:8000" in
`%APPDATA%\\share-pointclouds-studio\\logs\\<date>.log`).

Endpoint confirmed via app.asar's `V(e)` client factory: task submission goes
through the `/api/`-prefixed client (`POST /api/task/mapping`), not the bare
one. An `x-signature` header (MD5(MD5(json)) reversed) is added by a
`beforeRequest` hook, but ONLY for URLs ending exactly in `/api/task` -- it
does not fire for `/api/task/mapping`, so no signing is needed here.

Task body schema below matches an actual live submission captured from the
app's own per-project log while reprocessing scan
`2026-07-04_04-12-33_PointCloud` on 2026-07-21 (SAE 1.7.0-Beta.5):

    {"deviceModel":"SHARE SLAM S20","lidarModel":"MID-360",
     "dataPath":"D:/.../all_....bag","outputPath":"D:\\...\\output",
     "tempOutputPath":"D:\\.cache\\temp_XXXX","logPath":"D:\\...\\logs",
     "taskType":"mapping","coloration":true,"dynamicRemoval":true,
     "undistort":true,"enableSFM":false,"scene":1,"upsampling":false,
     "upsamplingInterval":10,"saveFileType":["LAS"],"keepScanFiles":false,
     "coordTransformParams":{"mode":"standard","source":"EPSG:7683",
       "horizontal_crs":"EPSG:21015","vertical_crs":"","elevationOffset":0,
       "innerRTK":true,"sourceHeightType":"ellipsoidal","posFilePath":"",
       "height":""}}

CAUTION: submitting a task starts genuine, resource-heavy processing (the
engine's own memory estimator asked for ~31GB RAM / 7GB VRAM for a 39-minute
S20 recording) and will contend with any job already running in the desktop
app. Only one task should run at a time. `submit_mapping_task` therefore
defaults to a dry run; pass `execute=True` deliberately to actually fire it.
"""

import argparse
import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_PORT = 8000


class ShareEngineError(RuntimeError):
    pass


@dataclass
class CoordTransformParams:
    source: str
    horizontal_crs: str
    mode: str = "standard"
    vertical_crs: str = ""
    elevation_offset: float = 0
    inner_rtk: bool = True
    source_height_type: str = "ellipsoidal"
    pos_file_path: str = ""
    height: str = ""

    def to_json(self) -> dict:
        return {
            "mode": self.mode,
            "source": self.source,
            "horizontal_crs": self.horizontal_crs,
            "vertical_crs": self.vertical_crs,
            "elevationOffset": self.elevation_offset,
            "innerRTK": self.inner_rtk,
            "sourceHeightType": self.source_height_type,
            "posFilePath": self.pos_file_path,
            "height": self.height,
        }


@dataclass
class MappingTaskParams:
    """Mirrors the S20-mapping task body the desktop app itself sends."""

    data_path: str
    output_path: str
    temp_output_path: str
    log_path: str
    coord_transform: CoordTransformParams
    device_model: str = "SHARE SLAM S20"
    lidar_model: str = "MID-360"
    coloration: bool = True
    dynamic_removal: bool = True
    undistort: bool = True
    enable_sfm: bool = False
    scene: int = 1
    upsampling: bool = False
    upsampling_interval: int = 10
    save_file_type: list = field(default_factory=lambda: ["LAS"])
    keep_scan_files: bool = False

    def to_json(self) -> dict:
        return {
            "deviceModel": self.device_model,
            "lidarModel": self.lidar_model,
            "dataPath": self.data_path,
            "outputPath": self.output_path,
            "tempOutputPath": self.temp_output_path,
            "logPath": self.log_path,
            "taskType": "mapping",
            "coloration": self.coloration,
            "dynamicRemoval": self.dynamic_removal,
            "undistort": self.undistort,
            "enableSFM": self.enable_sfm,
            "scene": self.scene,
            "upsampling": self.upsampling,
            "upsamplingInterval": self.upsampling_interval,
            "saveFileType": self.save_file_type,
            "keepScanFiles": self.keep_scan_files,
            "coordTransformParams": self.coord_transform.to_json(),
        }


class ShareEngineClient:
    def __init__(self, port: int = DEFAULT_PORT, timeout_s: float = 10.0):
        self.base = f"http://localhost:{port}/api/"
        self.timeout_s = timeout_s

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        url = self.base + path.lstrip("/")
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = urllib.request.Request(url, data=data, method=method)
        if data is not None:
            req.add_header("Content-Type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.URLError as exc:
            raise ShareEngineError(
                f"{method} {url} failed: {exc}. Is SHARE PointClouds Studio "
                "running with the engine started (check port in its logs)?"
            ) from exc

    def submit_mapping_task(self, params: MappingTaskParams, execute: bool = False) -> str:
        """Submit a mapping task. Returns the engine taskId.

        `execute` defaults to False: with it False, this only validates and
        prints the request body without sending it, since a real submission
        triggers genuine multi-GB-RAM processing that must not collide with
        whatever the desktop app itself might be running.
        """
        body = params.to_json()
        if not execute:
            print("[DRY RUN] would POST task/mapping with body:")
            print(json.dumps(body, indent=2, ensure_ascii=False))
            return ""
        result = self._request("POST", "task/mapping", body)
        task_id = result.get("taskId") or result.get("data", {}).get("taskId")
        if not task_id:
            raise ShareEngineError(f"no taskId in response: {result}")
        return task_id

    def poll_mapping_task(self, task_id: str) -> dict:
        """GET task/mapping/{id}. Live-confirmed response shape (2026-07-21,
        SAE 1.7.0-Beta.5, polled against a real in-progress mapping task):

            {"code": 0, "message": "",
             "data": {"uuid": "...", "taskState": 1,
                      "currentStep": 1, "currentStepName": "MAPPING",
                      "totalStep": 5, "progress": 6,
                      "totalDuration": 212, "exitCode": 99}}

        `totalStep=5` matches the 5 per-project log files a full S20 run
        produces (MAPPING -> SLAM -> FILTER -> COLOR -> one more). `progress`
        is 0-100 within the current step, not overall. `taskState` values
        seen so far: 0=queued, 1=running; the terminal value(s) for
        success/failure are not yet confirmed against a real completion --
        `wait_for_completion` below treats anything other than 0/1 as done,
        which is a reasonable guess but should be checked against the first
        real run that finishes while being polled.
        """
        return self._request("GET", f"task/mapping/{task_id}")

    def wait_for_completion(
        self, task_id: str, poll_interval_s: float = 5.0, timeout_s: float = 3600.0
    ) -> dict:
        deadline = time.monotonic() + timeout_s
        last_key = None
        while time.monotonic() < deadline:
            status = self.poll_mapping_task(task_id)
            data = status.get("data", {})
            state = data.get("taskState")
            key = (data.get("currentStepName"), data.get("progress"))
            if key != last_key:
                print(
                    f"  taskId={task_id} step={data.get('currentStepName')} "
                    f"({data.get('currentStep')}/{data.get('totalStep')}) "
                    f"progress={data.get('progress')} taskState={state}"
                )
                last_key = key
            if state not in (0, 1):
                return status
            time.sleep(poll_interval_s)
        raise ShareEngineError(f"timed out waiting for task {task_id} after {timeout_s}s")


def _cli() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_submit = sub.add_parser("submit", help="Submit one mapping task (dry-run unless --execute)")
    p_submit.add_argument("bag_path", help="Path to the .bag file, e.g. D:/0165/6/.../all_....bag")
    p_submit.add_argument("output_dir", help="Output project directory")
    p_submit.add_argument("--source-crs", default="EPSG:7683")
    p_submit.add_argument("--horizontal-crs", default="EPSG:21015")
    p_submit.add_argument("--execute", action="store_true", help="Actually submit (not a dry run)")

    p_poll = sub.add_parser("poll", help="Poll an existing engine taskId once")
    p_poll.add_argument("task_id")

    args = ap.parse_args()
    client = ShareEngineClient(port=args.port)

    if args.cmd == "submit":
        out = Path(args.output_dir)
        params = MappingTaskParams(
            data_path=args.bag_path,
            output_path=str(out / "output"),
            temp_output_path=str(out / "temp"),
            log_path=str(out / "logs"),
            coord_transform=CoordTransformParams(
                source=args.source_crs, horizontal_crs=args.horizontal_crs
            ),
        )
        task_id = client.submit_mapping_task(params, execute=args.execute)
        if task_id:
            print(f"submitted taskId={task_id}")
            client.wait_for_completion(task_id)
    elif args.cmd == "poll":
        print(json.dumps(client.poll_mapping_task(args.task_id), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    _cli()
