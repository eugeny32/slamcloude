# slamcloude

Облачная платформа обработки данных портативного 3D-сканера **SHARE S20**
(LiDAR + RTK GNSS + Visual SLAM): загрузка сырых сканов → пайплайн обработки
(фильтрация, геопривязка, colorization, octree) → геопривязанное цветное облако
точек в LAS/LAZ/E57 + потоковый веб-просмотр.

## Структура монорепозитория

```
backend/    FastAPI-сервис: API, модели БД (SQLAlchemy + GeoAlchemy2), Alembic-миграции
worker/     Celery-воркер: пайплайн обработки облаков точек (PDAL, RTKLIB)
frontend/   React + three.js/copc: стриминговый 3D-просмотр COPC, загрузка, PPK-панель
infra/      docker-compose для локальной разработки, k8s-манифесты (заготовка)
```

## Запуск локального стека одной командой

```powershell
docker compose -f infra/docker-compose.yml up --build
```

Поднимает: Postgres+PostGIS, Redis, MinIO (+ создание бакетов), применяет
Alembic-миграции, backend на http://localhost:8000 (Swagger: `/docs`),
Celery-воркер, frontend на http://localhost:3000.
Консоль MinIO: http://localhost:9001 (minioadmin/minioadmin).

Масштабирование воркеров: `docker compose -f infra/docker-compose.yml up --scale worker=4`.

## Разработка без Docker (только Python-часть)

```powershell
uv sync --all-packages          # окружение для backend + worker + dev-инструменты
uv run pytest                   # тесты (backend + worker)
uv run ruff check .             # линтер
uv run mypy backend worker      # типы
```

Миграции (нужен запущенный Postgres из compose):

```powershell
cd backend
uv run alembic upgrade head
uv run alembic revision --autogenerate -m "..."
```

## Работа с API (поток загрузки)

Авторизация — заголовок `X-API-Key`. Создать пользователя и получить ключ
(нужен запущенный Postgres):

```powershell
cd backend
uv run python -m app.cli create-user --email you@example.com --plan pro
```

Поток загрузки скана (chunked, S3 multipart, части по 64 МиБ):

1. `POST /projects` `{"name": "..."}` → `project_id`
2. `POST /scans/upload` `{"project_id", "filename", "file_size", "checksum_sha256"?}`
   → `scan_id`, `upload_id`, `part_size`, `num_parts`
3. Для каждой части: `PUT /scans/{scan_id}/upload/parts/{n}?upload_id=...`
   (тело — сырые байты; опциональный заголовок `X-Part-SHA256` для проверки
   целостности) → `etag`
4. `POST /scans/{scan_id}/upload/complete` `{"upload_id", "parts": [{part_number, etag}]}`
   → статус `uploaded`, создаются 5 записей `jobs` и пайплайн ставится в очередь
5. `GET /scans/{scan_id}/status` — прогресс по шагам пайплайна + готовые assets
6. `POST /scans/{scan_id}/process` — перезапуск пайплайна (после падения шага
   возобновляется с упавшего места, завершённые шаги пропускаются)
7. `GET /projects/{id}/scans?bbox=minLon,minLat,maxLon,maxLat` — геопоиск

Воркер локально без Docker (Windows): `cd worker; uv run celery -A
pipeline.celery_app:celery_app worker --loglevel=info --pool=solo`.

E2E-тест потока (при поднятом compose-стеке):

```powershell
$env:SLAMCLOUDE_E2E="1"; $env:SLAMCLOUDE_E2E_API_KEY="sk_..."
uv run pytest backend/tests/test_e2e_upload.py
```

## Пайплайн обработки

| Шаг | Что делает | Статус |
|---|---|---|
| decode_raw | скачивание из S3, sha256-проверка, LAS/LAZ → нормализованный LAZ (chunked) | реальный |
| filter_outliers | statistical outlier removal (kNN, scipy cKDTree, по чанкам) | реальный |
| ppk_correction | PPK: rover obs + базовый RINEX → RTKLIB `rnx2rtkp` → скорректированная траектория (.pos); `rtk_fixed` по доле Q=1 | реальный (RTKLIB — в Docker-образе воркера) |
| georeference | сдвиг облака по дельте траекторий (интерполяция по `gps_time` точек), bbox → EPSG:4326 → `scans.bbox` | реальный |
| colorize | окраска по фото двух камер | pass-through (ждёт спецификацию камер S20) |
| build_octree | публикация LAZ-результата + COPC-octree через PDAL | реальный (COPC — в Docker-образе воркера) |

### PPK-коррекция от своей базовой станции

1. Загрузить вспомогательные файлы скана (сырое тело запроса):
   `PUT /scans/{id}/inputs/trajectory?filename=track.pos` — PPK-траектория из
   исходных данных S20; `PUT /scans/{id}/inputs/rover_obs` — сырые GNSS-наблюдения
   ровера (RINEX); `PUT /scans/{id}/inputs/base_rinex` — базовая станция (RINEX);
   опционально `PUT /scans/{id}/inputs/nav` — эфемериды. Список: `GET /scans/{id}/inputs`.
2. `POST /scans/{id}/reprocess` `{"from_step": "ppk_correction"}` — пересчёт
   **без повторной распаковки**: decode/filter не выполняются (их LAZ-промежуточные
   результаты уже в S3), выполняются только PPK → georeference → colorize → octree.
   Результат — новая версия assets (download/preview отдают последнюю).

Без базового RINEX шаг ppk_correction — no-op (облако георефенцируется по CRS файла).

Декодер проприетарного потока SHARE S20 подключается в `decode_to_laz`
(`worker/pipeline/processing.py`), когда будет вендорский SDK/спецификация;
до тех пор поддерживаются LAS/LAZ.

## Схема БД

`users` → `projects` → `scans` → (`jobs`, `processed_assets`).
`scans.bbox` — geometry(POLYGON, 4326) с GiST-индексом для геопоиска
(«какие сканы пересекают область»). Статусы пайплайна — в `jobs`
(по записи на шаг: decode_raw → filter_outliers → georeference → colorize →
build_octree), чтобы падение шага не требовало пересчёта с нуля.
