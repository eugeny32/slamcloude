# slamcloude

Облачная платформа обработки данных LiDAR-сканера SHARE S20 (LiDAR + RTK + SLAM).
Поток: chunked upload сырых данных в S3/MinIO → Celery-пайплайн
(decode → filter → georeference → colorize → octree/COPC) → LAS/E57/PLY +
потоковый веб-просмотр (Potree). Пользователи: геодезисты, BIM, строители.

## Стек и структура

- Монорепозиторий, uv workspace: `backend/` (FastAPI, SQLAlchemy 2 async +
  GeoAlchemy2, Alembic, boto3, celery-клиент), `worker/` (Celery + Redis,
  пакет `pipeline`, импортирует модели из `app`), `frontend/` (React + Potree,
  заготовка), `infra/` (docker-compose, k8s-заготовки).
- Python 3.12+, строгая типизация (mypy strict), pydantic-схемы для API.
- БД: PostgreSQL + PostGIS. Enum-ы — non-native (VARCHAR, валидация на уровне приложения).

## Команды

- Всё окружение: `uv sync --all-packages`
- Тесты: `uv run pytest` (backend/tests + worker/tests)
- Линт/типы: `uv run ruff check .` / `uv run mypy backend worker`
- Локальный стек: `docker compose -f infra/docker-compose.yml up --build`
- Миграции: из `backend/` — `uv run alembic upgrade head`

## Правила проекта

- Большие файлы (50+ ГБ) обрабатывать только стримингом/чанками, не грузить в память.
- Каждый шаг пайплайна — отдельная Celery-задача с записью статуса в `jobs`,
  падение шага не должно требовать пересчёта с нуля.
- Дорожная карта MVP: (1) инфра+скелет ✔ (2) модели+миграции ✔ (3) upload +
  API-ключи + rate limit ✔ (4) Celery-пайплайн сквозного потока ✔ (5) реальная
  обработка облаков точек ✔ — остался frontend (React + Potree) и SHARE
  S20-специфика (см. ниже).
- Пайплайн: chain из immutable-сигнатур `pipeline.step`; шаг идемпотентен
  (completed → skip), падение пишет error в jobs и валит скан; resume —
  `POST /scans/{id}/process` (или авто-enqueue при complete). Backend
  ставит задачи только по имени (`app/services/queue.py`), код воркера
  не импортирует.
- Обработка (`worker/pipeline/processing.py` — чистые функции над локальными
  файлами, unit-тесты на синтетических LAS): decode LAS/LAZ→LAZ (chunked,
  проверка sha256 стримингом), SOR-фильтр выбросов (scipy cKDTree, по чанкам),
  георепроекция bbox в WGS84 (pyproj) → scans.bbox, COPC через PDAL
  (subprocess; бинарь есть в Docker-образе воркера, локально шаг даёт только LAS).
  Промежуточные артефакты шагов — в raw-бакете `{scan_id}/intermediate/<step>.laz`.
- PPK GNSS (`worker/pipeline/gnss.py`): шаг `ppk_correction` (между filter и
  georeference) решает rover_obs+base_rinex через RTKLIB rnx2rtkp (subprocess,
  бинарь в Docker-образе), кладёт corrected .pos в интермедиаты; georeference
  сдвигает облако по дельте траекторий, интерполированной на gps_time точек.
  Вспомогательные файлы — таблица scan_inputs (kind: trajectory/rover_obs/
  base_rinex/nav), загрузка `PUT /scans/{id}/inputs/{kind}`. Пересчёт без
  повторной распаковки — `POST /scans/{id}/reprocess {from_step}`: сброс jobs
  от шага, decode/filter скипаются (интермедиаты в S3), assets версионируются.
- Ждёт вендорских спецификаций SHARE S20: декодер сырого потока сканера
  (сейчас UnsupportedFormatError для не-LAS/LAZ), нормализация gps_time точек
  к эпохе траектории в decode, colorization по камерам (шаг pass-through).
- `GET /scans/{id}/download` → 307 на presigned URL (S3_PUBLIC_ENDPOINT_URL для
  браузеров вне compose-сети); `preview` отдаёт copc_url + bbox + num_points.
- Auth: `X-API-Key` (sha256-хэш в users.api_key_hash), создание пользователя —
  `python -m app.cli create-user`. Rate limit — Redis fixed window, fail-open.
- E2E-тест загрузки: `backend/tests/test_e2e_upload.py`, gated
  `SLAMCLOUDE_E2E=1` + запущенный compose-стек.
- Бизнес-логику согласовывать с владельцем до реализации (структура/схема — сначала на ревью).

## Особенности среды

- Windows; в песочнице Claude ruff/mypy падают на создании кэша — запускать
  `ruff check --no-cache`, `mypy --cache-dir=nul`.
- Репозиторий лежит в OneDrive-синхронизируемой папке: быстрые серии записей
  дают фантомные ENOENT (`vite build` в dist/, vite.config.ts.timestamp),
  dev-сервер vite зависает на файловом вотчере. Обход: сборка frontend с
  `--outDir "$env:LOCALAPPDATA\Temp\slamcloude-dist" --emptyOutDir`; vite.config
  нет намеренно (нужные настройки = дефолты, конфиг не загружается из-за той же
  проблемы). Запись в node_modules работает (OneDrive его не синхронизирует).
- Docker не установлен — compose-стек и E2E не прогнать локально.
