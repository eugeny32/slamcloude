# Build context: repository root (see infra/docker-compose.yml)
FROM python:3.12-slim

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /srv
COPY backend/ ./backend/
RUN uv pip install --system --no-cache ./backend

WORKDIR /srv/backend
EXPOSE 8000
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
