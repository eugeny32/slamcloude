# Build context: repository root (see infra/docker-compose.yml)
# NOTE: GPU-intensive steps (Open3D/CUDA) will move this to a CUDA base image
# scheduled on GPU nodes (see infra/k8s/).
FROM python:3.12-slim-bookworm

# RTKLIB: rnx2rtkp PPK solver (rover obs + base RINEX) in the ppk_correction step.
RUN apt-get update \
    && apt-get install -y --no-install-recommends rtklib curl bzip2 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# PDAL: COPC octree generation (writers.copc) in the build_octree step.
# No pdal package in Debian bookworm/trixie — install from conda-forge
# (PDAL's official distribution channel) via micromamba.
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
        | tar -xj -C /tmp bin/micromamba \
    && /tmp/bin/micromamba create -y -r /opt/mamba -p /opt/pdal -c conda-forge pdal \
    && ln -s /opt/pdal/bin/pdal /usr/local/bin/pdal \
    && pdal --version \
    && rm -rf /tmp/bin /opt/mamba/pkgs

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

WORKDIR /srv
COPY backend/ ./backend/
COPY pyproject.toml ./pyproject.toml
COPY worker/ ./worker/
RUN uv pip install --system --no-cache ./backend ./worker

WORKDIR /srv/worker
CMD ["celery", "-A", "pipeline.celery_app:celery_app", "worker", "--loglevel=info"]
