# syntax=docker/dockerfile:1
FROM pytorch/pytorch:2.11.0-cuda12.6-cudnn9-runtime
COPY --from=ghcr.io/astral-sh/uv:0.13.5 /uv /uvx /usr/local/bin/
ENV DEBIAN_FRONTEND=noninteractive \
    UV_NO_MANAGED_PYTHON=1 \
    UV_LINK_MODE=copy \
    PYTHONUNBUFFERED=1 \
    VIRTUAL_ENV=/opt/venv \
    PATH="/opt/venv/bin:${PATH}"
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libgl1 libglib2.0-0 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY docker/lerobot/requirements.in /tmp/requirements.in
COPY docker/lerobot/constraints.txt /tmp/constraints.txt
RUN uv venv --python "$(command -v python)" --system-site-packages "$VIRTUAL_ENV" \
    && uv pip install --python "$VIRTUAL_ENV/bin/python" --constraints /tmp/constraints.txt -r /tmp/requirements.in \
    && uv pip check --python "$VIRTUAL_ENV/bin/python"
WORKDIR /workspace
CMD ["sleep","infinity"]
