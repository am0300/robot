#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
: "${DISPLAY:?GUI session required}"
umask 077
rm -f .docker.xauth
touch .docker.xauth
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f .docker.xauth nmerge -
test -s .docker.xauth || { echo 'Xauthority not available'; exit 1; }
docker compose up -d --build --force-recreate
