#!/usr/bin/env bash
# Usage: docker/run.sh [command...]   (default: interactive bash)
# Builds the image if needed and mounts the workspace at /ws.
# Set CONTAINER_ENGINE=podman to use Podman instead of Docker.
set -euo pipefail

ENGINE="${CONTAINER_ENGINE:-docker}"
IMAGE="${OUMUAMUA_IMAGE:-oumuamua:lyrical}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! "$ENGINE" image inspect "$IMAGE" >/dev/null 2>&1; then
  "$ENGINE" build -t "$IMAGE" -f "$WS/docker/Dockerfile" "$WS"
fi

TTY_ARGS=()
[ -t 0 ] && TTY_ARGS=(-it)

if [ $# -eq 0 ]; then
  set -- bash
fi

exec "$ENGINE" run --rm "${TTY_ARGS[@]}" \
  --net=host \
  -v "$WS:/ws" \
  "$IMAGE" bash -ic '"$@"' _ "$@"
