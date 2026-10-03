#!/usr/bin/env bash
# Usage: container/run.sh [command...]   (default: interactive bash)
# Builds the image if needed and mounts the workspace at /ws.
#
# Environment:
#   CONTAINER_ENGINE  podman or docker (default: podman if installed, else docker)
#   OUMUAMUA_IMAGE    image tag (default: localhost/oumuamua:lyrical)
#   EXTRA_CA_CERT     CA certificate of a TLS-intercepting proxy, used only during the build
#   REBUILD=1         rebuild the image even if it exists
set -euo pipefail

if [ -n "${CONTAINER_ENGINE:-}" ]; then
  ENGINE="$CONTAINER_ENGINE"
elif command -v podman >/dev/null 2>&1; then
  ENGINE=podman
else
  ENGINE=docker
fi
IMAGE="${OUMUAMUA_IMAGE:-localhost/oumuamua:lyrical}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${REBUILD:-0}" = 1 ] || ! "$ENGINE" image inspect "$IMAGE" >/dev/null 2>&1; then
  BUILD_ARGS=()
  [ -n "${EXTRA_CA_CERT:-}" ] && BUILD_ARGS+=(--secret "id=extra_ca,src=$EXTRA_CA_CERT")
  "$ENGINE" build --network=host "${BUILD_ARGS[@]}" -t "$IMAGE" -f "$WS/container/Containerfile" "$WS"
fi

TTY_ARGS=()
[ -t 0 ] && TTY_ARGS=(-it)

if [ $# -eq 0 ]; then
  set -- bash
fi

# label=disable: let the bind mount work on SELinux hosts without relabeling the checkout.
exec "$ENGINE" run --rm "${TTY_ARGS[@]}" \
  --net=host \
  --security-opt label=disable \
  -v "$WS:/ws" \
  "$IMAGE" bash -ic '"$@"' _ "$@"
