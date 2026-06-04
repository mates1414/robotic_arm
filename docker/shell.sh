#!/usr/bin/env bash
# Open an additional shell in the already-running container.
# Useful because the workflow needs several terminals (sim, apriltag, task).
set -e
cd "$(dirname "$0")"
xhost +local:root >/dev/null 2>&1 || true
docker compose exec ur5 bash
