#!/usr/bin/env bash
# Build (if needed), start the container, and drop into a shell.
set -e
cd "$(dirname "$0")"

# Let local containers talk to the host X server (Gazebo + RViz windows).
xhost +local:root >/dev/null 2>&1 || \
    echo "!! 'xhost' not found or X not running — GUI apps may fail to open."

# Match the container user to your host UID/GID so mounted files stay yours.
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"

docker compose up -d --build

echo
echo "Container 'ur5_noetic' is running. Entering a shell..."
echo "First time? Build the workspace with:  build_ws.sh"
echo
docker compose exec ur5 bash
