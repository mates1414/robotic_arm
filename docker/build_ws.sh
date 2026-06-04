#!/usr/bin/env bash
# Resolve dependencies and build the catkin workspace.
# Run this once inside the container after the first `up`, and again whenever
# you add packages or change package.xml dependencies.
set -e

source /opt/ros/noetic/setup.bash
cd "${CATKIN_WS}"

echo ">>> Refreshing rosdep cache (Noetic is EOL -> must include EOL distros)..."
rosdep update --include-eol-distros --rosdistro noetic || true

echo ">>> Refreshing apt index (image strips /var/lib/apt/lists to stay small)..."
sudo apt-get update -qq || true

echo ">>> Resolving dependencies with rosdep (reads each package.xml)..."
rosdep install --from-paths src --ignore-src -r -y || \
    echo "!! rosdep reported missing deps; continuing — fix package.xml if the build fails."

echo ">>> Building with catkin_make..."
catkin_make

echo ">>> Done. Open a new shell (devel/setup.bash is auto-sourced) or run:"
echo "    source ${CATKIN_WS}/devel/setup.bash"
