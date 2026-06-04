#!/usr/bin/env bash
# Container entrypoint: make every shell/command ROS-aware.
set -e

source /opt/ros/noetic/setup.bash

# Source the built workspace if it exists (first run won't have it yet).
if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
    source "${CATKIN_WS}/devel/setup.bash"
fi

# Gazebo needs to find this project's models/materials (AprilTag texture).
export GAZEBO_MODEL_PATH="${CATKIN_WS}/src/ur5_with_robotiq_gripper/icl_ur5_setup_gazebo/worlds:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CATKIN_WS}/src/ur5_with_robotiq_gripper/icl_ur5_setup_description:${GAZEBO_RESOURCE_PATH}"

exec "$@"
