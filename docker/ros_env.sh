# ROS environment inside the container: sourced by interactive shells and by `run.sh <command>`.
source /opt/ros/noetic/setup.bash
if [ -n "${UR5_WS}" ] && [ -f "${UR5_WS}/devel/setup.bash" ]; then
  source "${UR5_WS}/devel/setup.bash"
fi
