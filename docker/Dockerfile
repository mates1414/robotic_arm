# =============================================================================
#  UR5 + Robotiq 2F-85 + AprilTag pick-and-place — ROS Noetic dev container
#
#  ROS Noetic targets Ubuntu 20.04 (Focal). This image lets you run the whole
#  Gazebo/MoveIt/AprilTag stack on a host that is NOT Focal (e.g. Ubuntu 22.04).
#  GUI apps (Gazebo, RViz) render through the host X server; GL is accelerated
#  on Intel/AMD integrated GPUs via /dev/dri (see docker-compose.yml).
# =============================================================================
FROM osrf/ros:noetic-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV CATKIN_WS=/home/${USERNAME}/catkin_ws

# --- System tools + ROS dependencies for this project -----------------------
# desktop-full already provides Gazebo 11, RViz, tf2, image pipeline, etc.
# We add MoveIt, ros_control/controllers, AprilTag, TRAC-IK and the UR driver
# system deps. Anything still missing is resolved by `rosdep` at build time
# (see build_ws.sh) from each package's package.xml.
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo git wget curl nano vim less iputils-ping net-tools \
        build-essential \
        python3-pip python3-rosdep python3-rosinstall \
        python3-catkin-tools python3-osrf-pycommon \
        mesa-utils libgl1-mesa-glx libgl1-mesa-dri \
        ros-noetic-moveit \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-joint-state-controller \
        ros-noetic-joint-trajectory-controller \
        ros-noetic-effort-controllers \
        ros-noetic-position-controllers \
        ros-noetic-velocity-controllers \
        ros-noetic-joint-state-publisher \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-robot-state-publisher \
        ros-noetic-xacro \
        ros-noetic-apriltag \
        ros-noetic-apriltag-ros \
        ros-noetic-trac-ik-kinematics-plugin \
        ros-noetic-ur-client-library \
        ros-noetic-ur-msgs \
    && rm -rf /var/lib/apt/lists/*

# --- Non-root user matching the host UID/GID --------------------------------
# Keeps files created in the mounted workspace (build/, devel/) owned by you.
RUN if getent group ${USER_GID} ; then echo "gid exists" ; else groupadd --gid ${USER_GID} ${USERNAME} ; fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep is already initialized in the base image; refresh the cache.
# --include-eol-distros is REQUIRED: Noetic went EOL (May 2025), so current
# rosdep skips it by default and would resolve nothing.
RUN rosdep update --include-eol-distros --rosdistro noetic || true

# --- gazebo_grasp_fix plugin (no Noetic binary) -----------------------------
# Robotiq parallel grasps are unreliable in Gazebo classic; this plugin attaches
# a grasped object to the gripper when a stable grip is detected. Built into its
# own workspace and put on GAZEBO_PLUGIN_PATH below.
RUN mkdir -p /opt/grasp_ws/src && cd /opt/grasp_ws/src \
    && git clone --depth 1 https://github.com/JenniferBuehler/gazebo-pkgs.git \
    && git clone --depth 1 https://github.com/JenniferBuehler/general-message-pkgs.git \
    && cd /opt/grasp_ws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='gazebo_version_helpers;gazebo_grasp_plugin_msgs;gazebo_grasp_plugin'" \
    && rm -rf /opt/grasp_ws/build /opt/grasp_ws/src/*/.git
ENV GAZEBO_PLUGIN_PATH="/opt/grasp_ws/devel/lib:${GAZEBO_PLUGIN_PATH}"
ENV LD_LIBRARY_PATH="/opt/grasp_ws/devel/lib:${LD_LIBRARY_PATH}"

COPY build_ws.sh /usr/local/bin/build_ws.sh
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/build_ws.sh /usr/local/bin/entrypoint.sh

# --- Per-shell environment --------------------------------------------------
RUN echo "source /opt/ros/noetic/setup.bash"                                    >> /home/${USERNAME}/.bashrc \
    && echo "[ -f ${CATKIN_WS}/devel/setup.bash ] && source ${CATKIN_WS}/devel/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=${CATKIN_WS}/src/ur5_with_robotiq_gripper/icl_ur5_setup_gazebo/worlds:\$GAZEBO_MODEL_PATH" >> /home/${USERNAME}/.bashrc \
    && echo "export GAZEBO_RESOURCE_PATH=${CATKIN_WS}/src/ur5_with_robotiq_gripper/icl_ur5_setup_description:\$GAZEBO_RESOURCE_PATH" >> /home/${USERNAME}/.bashrc \
    && echo "cd ${CATKIN_WS}"                                                   >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR ${CATKIN_WS}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
