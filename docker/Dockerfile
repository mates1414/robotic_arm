# UR5 + Robotiq 2F-85 manipulation: ROS 1 Noetic + Gazebo 11 + MoveIt 1
#
# Everything upstream (UR description/Gazebo/driver, MoveIt, controllers, AprilTag) comes from apt; only this
# project's own packages live in src/ and are built with catkin inside the container.
#
# Noetic is end-of-life (05/2025). The base image points apt at snapshots.ros.org/noetic/final, the frozen final
# state of the Noetic repository, so the package versions below are the last ones ever released.
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Package names verified with apt-cache (2026-10-28).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-moveit \
      ros-noetic-moveit-commander \
      ros-noetic-trac-ik-kinematics-plugin \
      ros-noetic-universal-robots \
      ros-noetic-ur-robot-driver \
      ros-noetic-gazebo-ros-control \
      ros-noetic-ros-controllers \
      ros-noetic-roboticsgroup-upatras-gazebo-plugins \
      ros-noetic-apriltag-ros \
      ros-noetic-rqt-joint-trajectory-controller \
      python3-catkin-tools \
      python3-osrf-pycommon \
      mesa-utils \
      sudo \
      bash-completion \
      less \
      nano \
    && rm -rf /var/lib/apt/lists/*

# Same UID/GID as the host user: files in the mounted workspace stay owned by the user.
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN if getent passwd ${USER_UID} >/dev/null; then userdel -r "$(getent passwd ${USER_UID} | cut -d: -f1)"; fi \
    && if getent group ${USER_GID} >/dev/null; then groupdel "$(getent group ${USER_GID} | cut -d: -f1)" || true; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && (usermod -aG video,render ${USERNAME} 2>/dev/null || usermod -aG video ${USERNAME}) \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep skips end-of-life distros unless asked to include them
USER ${USERNAME}
RUN rosdep update --rosdistro noetic --include-eol-distros
USER root

COPY ros_env.sh /etc/profile.d/ros_env.sh
RUN echo 'source /etc/profile.d/ros_env.sh' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}
