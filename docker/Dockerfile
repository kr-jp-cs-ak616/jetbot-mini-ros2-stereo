# Supply a JetPack/L4T-compatible ARM64 image WITH ROS 2 already installed.
ARG BASE_IMAGE
FROM ${BASE_IMAGE}
USER root
SHELL ["/bin/bash", "-c"]
RUN test -n "$ROS_DISTRO" && test -f /opt/ros/$ROS_DISTRO/setup.bash && \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-opencv python3-yaml \
      gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-proc \
      ros-${ROS_DISTRO}-camera-calibration ros-${ROS_DISTRO}-rtabmap-ros \
      ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-nav2-map-server && \
    rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY src /ws/src
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["ros2", "launch", "jetbot_stereo", "stereo.launch.py"]
