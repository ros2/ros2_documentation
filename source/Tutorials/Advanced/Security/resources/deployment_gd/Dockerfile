ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Install required packages
RUN apt-get update && apt-get install -y \
      tree \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*
