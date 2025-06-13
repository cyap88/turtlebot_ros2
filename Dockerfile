# use arm64 ros:humble as base image
FROM arm64v8/ros:humble-ros-core

# remove old ros repo configs and gpg keys to avoid conflicts
RUN rm -f /etc/apt/sources.list.d/ros* \
  && rm -f /etc/apt/trusted.gpg.d/ros2-latest-archive-keyring.gpg \
  && apt-get clean

# install curl to install new ros key using Ubuntu repos
RUN apt-get update -yq --allow-insecure-repositories \
  && apt-get install -y --allow-unauthenticated curl \
  && rm -rf /var/lib/apt/lists/*

# add latest ros2 GPG key and repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list

# install dependencies
RUN apt-get update && apt-get install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  ros-humble-turtlebot3* \
  ros-humble-webots-ros2 \
  git \
  && rm -rf /var/lib/apt/lists/*

# configure environment
ENV TURTLEBOT3_MODEL=burger
ENV WEBOTS_HOME=/Applications/Webots.app
# ENV DISPLAY=host.docker.internal:0 