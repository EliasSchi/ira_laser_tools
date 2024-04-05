FROM althack/ros2:humble-full

LABEL maintainer="elias-schildge@enabl-tech.de"

ENV ROS_DISTRO=humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN apt update \
  && apt install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*
# Set up autocompletion for user
RUN apt update && apt install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/* 

RUN apt-get update && apt-get install -y ros-humble-perception-pcl \
    ros-humble-pcl-ros

# Copy entrypoint scripts
RUN mkdir -p /usr/local/bin/scripts
COPY scripts/workspace-entrypoint.sh /usr/local/bin/scripts/
RUN  chmod +x /usr/local/bin/scripts/*.sh

# Copy middleware profiles
RUN mkdir -p /usr/local/share/middleware_profiles
COPY middleware_profiles/*.xml /usr/local/share/middleware_profiles/

RUN mkdir -p /colcon_ws
RUN chown -R ros:ros /colcon_ws
USER ros
