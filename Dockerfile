FROM osrf/ros:melodic-desktop-full
RUN apt-get update && apt-get install -y \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    python-jinja2 \
    python-pip \
    python-catkin-tools \
    python-rosinstall-generator && \
    pip install numpy toml && \
    mkdir -p /usr/src/catkin_ws/src/spd_simulation && \
    mkdir -p /usr/src/px4src && \
    cd /usr/src/px4src/
SHELL ["/bin/bash", "-c"]
ADD install.sh /usr/src/catkin_ws
RUN source /ros_entrypoint.sh && chmod +x /usr/src/catkin_ws/install.sh && cd /usr/src/catkin_ws && ./install.sh

