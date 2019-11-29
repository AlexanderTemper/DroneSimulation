FROM osrf/ros:melodic-desktop-full
# update 
RUN apt-get update && apt-get install -y
RUN apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
RUN mkdir -p /usr/src/catkin_ws
