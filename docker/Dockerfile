ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base
LABEL maintainer="Emanuel Nunez S gmail dot com"
ENV HOME /root
WORKDIR $HOME
SHELL ["/bin/bash", "-c"]


# Install ros2 packages
RUN apt-get update && apt-get install -y \ 
	ros-$ROS_DISTRO-cv-bridge \
	ros-$ROS_DISTRO-vision-opencv \
	ros-$ROS_DISTRO-compressed-image-transport\
	ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-tf-transformations

# General utilities
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    gdb \
    vim \
    nano \
	apt-utils \
	dialog \
	unzip \
	python3-vcstool

# Other tools
RUN apt-get update && apt-get install -y \
	scrot \
	python3-tk\
	python3-dev


##### microros installation #####
#  steps to install the micro-ROS build system
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	mkdir microros_ws && \
	cd microros_ws && \
	git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
	apt-get update && \
	rosdep update && \
	rosdep install --from-paths src --ignore-src -y && \
	apt-get install -y python3-pip && \
	colcon build && \
	source install/local_setup.bash

# To install the micro-ros Agent follow the steps below:
WORKDIR $HOME/microros_ws/
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
	source install/local_setup.bash && \
	ros2 run micro_ros_setup create_agent_ws.sh && \
	ros2 run micro_ros_setup build_agent.sh && \
	source install/local_setup.bash


# SET ENVIRONMENT
# WORKDIR $HOME/ws/origami

RUN echo 'alias python="python3"' >> $HOME/.bashrc
RUN echo 'source /opt/ros/humble/setup.sh && colcon build' >> $HOME/.bashrc
RUN echo 'source install/setup.bash' >> $HOME/.bashrc


