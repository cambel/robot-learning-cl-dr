# Docker for proyect glozzom (BASE)
# ros-melodic-base, gazebo9, gps, python libraries 
# Python 3 version

ARG UBUNTU_VERSION=20.04

FROM nvidia/cuda:11.5.1-cudnn8-devel-ubuntu20.04
LABEL maintainer Cristian C Beltran-Hernandez "beltran@hlab.sys.es.osaka-u.ac.jp"

# Avoid user interaction during install of tzdata
# https://askubuntu.com/questions/909277/avoiding-user-interaction-with-tzdata-when-installing-certbot-in-a-docker-contai
ENV TZ=Asia
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS Noetic Ninjemys.
# http://wiki.ros.org/noetic/Installation/Ubuntu

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-core \
    && rm -rf /var/lib/apt/lists/*

ENV python=python3
ENV pip=pip3

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python-is-python3 \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-pip \
    build-essential \
	&& rm -rf /var/lib/apt/lists/*

# install additional packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-spacenav-node \
    python3-catkin-tools \
    # utils
    git \
    locate \
    aptitude \
    vim htop tmux \
    curl wget \
    spacenavd \
    && rm -rf /var/lib/apt/lists/*

## Python libraries ##
RUN python3 -m pip install pip --upgrade && \
    pip install matplotlib==2.2.3 spicy protobuf pyyaml pyquaternion rospkg \
    lxml tqdm catkin-pkg empy PyVirtualDisplay defusedxml gym psutil pyprind \
    xmltodict imageio IPython future netifaces pandas sklearn seaborn

RUN apt-get update && apt-get install -y \
    python3-pymodbus \
    python3-catkin-pkg \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

### Ros Workspace ###
# Set up the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && mkdir -p ~/ros_ws/src \
    && cd ~/ros_ws/ \
    && catkin init"

# Installing repo required for homework
RUN /bin/bash -c "cd ~/ros_ws/src \
    && git clone  -b noetic-devel https://github.com/cambel/robot-learning-cl-dr.git glozzom"

RUN rosdep init

# Updating ROSDEP and installing dependencies
RUN cd ~/ros_ws \
    && rosinstall ~/ros_ws/src /opt/ros/$ROS_DISTRO src/glozzom/dependencies.rosinstall \
    && apt update -qq \
    && rosdep fix-permissions \
    && rosdep update -q \
    && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Compiling ros workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ~/ros_ws/ \
    && rm -rf build \
    && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DPYTHON_VERSION=3"

COPY docker/bashrc /etc/bash.bashrc
RUN chmod a+rwx /etc/bash.bashrc

ENV PYTHONIOENCODING UTF-8

# add rqt-multiplot
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-rqt-multiplot

RUN pip install xmltodict imageio IPython

RUN pip install PySide2 PyQt5 transform3d

RUN pip3 install pyserial

################################################
# Custom python libs
################################################

RUN pip install Cython cpprb

# fix problem with tf2rl
RUN apt-get -qq update && \
    apt-get -y --force-yes install \
    software-properties-common

# tf2rl
RUN /bin/bash -c "mkdir ~/pylibs && cd ~/pylibs \
    && git clone -b dev https://github.com/cambel/tf2rl.git \
    && cd tf2rl && pip install -e ."

RUN pip install future netifaces pandas sklearn seaborn

COPY docker/gazebo-gui.ini /root/.gazebo/gui.ini

# fix gym
RUN pip install gym[atari]==0.15.7

### End ###

# clean
RUN pip cache purge
RUN rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 11345
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc
RUN echo 'source ~/ros_ws/devel/setup.bash' >> ~/.bashrc

RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN echo 'export PYTHONPATH=/root/orocos_kinematics_dynamics/python_orocos_kdl/build:/root/gps:$PYTHONPATH' >> ~/.bashrc
RUN echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros_ws/src/ros-universal-robots/ur3_gazebo/models/' >> ~/.bashrc

RUN echo "alias cw='cd /root/ros_ws'"
RUN echo "alias cs='cd /root/ros_ws/src'"
RUN echo "alias cb='catkin build'"
RUN echo "alias cc='catkin clean'"
RUN echo "alias s='source /root/o2ac-ur/catkin_ws/devel/setup.bash'"

RUN echo "alias ur3e='roslaunch ur3_gazebo ur_peg.launch'" >> ~/.bashrc
RUN echo 'function ur3e_real { roslaunch ur_control ur3e_bringup.launch robot_ip:="$1" kinematics_config_filename:=${2:-"left"}; }'  >> ~/.bashrc
