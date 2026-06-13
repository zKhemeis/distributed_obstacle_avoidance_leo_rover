FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    locales \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    nano \
    vim \
    build-essential

RUN locale-gen en_US en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
> /etc/apt/sources.list.d/ros2.list
RUN apt update && apt install -y \
    ros-humble-desktop
RUN apt install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN apt update && apt install -y \
    ros-humble-ros-gz
RUN apt update && apt install -y \
    gz-fortress
RUN mkdir -p /root/leo_ws/src
