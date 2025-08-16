FROM osrf/ros:humble-desktop-full

LABEL shell=bash

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

SHELL ["/bin/bash", "-c"]

# Update expired ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor \
    | tee /usr/share/keyrings/ros2-latest-archive-keyring.gpg > /dev/null

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y \
    sudo \
    curl \
    git \
    git-extras \
    htop \
    tree \
    net-tools \
    tmux \
    vim \
    wget \
    usbutils \
    bash-completion \
    libreoffice \
    python3-pip \
    python3-rosdep \
    python3-setuptools \
    python3-numpy \
    # ros2
    ros-humble-ament-cmake \
    ros-humble-rqt-plot \
    ros-humble-rqt-tf-tree \
    ros-humble-nav2-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get dist-upgrade -y \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash
ENV TERM=xterm-256color

USER $USERNAME

RUN mkdir -p /home/user/ws

WORKDIR /home/$USERNAME/ws

CMD ["/bin/bash", "-ic"]