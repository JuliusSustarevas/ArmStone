FROM ros:noetic-perception

RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pip \
    build-essential \
    ros-noetic-rqt \
    ros-noetic-rqt-joint-trajectory-controller \
    ros-noetic-rqt-console \
    ros-noetic-rqt-logger-level \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher \
    ros-noetic-moveit \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-moveit-resources \
    ros-noetic-rviz \
    ros-noetic-rviz-visual-tools \
    ros-noetic-eigen-conversions \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rosserial \
    git \
    ssh \
    usbutils \
    iputils-ping \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install catkin_tools

RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/ros-planning/moveit_visual_tools.git \
                        && git clone https://github.com/wjwwood/serial.git"]

COPY . /catkin_ws/src/arm_stone

WORKDIR /catkin_ws/src/arm_stone
RUN ["/bin/bash", "-c", "git submodule init && git submodule update"]

WORKDIR /catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && \
    catkin build"]