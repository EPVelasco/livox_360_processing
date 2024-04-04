FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} livox
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} livox
RUN usermod -a -G dialout livox
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

RUN apt-get update && apt-get install -y \
    git build-essential ros-noetic-perception-pcl ros-noetic-rviz ros-noetic-eigen-conversions iputils-ping screen vim nano && \
    rm -rf /var/lib/apt/lists/*

ENV HOME /home/livox
RUN mkdir -p ${HOME}/catkin_ws/library

RUN cd ${HOME}/catkin_ws/library && git clone https://github.com/EPVelasco/Livox-SDK2.git && \
    mkdir -p ./Livox-SDK2/build && \
    cd ./Livox-SDK2/build && \
    cmake .. && make -j && make install
    
RUN . /opt/ros/noetic/setup.sh && \
    mkdir -p ${HOME}/catkin_ws/src && \ 
    cd ${HOME}/catkin_ws/src && \
    git clone https://github.com/EPVelasco/livox_ros_driver2.git && \
    cd ${HOME}/catkin_ws/src/livox_ros_driver2 && ./build.sh ROS1
    
COPY ./MID360_config.json ${HOME}/catkin_ws/src/livox_ros_driver2/config/

WORKDIR ${HOME}/catkin_ws/
CMD ["bash", "-c", "source ${HOME}/catkin_ws/devel/setup.bash && roslaunch livox_ros_driver2 rviz_MID360.launch rviz_enable:=false"]

