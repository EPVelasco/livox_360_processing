#!/bin/bash

# Valor por defecto para xfer_format si no se proporciona uno
XFER_FORMAT="${1:-1}"
rosbag_enable="${2:-false}"

# Ejecuta el comando con xfer_format especificado
exec bash -c "source ${HOME}/catkin_ws/devel/setup.bash && roslaunch livox_ros_driver2 rviz_MID360.launch rviz_enable:=false xfer_format:=${XFER_FORMAT} rosbag_enable:=${rosbag_enable}"
