#!/bin/bash
xhost +local:*
docker exec -e "TERM=xterm-color" -it lidar-slam bash