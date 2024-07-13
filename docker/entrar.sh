#!/bin/bash
number="${1:-}"
xhost +local:*
docker exec -e "TERM=xterm-color" -it lidar-slam$number bash
