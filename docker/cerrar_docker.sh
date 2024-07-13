#!/bin/bash
number="${1:-}"
docker kill lidar-slam"$number"
docker rm lidar-slam"$number"