#!/bin/bash
# Build the ROS navigation workspace on the G1 body.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"

if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

cd "$BASE/G1Nav2D"
echo "[build-nav] building in $(pwd)"
catkin_make -DROS_EDITION=ROS1 --make-args -j2
