#!/bin/bash
# One-click hazard pull flow:
# clean old camera/YOLO, start navigation, go to waypoint 4,
# kill navigation ROS, start camera/YOLO, align, then pull switch.

set -e

G1_HOST="${G1_HOST:-10.231.138.24}"

cd "$(dirname "$0")"

exec ./demo_hazard_pull_task.py \
  --host "$G1_HOST" \
  --ensure-backend \
  --start-nav \
  --start-camera \
  --skip-photos \
  "$@"
