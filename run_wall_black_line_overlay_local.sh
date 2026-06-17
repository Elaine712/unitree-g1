#!/usr/bin/env bash
# Run the wall black-line live overlay on this PC against the G1 ROS master.

set -e

BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_ARGS=("$@")
G1_HOST="${G1_HOST:-10.231.138.24}"
WALL_LINE_ROS_MASTER_URI="${WALL_LINE_ROS_MASTER_URI:-http://${G1_HOST}:11311}"

if [ -z "${ROS_IP:-}" ]; then
    WALL_LINE_ROS_IP="$(python3 - "$G1_HOST" <<'PY'
import socket
import sys

host = sys.argv[1]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.connect((host, 11311))
    print(sock.getsockname()[0])
finally:
    sock.close()
PY
)"
else
    WALL_LINE_ROS_IP="$ROS_IP"
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-${BASE}/.runtime/ros_logs}"
mkdir -p "$ROS_LOG_DIR" "${BASE}/.runtime/wall_black_line_live"

set --
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="$WALL_LINE_ROS_MASTER_URI"
export ROS_IP="$WALL_LINE_ROS_IP"

echo "[wall-line] ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "[wall-line] ROS_IP=${ROS_IP}"
echo "[wall-line] latest: ${BASE}/.runtime/wall_black_line_live/latest_overlay.jpg"

exec "${BASE}/tools/live_wall_black_line_overlay_local.py" "${SCRIPT_ARGS[@]}"
