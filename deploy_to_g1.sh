#!/bin/bash
# Deploy HongTu to an isolated directory on the G1 body over the wired link.

set -e

G1_HOST="${G1_HOST:-192.168.123.164}"
G1_USER="${G1_USER:-unitree}"
G1_DIR="${G1_DIR:-/home/unitree/zgx_g1}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.123.164}"
MAP_SRC_DIR="${MAP_SRC_DIR:-$HOME/Desktop}"
REMOTE="${G1_USER}@${G1_HOST}"

BASE="$(cd "$(dirname "$0")" && pwd)"

echo "[deploy] target: ${REMOTE}:${G1_DIR}"
echo "[deploy] source: ${BASE}"

ssh "$REMOTE" "mkdir -p '${G1_DIR}' '${G1_DIR}/.runtime'"

rsync -az --delete \
    --exclude='.git/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.runtime/' \
    --exclude='G1Nav2D/build/' \
    --exclude='G1Nav2D/devel/' \
    "$BASE/" "${REMOTE}:${G1_DIR}/"

ssh "$REMOTE" "cd '${G1_DIR}' && chmod +x start_robot_gui.sh start_g1_backend.sh start_pc_remote_gui.sh build_g1_nav_on_robot.sh deploy_to_g1.sh remote_robot_gui.sh g1_robot_service.py g1_remote_client.py"

ssh "$REMOTE" "cd '${G1_DIR}' && python3 - '${LIVOX_HOST_IP}' <<'PY'
import json
import sys
from pathlib import Path

host_ip = sys.argv[1]
path = Path('G1Nav2D/src/livox_ros_driver2/config/MID360_config.json')
if path.exists():
    data = json.loads(path.read_text())
    info = data.get('MID360', {}).get('host_net_info', {})
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        if key in info:
            info[key] = host_ip
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    print(f'[deploy] MID360 host_net_info -> {host_ip}')
else:
    print(f'[deploy] skip MID360 config, not found: {path}')
PY"

if [ -f "${MAP_SRC_DIR}/G1map.pgm" ] && [ -f "${MAP_SRC_DIR}/G1map.yaml" ]; then
    echo "[deploy] map: ${MAP_SRC_DIR}/G1map.* -> ${G1_DIR}/.runtime/maps/"
    ssh "$REMOTE" "mkdir -p '${G1_DIR}/.runtime/maps'"
    rsync -az "${MAP_SRC_DIR}/G1map.pgm" "${REMOTE}:${G1_DIR}/.runtime/maps/G1map.pgm"
    resolution="$(awk '/^resolution:/ {print $2; exit}' "${MAP_SRC_DIR}/G1map.yaml")"
    origin="$(sed -n 's/^origin:[[:space:]]*//p' "${MAP_SRC_DIR}/G1map.yaml" | head -1)"
    negate="$(awk '/^negate:/ {print $2; exit}' "${MAP_SRC_DIR}/G1map.yaml")"
    occupied_thresh="$(awk '/^occupied_thresh:/ {print $2; exit}' "${MAP_SRC_DIR}/G1map.yaml")"
    free_thresh="$(awk '/^free_thresh:/ {print $2; exit}' "${MAP_SRC_DIR}/G1map.yaml")"
    ssh "$REMOTE" "cat > '${G1_DIR}/.runtime/maps/G1map.yaml' <<EOF
image: ${G1_DIR}/.runtime/maps/G1map.pgm
resolution: ${resolution:-0.050000}
origin: ${origin:-[0.0, 0.0, 0.0]}
negate: ${negate:-0}
occupied_thresh: ${occupied_thresh:-0.65}
free_thresh: ${free_thresh:-0.196}
EOF"
else
    echo "[deploy] skip map copy: ${MAP_SRC_DIR}/G1map.pgm/yaml not found"
fi

echo "[deploy] done"
echo "[deploy] remote start:"
echo "  ssh ${REMOTE} 'cd ${G1_DIR} && ./start_g1_backend.sh'"
echo "[deploy] if navigation binaries are missing on G1:"
echo "  ssh ${REMOTE} 'cd ${G1_DIR} && ./build_g1_nav_on_robot.sh'"
