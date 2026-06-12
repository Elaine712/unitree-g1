#!/bin/bash
# Run on the PC. Starts the G1 backend if needed, then runs the pull-switch demo.

set -e

G1_USER="${G1_USER:-unitree}"
G1_WIRED_HOST="${G1_WIRED_HOST:-192.168.123.164}"
G1_WIFI_HOST="${G1_WIFI_HOST:-192.168.13.24}"
G1_HOST="${G1_HOST:-${G1_BACKEND_HOST:-}}"
G1_DIR="${G1_DIR:-/home/unitree/zgx_g1}"
G1_POSES="${G1_POSES:-g1_poses2.json}"
SSH_OPTS=(-o ConnectTimeout=3)

can_ssh() {
    local host="$1"
    ssh "${SSH_OPTS[@]}" "${G1_USER}@${host}" "true" >/dev/null 2>&1
}

if [ -z "$G1_HOST" ]; then
    for host in "$G1_WIRED_HOST" "$G1_WIFI_HOST"; do
        if can_ssh "$host"; then
            G1_HOST="$host"
            break
        fi
    done
fi

if [ -z "$G1_HOST" ]; then
    echo "[pull-demo] 未找到可 SSH 的 G1。可指定：G1_HOST=192.168.13.24 ./run_pull_switch_demo.sh" >&2
    exit 1
fi

echo "[pull-demo] G1 host: $G1_HOST"
echo "[pull-demo] checking backend..."
ssh "${SSH_OPTS[@]}" "${G1_USER}@${G1_HOST}" \
    "cd '${G1_DIR}' && mkdir -p .runtime && \
     if ss -ltn | grep -q ':5055 '; then \
         echo '[pull-demo] backend already running'; \
         pgrep -af g1_robot_service.py || true; \
     else \
         echo '[pull-demo] starting backend'; \
         nohup ./start_g1_backend.sh > .runtime/g1_backend.log 2>&1 < /dev/null & \
         for i in 1 2 3 4 5 6 7 8; do ss -ltn | grep -q ':5055 ' && break; sleep 0.5; done; \
         ss -ltn | grep 5055; \
         pgrep -af g1_robot_service.py; \
     fi"

echo "[pull-demo] running pull-switch demo..."
ssh "${SSH_OPTS[@]}" "${G1_USER}@${G1_HOST}" \
    "cd '${G1_DIR}' && ./demo_pull_switch.py --poses '${G1_POSES}' $*"
