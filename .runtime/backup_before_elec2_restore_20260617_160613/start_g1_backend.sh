#!/bin/bash
# Run on the G1 body. Starts the local DDS/RPC control backend and web UI.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${BASE}/.runtime"
mkdir -p "$RUNTIME_DIR"

# G1 本体上的 DDS/雷达链路固定走 192.168.123.x 内网，HTTP 仍监听 0.0.0.0
# 这样 PC 可经 WiFi 访问后台，但本体控制和导航不会误选 wlan0。
export HONGTU_G1_NET_IF="${HONGTU_G1_NET_IF:-eth0}"
export HONGTU_SERVICE_HOST="${HONGTU_SERVICE_HOST:-0.0.0.0}"
export HONGTU_SERVICE_PORT="${HONGTU_SERVICE_PORT:-5055}"
export HONGTU_NAV_CMD_BRIDGE="${HONGTU_NAV_CMD_BRIDGE:-1}"
export HONGTU_PAUSE_CONTROL_DURING_NAV="${HONGTU_PAUSE_CONTROL_DURING_NAV:-1}"
export HONGTU_NAV_GOAL_BACKOFF_M="${HONGTU_NAV_GOAL_BACKOFF_M:-0.25}"
export HONGTU_NAV_GOAL_LATERAL_M="${HONGTU_NAV_GOAL_LATERAL_M:--0.08}"
export HONGTU_NAV_GOAL_YAW_OFFSET_DEG="${HONGTU_NAV_GOAL_YAW_OFFSET_DEG:-0}"
export HONGTU_AUTO_RELOC="${HONGTU_AUTO_RELOC:-1}"
export HONGTU_AUTO_RELOC_DELAY="${HONGTU_AUTO_RELOC_DELAY:-6}"
export HONGTU_AUTO_RELOC_XY_STEP="${HONGTU_AUTO_RELOC_XY_STEP:-0.3}"
export HONGTU_AUTO_RELOC_YAW_STEP_DEG="${HONGTU_AUTO_RELOC_YAW_STEP_DEG:-15}"
export HONGTU_AUTO_RELOC_MAX_TRIES="${HONGTU_AUTO_RELOC_MAX_TRIES:-40}"
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
export PYTHONPATH="${RUNTIME_DIR}/python:${BASE}/unitree_sdk2_python:${BASE}/inspire_hand:${PYTHONPATH}"
export LD_LIBRARY_PATH="${CYCLONEDDS_HOME}/lib:/usr/local/athena/third_party/arm64/lib/unitree_sdk2:${LD_LIBRARY_PATH}"
export LANG="${LANG:-en_US.UTF-8}"
export LC_CTYPE="${LC_CTYPE:-${LANG}}"

if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi
if [ -f "${BASE}/G1Nav2D/devel/setup.bash" ]; then
    source "${BASE}/G1Nav2D/devel/setup.bash"
fi

cd "$BASE"

if [ "${HONGTU_SETUP_HAND_RELAY:-1}" = "1" ] && [ -x "${BASE}/setup_hand_relay.sh" ]; then
    "${BASE}/setup_hand_relay.sh" || echo "[backend] 灵巧手端口转发配置失败（可忽略，不影响非灵巧手功能）"
fi

PID_R=""
PID_L=""
start_hand_driver_once() {
    local lr="$1"
    local ip="$2"
    local name="$3"
    if timeout 2 bash -lc "</dev/tcp/${ip}/6000" >/dev/null 2>&1; then
        echo "[backend] 启动${name}驱动 (${ip})…"
        python3 inspire_hand_driver.py --lr "$lr" --tcp-ip "$ip" --network "$HONGTU_G1_NET_IF" &
        if [ "$lr" = "r" ]; then
            PID_R=$!
        else
            PID_L=$!
        fi
    else
        echo "[backend] 跳过${name}驱动：${ip}:6000 不可达（未连接灵巧手时可忽略）"
    fi
}

if python3 - <<'PY'
import pymodbus
PY
then
    start_hand_driver_once r 192.168.123.211 "右手"
    start_hand_driver_once l 192.168.123.210 "左手"
else
    echo "[backend] 跳过灵巧手 Modbus 驱动：未安装 pymodbus"
fi

cleanup() {
    [ -n "$PID_R" ] && kill "$PID_R" 2>/dev/null || true
    [ -n "$PID_L" ] && kill "$PID_L" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[backend] HongTu: $BASE"
echo "[backend] DDS net_if: $HONGTU_G1_NET_IF"
echo "[backend] HTTP: http://0.0.0.0:${HONGTU_SERVICE_PORT}"
python3 g1_robot_service.py --host "$HONGTU_SERVICE_HOST" --port "$HONGTU_SERVICE_PORT" --net-if "$HONGTU_G1_NET_IF"
