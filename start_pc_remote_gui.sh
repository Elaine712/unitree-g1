#!/bin/bash
# Run on the PC. Starts the GUI locally and sends G1 commands to the body backend.

set -e

BASE="$(cd "$(dirname "$0")" && pwd)"
RUNTIME_DIR="${BASE}/.runtime"
LAST_HOST_FILE="${RUNTIME_DIR}/last_g1_backend_host"
mkdir -p "$RUNTIME_DIR"
G1_WIRED_HOST="${G1_WIRED_HOST:-192.168.123.164}"
G1_WIFI_HOST="${G1_WIFI_HOST:-192.168.13.24}"
G1_BACKEND_HOST="${G1_BACKEND_HOST:-${G1_HOST:-}}"
G1_USER="${G1_USER:-unitree}"
G1_BACKEND_PORT="${G1_BACKEND_PORT:-5055}"
G1_TUNNEL_PORT="${G1_TUNNEL_PORT:-15055}"
SSH_CHECK_OPTS=(-o BatchMode=yes -o ConnectTimeout=2)
TUNNEL_PID=""
BACKEND_ADDR=""
BACKEND_HOST=""
LAST_BACKEND_HOST=""
[ -f "$LAST_HOST_FILE" ] && LAST_BACKEND_HOST="$(tr -d '[:space:]' < "$LAST_HOST_FILE")"
export NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,192.168.0.0/16,10.0.0.0/8}"
export no_proxy="${no_proxy:-$NO_PROXY}"

can_http() {
    local host="$1"
    local port="${2:-$G1_BACKEND_PORT}"
    python3 - "$host" "$port" <<'PY'
import json
import sys
import urllib.request
host, port = sys.argv[1], sys.argv[2]
try:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(f"http://{host}:{port}/api/status", timeout=1.5) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    raise SystemExit(0 if data.get("ok") else 1)
except Exception:
    raise SystemExit(1)
PY
}

can_ssh() {
    local host="$1"
    ssh "${SSH_CHECK_OPTS[@]}" "${G1_USER}@${host}" "true" >/dev/null 2>&1
}

start_remote_backend() {
    local host="$1"
    echo "[pc-gui] backend not running, starting on ${host}..." >&2
    ssh "${SSH_CHECK_OPTS[@]}" "${G1_USER}@${host}" \
        "cd /home/unitree/zgx_g1 && mkdir -p .runtime && if ! ss -ltn | grep -q ':${G1_BACKEND_PORT} '; then nohup ./start_g1_backend.sh > .runtime/g1_backend.log 2>&1 < /dev/null & fi"
    for _ in $(seq 1 40); do
        if can_http "$host"; then
            BACKEND_ADDR="${host}:${G1_BACKEND_PORT}"
            BACKEND_HOST="$host"
            return 0
        fi
        sleep 0.5
    done
    echo "[pc-gui] backend did not become ready on ${host}; last log:" >&2
    ssh "${SSH_CHECK_OPTS[@]}" "${G1_USER}@${host}" "tail -80 /home/unitree/zgx_g1/.runtime/g1_backend.log 2>/dev/null || true" >&2 || true
    return 1
}

start_tunnel() {
    local host="$1"
    echo "[pc-gui] direct HTTP blocked, opening SSH tunnel ${G1_TUNNEL_PORT}->${host}:${G1_BACKEND_PORT}..." >&2
    ssh "${SSH_CHECK_OPTS[@]}" -o ExitOnForwardFailure=yes -N \
        -L "127.0.0.1:${G1_TUNNEL_PORT}:127.0.0.1:${G1_BACKEND_PORT}" \
        "${G1_USER}@${host}" &
    TUNNEL_PID=$!
    for _ in $(seq 1 20); do
        if can_http "127.0.0.1" "$G1_TUNNEL_PORT"; then
            BACKEND_ADDR="127.0.0.1:${G1_TUNNEL_PORT}"
            BACKEND_HOST="$host"
            return 0
        fi
        sleep 0.2
    done
    echo "[pc-gui] SSH tunnel started but backend did not respond" >&2
    return 1
}

wifi_prefixes() {
    ip -o -4 addr show up 2>/dev/null | awk '
        $2 ~ /^(wl|wlan)/ {
            split($4, a, "/");
            split(a[1], b, ".");
            if (b[1] && b[2] && b[3]) print b[1]"."b[2]"."b[3]
        }' | sort -u
}

try_backend_host() {
    local host="$1"
    [ -z "$host" ] && return 1
    echo "[pc-gui] checking backend ${host}:${G1_BACKEND_PORT}..." >&2
    if can_http "$host"; then
        BACKEND_ADDR="${host}:${G1_BACKEND_PORT}"
        BACKEND_HOST="$host"
        return 0
    fi
    if can_ssh "$host"; then
        if start_remote_backend "$host"; then
            return 0
        fi
        start_tunnel "$host"
        return 0
    fi
    return 1
}

discover_backend() {
    if [ -n "$G1_BACKEND_HOST" ]; then
        try_backend_host "$G1_BACKEND_HOST" && return 0
        echo "[pc-gui] 指定的 G1_BACKEND_HOST 不可访问: $G1_BACKEND_HOST" >&2
        return 1
    fi
    for host in "$LAST_BACKEND_HOST" "$G1_WIFI_HOST" "192.168.1.24" "$G1_WIRED_HOST"; do
        try_backend_host "$host" && return 0
    done
    for prefix in $(wifi_prefixes); do
        for suffix in 24 164; do
            host="${prefix}.${suffix}"
            echo "[pc-gui] probing backend ${host}:${G1_BACKEND_PORT}..." >&2
            try_backend_host "$host" && return 0
        done
    done
    echo "[pc-gui] 未找到 G1 后台服务。先在 G1 上运行: cd /home/unitree/zgx_g1 && ./start_g1_backend.sh" >&2
    return 1
}

cleanup() {
    if [ -n "$TUNNEL_PID" ]; then
        kill "$TUNNEL_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

if discover_backend; then
    [ -n "$BACKEND_HOST" ] && printf '%s\n' "$BACKEND_HOST" > "$LAST_HOST_FILE"
    export HONGTU_G1_BACKEND_URL="http://${BACKEND_ADDR}"
    export HONGTU_REMOTE_AUTO_CONNECT="${HONGTU_REMOTE_AUTO_CONNECT:-1}"
else
    FALLBACK_HOST="${LAST_BACKEND_HOST:-${G1_WIFI_HOST:-192.168.13.24}}"
    export HONGTU_G1_BACKEND_URL="http://${FALLBACK_HOST}:${G1_BACKEND_PORT}"
    export HONGTU_REMOTE_AUTO_CONNECT=0
    echo "[pc-gui] 自动连接失败，仍打开 GUI，可在左上角手动修改并连接: $HONGTU_G1_BACKEND_URL" >&2
fi

echo "[pc-gui] backend: $HONGTU_G1_BACKEND_URL"
if [ -n "$BACKEND_HOST" ]; then
    echo "[pc-gui] G1 host: $BACKEND_HOST"
fi
if [ "${HONGTU_PC_GUI_DRY_RUN:-0}" = "1" ]; then
    echo "[pc-gui] dry run ok"
    exit 0
fi
cd "$BASE/g1_nav_panel"
python3 main.py
