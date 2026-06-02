#!/bin/bash
# Open a PC-local web tunnel to the G1 backend and keep it alive.

set -e

G1_HOST="${G1_HOST:-${G1_BACKEND_HOST:-192.168.123.164}}"
G1_USER="${G1_USER:-unitree}"
G1_BACKEND_PORT="${G1_BACKEND_PORT:-5055}"
G1_TUNNEL_PORT="${G1_TUNNEL_PORT:-15055}"

can_http() {
    local url="$1"
    python3 - "$url" <<'PY'
import json
import sys
import urllib.request

url = sys.argv[1].rstrip("/") + "/api/status"
try:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=2.0) as resp:
        data = json.loads(resp.read().decode("utf-8"))
    raise SystemExit(0 if data.get("ok") else 1)
except Exception:
    raise SystemExit(1)
PY
}

LOCAL_URL="http://127.0.0.1:${G1_TUNNEL_PORT}"

if can_http "$LOCAL_URL"; then
    echo "[web] already running: ${LOCAL_URL}/"
    echo "[web] press Ctrl+C only if this script owns the tunnel in another terminal."
    while true; do sleep 3600; done
fi

if ss -ltn 2>/dev/null | grep -q "127.0.0.1:${G1_TUNNEL_PORT} "; then
    echo "[web] local port ${G1_TUNNEL_PORT} is occupied but not responding."
    echo "[web] try another port: G1_TUNNEL_PORT=15056 ./start_web_gui.sh"
    exit 1
fi

echo "[web] opening SSH tunnel ${LOCAL_URL}/ -> ${G1_HOST}:${G1_BACKEND_PORT}"
echo "[web] keep this terminal open, then visit: ${LOCAL_URL}/"
ssh -o ExitOnForwardFailure=yes -N \
    -L "127.0.0.1:${G1_TUNNEL_PORT}:127.0.0.1:${G1_BACKEND_PORT}" \
    "${G1_USER}@${G1_HOST}"
