#!/bin/bash
# One-shot Inspire hand TCP relay for G1 WiFi access.

set -e

SUDO_PASSWORD="${HONGTU_SUDO_PASSWORD:-123}"

sudo_cmd() {
    echo "$SUDO_PASSWORD" | sudo -S "$@" >/dev/null
}

ensure_rule() {
    local table="$1"
    shift
    if [ "$table" = "filter" ]; then
        echo "$SUDO_PASSWORD" | sudo -S iptables -C "$@" >/dev/null 2>&1 || sudo_cmd iptables -A "$@"
    else
        echo "$SUDO_PASSWORD" | sudo -S iptables -t "$table" -C "$@" >/dev/null 2>&1 || sudo_cmd iptables -t "$table" -A "$@"
    fi
}

sudo_cmd sysctl -w net.ipv4.ip_forward=1

ensure_rule nat PREROUTING -p tcp --dport 5021 -j DNAT --to-destination 192.168.123.210:6000
ensure_rule nat PREROUTING -p tcp --dport 5022 -j DNAT --to-destination 192.168.123.211:6000

ensure_rule filter FORWARD -i wlan0 -o eth0 -p tcp --dport 6000 -d 192.168.123.210 -j ACCEPT
ensure_rule filter FORWARD -i wlan0 -o eth0 -p tcp --dport 6000 -d 192.168.123.211 -j ACCEPT
ensure_rule filter FORWARD -i eth0 -o wlan0 -p tcp --sport 6000 -s 192.168.123.210 -j ACCEPT
ensure_rule filter FORWARD -i eth0 -o wlan0 -p tcp --sport 6000 -s 192.168.123.211 -j ACCEPT
ensure_rule filter FORWARD -m state --state RELATED,ESTABLISHED -j ACCEPT

ensure_rule nat POSTROUTING -p tcp -d 192.168.123.210 --dport 6000 -j MASQUERADE
ensure_rule nat POSTROUTING -p tcp -d 192.168.123.211 --dport 6000 -j MASQUERADE

echo "[hand-relay] 端口转发已配置（一次性）"
