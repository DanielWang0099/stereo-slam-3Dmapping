#!/usr/bin/env bash
set -euo pipefail

# Run inside the Isaac ROS dev container (ROS 2 Humble).
# Exposes a Foxglove WebSocket server for Foxglove Studio.

FOXGLOVE_ADDRESS="${FOXGLOVE_ADDRESS:-0.0.0.0}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
FOXGLOVE_CONNECT_HOST="${FOXGLOVE_CONNECT_HOST:-}"

if [[ -z "$FOXGLOVE_CONNECT_HOST" ]] && command -v python3 >/dev/null 2>&1; then
  FOXGLOVE_CONNECT_HOST="$(
    python3 - <<'PY'
import socket
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(("8.8.8.8", 80))
    print(sock.getsockname()[0])
except Exception:
    pass
PY
  )"
fi

if [[ -z "$FOXGLOVE_CONNECT_HOST" ]] && command -v hostname >/dev/null 2>&1; then
  FOXGLOVE_CONNECT_HOST="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi

echo "Foxglove bridge starting on ${FOXGLOVE_ADDRESS}:${FOXGLOVE_PORT}"
if [[ -n "$FOXGLOVE_CONNECT_HOST" ]]; then
  echo "Connect Foxglove Studio to: ws://${FOXGLOVE_CONNECT_HOST}:${FOXGLOVE_PORT}"
fi

# Run foxglove bridge
exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:="$FOXGLOVE_ADDRESS" \
  -p port:="$FOXGLOVE_PORT"
