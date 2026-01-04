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

# Run foxglove bridge with reduced verbosity for internal nodes
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  port:="${FOXGLOVE_PORT}" \
  address:="${FOXGLOVE_ADDRESS}" \
  include_hidden:=false \
  send_buffer_limit:=100000000
  echo "Connect from Windows: ws://${FOXGLOVE_CONNECT_HOST}:${FOXGLOVE_PORT}"
else
  echo "Connect from Windows: ws://<wsl-ip>:${FOXGLOVE_PORT}"
fi
echo "Override with FOXGLOVE_ADDRESS, FOXGLOVE_PORT, or FOXGLOVE_CONNECT_HOST."

exec ros2 run foxglove_bridge foxglove_bridge --ros-args \
  -p address:="$FOXGLOVE_ADDRESS" \
  -p port:="$FOXGLOVE_PORT"
