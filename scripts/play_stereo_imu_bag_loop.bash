#!/usr/bin/env bash
set -euo pipefail

# Continuously plays the stereo-imu rosbag.
# Intended to be run inside the Isaac ROS dev container.
#
# Defaults can be overridden via env vars:
#   BAG_PATH (default: /workspaces/isaac_ros-dev/bags/stereo-imu)
#   RATE     (default: 1.0)
#   START    (default: 0.0 seconds)

BAG_PATH="${BAG_PATH:-/workspaces/isaac_ros-dev/bags/stereo-imu}"
RATE="${RATE:-1.0}"
START="${START:-0.0}"

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "ERROR: Bag not found or invalid: $BAG_PATH (missing metadata.yaml)" >&2
  exit 1
fi

exec ros2 bag play "$BAG_PATH" \
  --loop \
  --rate "$RATE" \
  --start-offset "$START"
