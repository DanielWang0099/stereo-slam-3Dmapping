#!/usr/bin/env bash
set -euo pipefail

# Continuously plays the stereo-imu rosbag.
# Intended to be run inside the Isaac ROS dev container.
#
# Defaults can be overridden via env vars:
#   BAG_PATH (default: /workspaces/isaac_ros-dev/bags/bag_with_stereo_imu)
#   RATE     (default: 1.0)
#   START    (default: 0.0 seconds)

BAG_PATH="${BAG_PATH:-/workspaces/isaac_ros-dev/bags/bag_with_stereo_imu}"
RATE="${RATE:-1.0}"
START="${START:-0.0}"

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "ERROR: Bag not found or invalid: $BAG_PATH (missing metadata.yaml)" >&2
  exit 1
fi

# IMPORTANT: Exclude /tf from bag playback to prevent vehicle odometry pollution
# The bag contains odom->base_link on /tf which should NOT be on the global /tf
# Only ground truth (map->robot/base_link_gt) should be on global /tf
# Vehicle-specific transforms are correctly on /kevin/tf
exec ros2 bag play "$BAG_PATH" \
  --loop \
  --rate "$RATE" \
  --start-offset "$START" \
  --topics-regex "^(?!/tf$).*"
