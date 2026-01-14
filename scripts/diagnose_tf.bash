#!/usr/bin/env bash
# =============================================================================
# TF Diagnostics Script
# =============================================================================
# This script helps diagnose TF issues by showing:
# - What TF topics exist
# - What frames are available
# - What each node is subscribing to
# =============================================================================

set -euo pipefail

echo "=============================================="
echo "  TF Diagnostics"
echo "=============================================="
echo ""

echo "=== Available TF Topics ==="
ros2 topic list | grep -E "(^/tf|/tf_static|kevin/tf)" || echo "No TF topics found"
echo ""

echo "=== /tf Topic Info ==="
ros2 topic info /tf || echo "Topic /tf does not exist"
echo ""

echo "=== /kevin/tf Topic Info ==="
ros2 topic info /kevin/tf || echo "Topic /kevin/tf does not exist"
echo ""

echo "=== Listening to /kevin/tf for 3 seconds ==="
timeout 3 ros2 topic echo /kevin/tf --once || echo "No messages received on /kevin/tf"
echo ""

echo "=== Available TF Frames (from /tf) ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -20 || echo "Could not get transform"
echo ""

echo "=== Available TF Frames (from /kevin/tf) ==="
timeout 5 ros2 run tf2_ros tf2_echo --ros-args --remap /tf:=/kevin/tf -- map base_link 2>&1 | head -20 || echo "Could not get transform"
echo ""

echo "=== Node List (kevin namespace) ==="
ros2 node list | grep kevin || echo "No kevin namespace nodes found"
echo ""

echo "=============================================="
echo "  Diagnostics Complete"
echo "=============================================="
