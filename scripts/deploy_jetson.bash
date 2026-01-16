#!/usr/bin/env bash
# =============================================================================
# Jetson Deployment Script
# Build and run Stereo SLAM on Jetson Orin NX / Nano
# =============================================================================

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DOCKER_IMAGE="stereo-slam:jetson-orin"
JETSON_DEVICE="nx"
BUILD_IMAGE=true
BAG_PATH="${BAG_PATH:-}"

print_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[OK]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# -------------------------
# Parse args
# -------------------------
while [[ $# -gt 0 ]]; do
  case $1 in
    --nano) JETSON_DEVICE="nano"; shift ;;
    --no-build) BUILD_IMAGE=false; shift ;;
    --bag-path) BAG_PATH="$2"; shift 2 ;;
    *) print_error "Unknown option: $1"; exit 1 ;;
  esac
done

# -------------------------
# Detect Jetson
# -------------------------
detect_jetson_device() {
  if [[ -f /proc/device-tree/model ]]; then
    local model=$(cat /proc/device-tree/model 2>/dev/null || echo "")
    if [[ "$model" =~ "Orin Nano" ]]; then
      JETSON_DEVICE="nano"
    elif [[ "$model" =~ "Orin NX" ]]; then
      JETSON_DEVICE="nx"
    fi
  fi
}

# -------------------------
# Check prerequisites
# -------------------------
check_prerequisites() {
  print_info "Checking prerequisites..."
  if ! command -v docker &>/dev/null; then
    print_error "Docker not installed"; exit 1
  fi
  print_success "Docker found"

  if ! docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-jetpack:r36.4 nvidia-smi &>/dev/null; then
    print_warning "NVIDIA Docker runtime may not be fully functional"
  else
    print_success "NVIDIA Docker runtime working"
  fi

  detect_jetson_device
  print_success "Detected Jetson device: $JETSON_DEVICE"
}

# -------------------------
# Build Docker image
# -------------------------
build_docker_image() {
  print_info "Building Docker image for Jetson $JETSON_DEVICE..."
  DOCKER_IMAGE="stereo-slam:jetson-orin-${JETSON_DEVICE}"
  local build_memory="3g"
  [[ "$JETSON_DEVICE" == "nano" ]] && build_memory="2g"

  docker build \
    --no-cache \
    -f "${PROJECT_DIR}/docker/Dockerfile.mine" \
    -t "$DOCKER_IMAGE" \
    --memory="$build_memory" \
    "${PROJECT_DIR}" || { print_error "Docker build failed"; exit 1; }

  print_success "Docker image built: $DOCKER_IMAGE"
}

# -------------------------
# Optimize Jetson
# -------------------------
optimize_jetson_for_pipeline() {
  print_info "Optimizing Jetson for pipeline..."
  if [[ "$JETSON_DEVICE" == "nano" ]]; then
    export NITROS_NUM_BUFFERS=4
    export NITROS_BUFFER_SIZE=4194304
    print_warning "Jetson Nano: reduced buffer pool (4 buffers, 4MB each)"
  else
    export NITROS_NUM_BUFFERS=8
    export NITROS_BUFFER_SIZE=8388608
    print_success "Jetson NX: standard buffer pool (8 buffers, 8MB each)"
  fi
}

# -------------------------
# Run container
# -------------------------
run_pipeline() {
  print_info "Starting pipeline container..."
  local docker_args=(
    "--rm" "--runtime=nvidia"
    "-e" "NVIDIA_VISIBLE_DEVICES=all"
    "-e" "CUDA_VISIBLE_DEVICES=0"
    "-e" "NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility"
    "-e" "NITROS_NUM_BUFFERS=${NITROS_NUM_BUFFERS}"
    "-e" "NITROS_BUFFER_SIZE=${NITROS_BUFFER_SIZE}"
    "-v" "${PROJECT_DIR}:/workspaces/isaac_ros-dev"
    "-it"
  )

  [[ -n "$BAG_PATH" ]] && docker_args+=("-v" "${BAG_PATH}:/rosbag:ro" "-e" "BAG_PATH=/rosbag")

  docker run "${docker_args[@]}" "$DOCKER_IMAGE" bash
}

# -------------------------
# Main
# -------------------------
main() {
  print_info "Stereo SLAM Jetson Deployment"
  check_prerequisites
  [[ "$BUILD_IMAGE" == true ]] && build_docker_image
  optimize_jetson_for_pipeline
  print_info "Inside container, run:"
  echo "  cd /workspaces/isaac_ros-dev"
  echo "  colcon build --symlink-install --parallel-workers 2"
  echo "  ./scripts/run_full_pipeline.bash"
  run_pipeline
}

main "$@"
