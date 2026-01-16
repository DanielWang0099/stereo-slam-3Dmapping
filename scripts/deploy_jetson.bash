#!/usr/bin/env bash
# =============================================================================
# Jetson Deployment Script
# =============================================================================
# Deploy and run stereo SLAM pipeline on Jetson Orin NX/Nano
# Handles Docker setup, GPU memory optimization, and pipeline launch
#
# Usage:
#   ./deploy_jetson.bash [--nano] [--no-build] [--bag-path PATH]
# =============================================================================

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DOCKER_IMAGE="stereo-slam:jetson-orin"
JETSON_DEVICE="nx"  # or "nano"
BUILD_IMAGE=true
BAG_PATH="${BAG_PATH:-}"

print_info() { echo -e "${CYAN}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[OK]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --nano) JETSON_DEVICE="nano"; shift ;;
    --no-build) BUILD_IMAGE=false; shift ;;
    --bag-path) BAG_PATH="$2"; shift 2 ;;
    *) print_error "Unknown option: $1"; exit 1 ;;
  esac
done

# Detect Jetson device if not specified
detect_jetson_device() {
  if [[ ! -f /proc/device-tree/model ]]; then
    print_error "Not running on Jetson device (no /proc/device-tree/model)"
    return 1
  fi
  
  local model=$(cat /proc/device-tree/model 2>/dev/null || echo "")
  if [[ "$model" =~ "Orin Nano" ]]; then
    JETSON_DEVICE="nano"
  elif [[ "$model" =~ "Orin NX" ]]; then
    JETSON_DEVICE="nx"
  fi
}

check_prerequisites() {
  print_info "Checking prerequisites..."
  
  # Check Docker
  if ! command -v docker &> /dev/null; then
    print_error "Docker not installed"
    exit 1
  fi
  print_success "Docker found"
  
  # Check nvidia-docker runtime
  if ! docker run --rm --runtime=nvidia nvidia/cuda:12.2.0-runtime-ubuntu22.04 nvidia-smi &>/dev/null; then
    print_warning "NVIDIA Docker runtime not fully functional"
    print_info "Install: sudo apt-get install -y nvidia-docker2"
    print_info "Configure: sudo systemctl restart docker"
  else
    print_success "NVIDIA Docker runtime working"
  fi
  
  # Detect actual device if on Jetson
  if [[ -f /proc/device-tree/model ]]; then
    detect_jetson_device
    print_success "Detected Jetson device: $JETSON_DEVICE"
  fi
}

build_docker_image() {
  print_info "Building Docker image for Jetson $JETSON_DEVICE..."
  
  # Update tag with device suffix
  DOCKER_IMAGE="stereo-slam:jetson-orin-${JETSON_DEVICE}"
  
  # Memory limits for Jetson (prevent build OOM)
  local build_memory="3g"
  if [[ "$JETSON_DEVICE" == "nano" ]]; then
    build_memory="2g"
  fi
  
  docker build \
    -f "${PROJECT_DIR}/docker/Dockerfile.mine" \
    --build-arg BASE_IMAGE="nvcr.io/nvidia/isaac/ros:jetson-orin-ros2_humble" \
    -t "$DOCKER_IMAGE" \
    --memory="$build_memory" \
    "${PROJECT_DIR}" || {
    print_error "Docker build failed"
    exit 1
  }
  
  print_success "Docker image built: $DOCKER_IMAGE"
}

get_jetson_memory() {
  local total_mem=$(free -b | awk 'NR==2 {print $2}')
  local available_mem=$(free -b | awk 'NR==2 {print $7}')
  echo "Total: $(numfmt --to=iec-i --suffix=B $total_mem) | Available: $(numfmt --to=iec-i --suffix=B $available_mem)"
}

optimize_jetson_for_pipeline() {
  print_info "Optimizing Jetson for pipeline..."
  
  # Set buffer pool size based on available memory
  if [[ "$JETSON_DEVICE" == "nano" ]]; then
    # Nano 4GB: use smaller buffer pool
    export NITROS_NUM_BUFFERS=4
    export NITROS_BUFFER_SIZE=4194304  # 4MB per buffer
    print_warning "Jetson Nano: reduced buffer pool (4 buffers, 4MB each)"
  else
    # NX 8GB: can use more buffers
    export NITROS_NUM_BUFFERS=8
    export NITROS_BUFFER_SIZE=8388608  # 8MB per buffer
    print_success "Jetson NX: standard buffer pool (8 buffers, 8MB each)"
  fi
  
  # Enable Jetson clocks for maximum performance (optional - higher power/thermal)
  # Uncomment to enable:
  # sudo /usr/bin/jetson_clocks
}

run_pipeline() {
  print_info "Starting pipeline container..."
  
  local docker_args=(
    "--rm"
    "--runtime=nvidia"
    "-e" "NVIDIA_VISIBLE_DEVICES=all"
    "-e" "CUDA_VISIBLE_DEVICES=0"
    "-e" "NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility"
    "-e" "CUDA_FORCE_PTX_JIT=0"
    "-e" "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    "-e" "NITROS_NUM_BUFFERS=${NITROS_NUM_BUFFERS:-8}"
    "-e" "NITROS_BUFFER_SIZE=${NITROS_BUFFER_SIZE:-8388608}"
    "-e" "JETSON_DEVICE=${JETSON_DEVICE}"
    # GPU device access
    "--device" "/dev/nvhost-ctrl"
    "--device" "/dev/nvhost-ctrl-gp1"
    "--device" "/dev/nvhost-gpu"
    "--device" "/dev/nvmap"
    # Volume mounts
    "-v" "${PROJECT_DIR}:/workspaces/isaac_ros-dev"
  )
  
  # Add bag path if provided
  if [[ -n "$BAG_PATH" ]]; then
    docker_args+=("-v" "${BAG_PATH}:/rosbag:ro")
    docker_args+=("-e" "BAG_PATH=/rosbag")
  fi
  
  # Add interactive terminal
  docker_args+=("-it")
  
  print_success "Container memory: $(get_jetson_memory)"
  print_info "Running: docker run ${docker_args[@]} $DOCKER_IMAGE bash"
  echo ""
  
  docker run "${docker_args[@]}" "$DOCKER_IMAGE" bash
}

print_welcome() {
  echo ""
  echo "=========================================="
  echo "  Stereo SLAM for Jetson Deployment"
  echo "=========================================="
  echo "Device:    Jetson Orin $JETSON_DEVICE"
  echo "Image:     $DOCKER_IMAGE"
  echo "Workspace: $PROJECT_DIR"
  echo "=========================================="
  echo ""
}

main() {
  print_welcome
  check_prerequisites
  
  if [[ "$BUILD_IMAGE" == true ]]; then
    build_docker_image
  fi
  
  optimize_jetson_for_pipeline
  
  # Inside container, user needs to run:
  # cd /workspaces/isaac_ros-dev && colcon build --symlink-install
  # Then: ./scripts/run_full_pipeline.bash
  
  print_info "When inside container, run:"
  echo "  cd /workspaces/isaac_ros-dev"
  echo "  colcon build --symlink-install --parallel-workers 2"
  echo "  ./scripts/run_full_pipeline.bash"
  echo ""
  
  run_pipeline
}

main "$@"
