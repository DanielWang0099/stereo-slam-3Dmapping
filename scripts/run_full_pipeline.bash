#!/usr/bin/env bash
# =============================================================================
# Run Full GPU Pipeline with Logging
# =============================================================================
# Launches the unity_full_pipeline and captures all output to a log file.
# The log file is overwritten each time this script is started.
#
# Usage:
#   ./run_full_pipeline.bash
#   ./run_full_pipeline.bash --use_sim_time:=false
#
# Environment variables:
#   LOG_FILE  - Custom log file path (default: /workspaces/isaac_ros-dev/logs/pipeline.log)
#   LOG_DIR   - Custom log directory (default: /workspaces/isaac_ros-dev/logs)
# =============================================================================

set -eo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${SCRIPT_DIR}/.."
LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/pipeline.log}"
LAUNCH_PACKAGE="nvblox_mapping"
LAUNCH_FILE="unity_full_pipeline.launch.py"

# Colors for terminal output (only for initial messages)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${CYAN}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Create log directory if it doesn't exist
mkdir -p "${LOG_DIR}"

# Overwrite log file (truncate to empty)
: > "${LOG_FILE}"

print_info "=============================================="
print_info "  Isaac ROS Full GPU Pipeline Launcher"
print_info "=============================================="
print_info "Log file: ${LOG_FILE}"
print_info "Launch:   ${LAUNCH_PACKAGE} ${LAUNCH_FILE}"
print_info "=============================================="

# Source ROS2 workspace
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    print_info "Sourcing workspace: ${WORKSPACE_DIR}/install/setup.bash"
    source "${WORKSPACE_DIR}/install/setup.bash"
else
    print_error "Workspace not built! Run 'colcon build' first."
    exit 1
fi

# Enable verbose TF debugging
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
print_info "TF2 debug logging enabled"

# Write header to log file
{
    echo "=============================================="
    echo "  Isaac ROS Full GPU Pipeline Log"
    echo "=============================================="
    echo "Started at: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "Launch: ${LAUNCH_PACKAGE} ${LAUNCH_FILE}"
    echo "Arguments: $*"
    echo "=============================================="
    echo ""
} >> "${LOG_FILE}"

print_success "Starting pipeline... (logging to ${LOG_FILE})"
print_info "Press Ctrl+C to stop"
echo ""

# Trap to handle cleanup on exit
cleanup() {
    echo ""
    print_warning "Shutting down pipeline..."
    {
        echo ""
        echo "=============================================="
        echo "  Pipeline stopped at: $(date '+%Y-%m-%d %H:%M:%S')"
        echo "=============================================="
    } >> "${LOG_FILE}"
    print_success "Log saved to: ${LOG_FILE}"
}
trap cleanup EXIT

# Launch the pipeline, redirect stdout and stderr to log file
# Using unbuffered output to ensure real-time logging
exec ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" "$@" \
    2>&1 | tee -a "${LOG_FILE}"
