# Stereo SLAM 3D Mapping System

GPU-accelerated stereo SLAM and 3D reconstruction pipeline using NVIDIA Isaac ROS on Jetson Orin NX.

## Overview

This system processes stereo camera images and IMU data to:
- Generate depth maps using ESS (Efficient Stereo Solver)
- Perform visual SLAM (cuVSLAM)
- Build 3D voxel maps (Nvblox)
- Stream data from Unity simulation via ROS-TCP-Endpoint

## Hardware Requirements

- **Platform**: NVIDIA Jetson Orin NX (or AGX Orin)
- **JetPack**: 6.0+ (Ubuntu 22.04 + ROS 2 Humble)
- **CUDA**: 12.x
- **Memory**: 8GB+ unified memory
- **Storage**: 32GB+ for Docker images

## Quick Start

### 1. Prerequisites

```bash
# Install Docker (if not already installed)
sudo apt-get update
sudo apt-get install docker.io nvidia-container-toolkit

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker  # Or logout/login
```

### 2. Clone and Setup

```bash
# Clone this repository
git clone <your-repo-url> ~/stereo_ws/stereo-slam-3Dmapping
cd ~/stereo_ws/stereo-slam-3Dmapping

# Set workspace environment variable
export ISAAC_ROS_WS=$(pwd)
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
```

### 3. Launch Development Container

The Isaac ROS development container includes all dependencies (CUDA, CV-CUDA, cuVSLAM, etc.):

```bash
# Launch the Isaac ROS dev container
src/isaac_ros_common/scripts/run_dev.sh
```

This will:
- Pull the official Isaac ROS Docker image (~8GB)
- Mount your workspace at `/workspaces/isaac_ros-dev`
- Enable GPU access
- Drop you into a container shell

### 4. Build Inside Container

Once inside the container:

```bash
# Build the workspace
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-up-to nvblox_mapping unity_nitros_bridge

# Source the workspace
source install/setup.bash
```

### 5. Run the Pipeline

```bash
# Launch the full GPU pipeline
./scripts/run_full_pipeline.bash
```

## Project Structure

```
stereo-slam-3Dmapping/
├── scripts/
│   ├── run_full_pipeline.bash       # Main launcher
│   ├── play_stereo_imu_bag_loop.bash # Rosbag playback
│   ├── run_foxglove_bridge.bash     # Visualization bridge
│   └── diagnose_tf.bash             # TF tree diagnostics
├── src/
│   ├── isaac_ros_common/            # Isaac ROS utilities & Docker scripts
│   ├── isaac_ros_nitros/            # Zero-copy framework
│   ├── isaac_ros_dnn_stereo_depth/  # ESS depth estimation
│   ├── isaac_ros_visual_slam/       # cuVSLAM
│   ├── isaac_ros_nvblox/            # 3D mapping
│   ├── isaac_ros_image_pipeline/    # Image processing utilities
│   ├── nvblox_mapping/              # Launch files & configs
│   ├── unity_nitros_bridge/         # Unity-ROS bridge
│   └── ROS-TCP-Endpoint/            # Unity TCP communication
└── README.md                        # This file
```

## Configuration Files

Located in `src/nvblox_mapping/config/`:

- **`vslam_params.yaml`**: cuVSLAM parameters (features, tracking)
- **`ess_params.yaml`**: ESS depth estimation settings
- **`nvblox_params.yaml`**: Voxel mapping configuration
- **`decoder_params.yaml`**: Image decompression settings
- **`resize_params.yaml`**: Image preprocessing

## Launch Files

### Main Pipeline
```bash
ros2 launch nvblox_mapping unity_full_pipeline.launch.py
```

Components launched:
1. **ROS-TCP-Endpoint**: Receives data from Unity
2. **NITROS JPEG Decoders**: GPU-accelerated image decompression
3. **Image Processors**: Resizing, rectification
4. **ESS Disparity Node**: Stereo depth estimation
5. **cuVSLAM**: Visual SLAM
6. **Nvblox**: 3D reconstruction

## Common Workflows

### Testing with Rosbag

```bash
# Inside container
./scripts/play_stereo_imu_bag_loop.bash

# In another terminal (host)
export ISAAC_ROS_WS=~/stereo_ws/stereo-slam-3Dmapping
src/isaac_ros_common/scripts/run_dev.sh
# Inside container
source install/setup.bash
./scripts/run_full_pipeline.bash
```

### Visualization with Foxglove

```bash
# Launch Foxglove bridge (host)
./scripts/run_foxglove_bridge.bash

# Open browser to http://localhost:8765
```

### Rebuilding Specific Packages

```bash
# Inside container
colcon build --symlink-install --packages-select nvblox_mapping
source install/setup.bash
```

## Troubleshooting

### Container won't start
```bash
# Check Docker is running
sudo systemctl status docker

# Check user is in docker group
groups | grep docker

# Re-login if needed
newgrp docker
```

### Build failures
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### GPU not accessible
```bash
# Check NVIDIA runtime
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

### TF tree issues
```bash
# Inside container with pipeline running
./scripts/diagnose_tf.bash
```

## Performance Tuning

### Memory Constraints (8GB Jetson)

```bash
# Build with limited parallelism
colcon build --symlink-install --parallel-workers 2

# Reduce voxel resolution in nvblox_params.yaml
voxel_size: 0.1  # Increase from 0.05 to reduce memory
```

### Processing Rate

Adjust in launch file or configs:
- ESS input resolution: Lower for faster processing
- VSLAM feature count: Reduce for speed
- Nvblox integration rate: Lower frequency

## Development

### Adding New Nodes

1. Create package in `src/`
2. Add to `colcon build` command
3. Update launch file in `nvblox_mapping/launch/`

### Modifying Parameters

Edit YAML files in `src/nvblox_mapping/config/` and rebuild:

```bash
colcon build --symlink-install --packages-select nvblox_mapping
```

## References

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Nvblox](https://github.com/nvidia-isaac/nvblox)
- [cuVSLAM](https://docs.nvidia.com/isaac/packages/visual_slam/doc/index.html)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

## License

See individual package licenses in `src/` subdirectories.
