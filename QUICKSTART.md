# Quick Start Guide

## First Time Setup (5 minutes)

### 1. Set Environment Variable

```bash
cd ~/stereo_ws/stereo-slam-3Dmapping
export ISAAC_ROS_WS=$(pwd)
echo "export ISAAC_ROS_WS=$(pwd)" >> ~/.bashrc
source ~/.bashrc
```

### 2. Launch Development Container

```bash
# This pulls the Isaac ROS Docker image (~8GB download)
src/isaac_ros_common/scripts/run_dev.sh
```

**What this does:**
- Downloads official Isaac ROS Docker image with all dependencies
- Mounts your workspace at `/workspaces/isaac_ros-dev`
- Enables GPU access
- Gives you a container shell

### 3. Build Inside Container

Once you see `admin@container:~$` prompt:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

**First build takes ~30-45 minutes on Jetson Orin NX**

### 4. Run the Pipeline

```bash
./scripts/run_full_pipeline.bash
```

## Daily Workflow

### Start Container
```bash
cd ~/stereo_ws/stereo-slam-3Dmapping
src/isaac_ros_common/scripts/run_dev.sh
```

### Inside Container
```bash
cd /workspaces/isaac_ros-dev
source install/setup.bash
./scripts/run_full_pipeline.bash
```

## Multiple Terminals

### Terminal 1: Pipeline
```bash
cd ~/stereo_ws/stereo-slam-3Dmapping
src/isaac_ros_common/scripts/run_dev.sh
# Inside container:
source install/setup.bash
./scripts/run_full_pipeline.bash
```

### Terminal 2: Rosbag Playback
```bash
cd ~/stereo_ws/stereo-slam-3Dmapping
src/isaac_ros_common/scripts/run_dev.sh
# Inside container:
source install/setup.bash
./scripts/play_stereo_imu_bag_loop.bash
```

### Terminal 3: Monitoring (on host)
```bash
docker logs -f <container-name>
# Or use Foxglove Studio at http://localhost:8765
```

## Common Commands

### Check if container is running
```bash
docker ps
```

### Attach to running container
```bash
docker exec -it <container-name> /bin/bash
```

### Stop container
```bash
docker stop <container-name>
```

### Clean rebuild
```bash
# Inside container
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Troubleshooting

### "ISAAC_ROS_WS not set"
```bash
export ISAAC_ROS_WS=~/stereo_ws/stereo-slam-3Dmapping
```

### "Permission denied" for Docker
```bash
sudo usermod -aG docker $USER
# Then logout/login or:
newgrp docker
```

### Out of memory during build
```bash
colcon build --symlink-install --parallel-workers 1
```

### Need to rebuild one package
```bash
colcon build --symlink-install --packages-select nvblox_mapping
```

## Next Steps

- See [README.md](README.md) for full documentation
- Edit configs in `src/nvblox_mapping/config/`
- Add rosbags to test in `bags/` directory
- Connect Unity simulation on port 10000
