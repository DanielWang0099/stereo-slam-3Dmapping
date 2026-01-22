# Manual Setup Steps for Isaac ROS in Container

After starting the container with `./scripts/deploy_jetson.bash --bag-path $(pwd)/bags/bag_with_imu_stereo`, follow these steps:

## Step 1: Add NVIDIA Jetson Repositories
```bash
apt-key adv --fetch-keys https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
echo "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
apt-get update
```

## Step 2: Install VPI (Vision Programming Interface)
```bash
apt-get install -y libnvvpi3 vpi3-dev
```

## Step 3: Fix magic_enum (Use Bundled Version)
```bash
# Remove any broken installations
rm -rf /usr/include/magic_enum* /usr/local/include/magic_enum* /tmp/magic_enum

# Find the bundled version
find /workspaces/isaac_ros-dev/src -name "magic_enum.hpp" -type f

# Copy the correct version from isaac_ros_gxf
cp /workspaces/isaac_ros-dev/src/isaac_ros_nitros/isaac_ros_gxf/gxf/common/include/magic_enum.hpp /usr/include/

# Verify it's there
ls -la /usr/include/magic_enum.hpp
```

## Step 4: Build Workspace
```bash
# Navigate to workspace
cd /workspaces/isaac_ros-dev

# Source ROS2
source /opt/ros/humble/setup.bash

# Clean previous builds
rm -rf build/ install/ log/

# Build with NVTX disabled (this will take 30-60 minutes)
colcon build --symlink-install --parallel-workers 2 --cmake-args -DUSE_NVTX=OFF
```

## Step 5: Test Rosbag Playback
```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check rosbag info
ros2 bag info /rosbag

# Play the bag
ros2 bag play /rosbag --loop
```

## Step 6: (Optional) Run Full Pipeline
Only if all Isaac ROS packages built successfully:
```bash
./scripts/run_full_pipeline.bash
```

---

## Common Issues

### Issue 1: VPI packages not found
**Solution:** Verify Jetson repositories were added correctly. Check with:
```bash
apt-cache search vpi | grep nvidia
```

### Issue 2: magic_enum compilation errors
**Symptoms:** Errors like `error: 'enum_subtype' has not been declared`
**Solution:** Ensure you copied the correct magic_enum.hpp from isaac_ros_gxf

### Issue 3: CUDA errors about CUgreenCtx
**Symptoms:** `error: 'CUgreenCtx' has not been declared`
**Solution:** This is a CUDA version compatibility issue - these errors may persist but shouldn't block basic builds

### Issue 4: Build takes too long or runs out of memory
**Solution:** 
- Use `--parallel-workers 1` instead of 2
- Close other applications
- Monitor with `htop` in another terminal

---

## Quick Command Reference

**Rebuild specific package:**
```bash
colcon build --symlink-install --parallel-workers 2 --packages-select <package_name> --cmake-args -DUSE_NVTX=OFF
```

**Build only packages you need:**
```bash
colcon build --symlink-install --parallel-workers 2 --packages-up-to nvblox_mapping unity_nitros_bridge --cmake-args -DUSE_NVTX=OFF
```

**Skip problematic packages:**
```bash
colcon build --symlink-install --parallel-workers 2 --packages-skip isaac_ros_ess isaac_ros_visual_slam --cmake-args -DUSE_NVTX=OFF
```
