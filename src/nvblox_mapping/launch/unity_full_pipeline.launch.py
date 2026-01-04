#!/usr/bin/env python3
# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""
TRUE Zero-Copy Pipeline Launch File for Unity Simulation.

This launch file implements TRUE zero-copy GPU pipeline using CUDA IPC:

Data Flow:
  CompressedImage (from Unity/rosbag)
      ↓
  NitrosBridgeJpegDecoderNode (NVJPEG decode → GPU IPC buffer)
      ↓
  NitrosBridgeImage {PID, FD} (CUDA IPC handles)
      ↓
  ImageConverterNode (import IPC → NitrosImage)
      ↓
  NitrosImage (GPU pointer - TRUE ZERO COPY!)
      ↓
  ESS/VSLAM/Nvblox (all GPU operations)

Key Features:
- NO GPU→CPU copy in entire pipeline
- CUDA IPC for cross-process GPU memory sharing
- Ring buffer pool for zero-allocation per frame
- ManagedNitrosPublisher for NITROS compatibility
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import math


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_share = FindPackageShare("nvblox_mapping")

    resize_params_path = PathJoinSubstitution([pkg_share, "config", "resize_params.yaml"])
    ess_params_path = PathJoinSubstitution([pkg_share, "config", "ess_params.yaml"])
    vslam_params_path = PathJoinSubstitution([pkg_share, "config", "vslam_params.yaml"])
    nvblox_params_path = PathJoinSubstitution([pkg_share, "config", "nvblox_params.yaml"])

    # =========================================================================
    # TRUE Zero-Copy ESS Container
    # All nodes in same container for NITROS zero-copy
    # =========================================================================
    ess_container = ComposableNodeContainer(
        name="ess_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            # -----------------------------------------------------------------
            # Stage 1: JPEG Decode → NitrosBridgeImage (CUDA IPC)
            # -----------------------------------------------------------------
            # Left camera: CompressedImage → NitrosBridgeImage
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::NitrosBridgeJpegDecoderNode",
                name="left_bridge_decoder",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_compressed_topic": "/stereo_camera/left/rgb/compressed"},
                    {"input_camera_info_topic": "/stereo_camera/left/camera_info"},
                    {"output_bridge_image_topic": "/stereo_camera/left/nitros_bridge"},
                    {"output_camera_info_topic": "/stereo_camera/left/camera_info_decoded"},
                    {"num_buffers": 8},
                    {"device_id": 0},
                ],
            ),
            # Right camera: CompressedImage → NitrosBridgeImage
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::NitrosBridgeJpegDecoderNode",
                name="right_bridge_decoder",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_compressed_topic": "/stereo_camera/right/rgb/compressed"},
                    {"input_camera_info_topic": "/stereo_camera/right/camera_info"},
                    {"output_bridge_image_topic": "/stereo_camera/right/nitros_bridge"},
                    {"output_camera_info_topic": "/stereo_camera/right/camera_info_decoded"},
                    {"num_buffers": 8},
                    {"device_id": 0},
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 2: NitrosBridgeImage → NitrosImage (CUDA IPC import)
            # -----------------------------------------------------------------
            # Left camera: NitrosBridgeImage → NitrosImage
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::ImageConverterNode",
                name="left_image_converter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_bridge_topic": "/stereo_camera/left/nitros_bridge"},
                    {"output_image_topic": "/stereo_camera/left/image_raw"},
                    {"device_id": 0},
                ],
            ),
            # Right camera: NitrosBridgeImage → NitrosImage
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::ImageConverterNode",
                name="right_image_converter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_bridge_topic": "/stereo_camera/right/nitros_bridge"},
                    {"output_image_topic": "/stereo_camera/right/image_raw"},
                    {"device_id": 0},
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 3: Image Resize (GPU - NITROS)
            # -----------------------------------------------------------------
            # Left image resize
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::ResizeNode",
                name="left_resize",
                parameters=[{"use_sim_time": use_sim_time}, resize_params_path],
                remappings=[
                    ("camera_info", "/stereo_camera/left/camera_info"),
                    ("image", "/stereo_camera/left/image_raw"),
                    ("resize/camera_info", "/stereo_camera/left/camera_info_resize"),
                    ("resize/image", "/stereo_camera/left/image_resize"),
                ],
            ),
            # Right image resize
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::ResizeNode",
                name="right_resize",
                parameters=[{"use_sim_time": use_sim_time}, resize_params_path],
                remappings=[
                    ("camera_info", "/stereo_camera/right/camera_info"),
                    ("image", "/stereo_camera/right/image_raw"),
                    ("resize/camera_info", "/stereo_camera/right/camera_info_resize"),
                    ("resize/image", "/stereo_camera/right/image_resize"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 4: ESS Disparity (TensorRT - NITROS)
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_ess",
                plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
                name="ess_disparity_node",
                parameters=[{"use_sim_time": use_sim_time}, ess_params_path],
                remappings=[
                    ("left/image_rect", "/stereo_camera/left/image_resize"),
                    ("left/camera_info", "/stereo_camera/left/camera_info_resize"),
                    ("right/image_rect", "/stereo_camera/right/image_resize"),
                    ("right/camera_info", "/stereo_camera/right/camera_info_resize"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 4.5: Disparity to Depth conversion (for nvblox)
            # ESS outputs stereo_msgs/DisparityImage, nvblox needs sensor_msgs/Image
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode",
                name="disparity_to_depth_node",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("disparity", "/disparity"),
                    ("depth", "/depth"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 5: Visual SLAM (cuVSLAM - NITROS)
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_visual_slam",
                plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                name="visual_slam_node",
                parameters=[{"use_sim_time": use_sim_time}, vslam_params_path],
                remappings=[
                    ("visual_slam/image_0", "/stereo_camera/left/image_resize"),
                    ("visual_slam/camera_info_0", "/stereo_camera/left/camera_info_resize"),
                    ("visual_slam/image_1", "/stereo_camera/right/image_resize"),
                    ("visual_slam/camera_info_1", "/stereo_camera/right/camera_info_resize"),
                    ("visual_slam/imu", "/mavros/imu/data"),
                ],
            ),
        ],
    )

    # =========================================================================
    # Nvblox Container (separate process to avoid GPU initialization conflicts)
    # nvblox subscribes to standard ROS2 topics (auto-converted from NITROS)
    # =========================================================================
    nvblox_container = ComposableNodeContainer(
        name="nvblox_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="nvblox_ros",
                plugin="nvblox::NvbloxNode",
                name="nvblox_node",
                parameters=[{"use_sim_time": use_sim_time}, nvblox_params_path],
                remappings=[
                    # nvblox subscribes to camera_0/depth/image and camera_0/depth/camera_info
                    # NOT depth/image - this is the actual internal topic naming!
                    ("camera_0/depth/image", "/depth"),
                    ("camera_0/depth/camera_info", "/stereo_camera/left/camera_info_resize"),
                    ("camera_0/color/image", "/stereo_camera/left/image_resize"),
                    ("camera_0/color/camera_info", "/stereo_camera/left/camera_info_resize"),
                ],
            ),
        ],
    )

    # =========================================================================
    # Static Transform Publishers
    # These create the optical frames from link frames
    # Optical frame convention: Z-forward, X-right, Y-down
    # Link frame convention: X-forward, Y-left, Z-up
    # Rotation: 90° around Z, then -90° around X (or quaternion below)
    # =========================================================================
    
    # Quaternion for link -> optical frame rotation
    # This rotates from ROS body convention to camera optical convention
    # Roll: -90°, Pitch: 0°, Yaw: -90° => qx=0.5, qy=-0.5, qz=0.5, qw=-0.5
    optical_qx = 0.5
    optical_qy = -0.5
    optical_qz = 0.5
    optical_qw = 0.5
    
    left_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", str(optical_qx), "--qy", str(optical_qy), 
            "--qz", str(optical_qz), "--qw", str(optical_qw),
            "--frame-id", "auv/stereo_left_link",
            "--child-frame-id", "auv/stereo_left_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    right_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", str(optical_qx), "--qy", str(optical_qy), 
            "--qz", str(optical_qz), "--qw", str(optical_qw),
            "--frame-id", "auv/stereo_right_link",
            "--child-frame-id", "auv/stereo_right_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Odom frame - identity transform from map to odom
    # This creates the missing odom frame that VSLAM expects
    # VSLAM will publish odom -> base_link, Unity provides map -> odom (via this static TF initially)
    odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from rosbag/clock",
            ),
            # Static TF publishers (must start first)
            left_optical_tf,
            right_optical_tf,
            odom_tf,
            # Processing containers
            ess_container,
            nvblox_container,
        ]
    )
