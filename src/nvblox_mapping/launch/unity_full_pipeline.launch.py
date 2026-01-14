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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_vslam = LaunchConfiguration("enable_vslam")
    enable_nvblox = LaunchConfiguration("enable_nvblox")
    ess_threshold_1 = LaunchConfiguration("ess_threshold_1")
    ess_threshold_2 = LaunchConfiguration("ess_threshold_2")
    ess_threshold_3 = LaunchConfiguration("ess_threshold_3")
    ess_threshold_4 = LaunchConfiguration("ess_threshold_4")
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
        namespace="kevin",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level", "kevin.visual_slam_node:=info",
            "--log-level", "tf2_ros:=debug",
            "--log-level", "tf2:=debug"
        ],
        remappings=[("/tf", "/kevin/tf"), ("/tf_static", "/kevin/tf_static")],
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
                    {"input_compressed_topic": "/kevin/stereo_camera/left/rgb/compressed"},
                    {"input_camera_info_topic": "/kevin/stereo_camera/left/camera_info"},
                    {"output_bridge_image_topic": "/kevin/stereo_camera/left/nitros_bridge"},
                    {"output_camera_info_topic": "/kevin/stereo_camera/left/camera_info_decoded"},
                    {"output_frame_id": "stereo_left_optical_frame"},  # Try frame_id override
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
                    {"input_compressed_topic": "/kevin/stereo_camera/right/rgb/compressed"},
                    {"input_camera_info_topic": "/kevin/stereo_camera/right/camera_info"},
                    {"output_bridge_image_topic": "/kevin/stereo_camera/right/nitros_bridge"},
                    {"output_camera_info_topic": "/kevin/stereo_camera/right/camera_info_decoded"},
                    {"output_frame_id": "stereo_right_optical_frame"},  # Try frame_id override
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
                    {"input_bridge_topic": "/kevin/stereo_camera/left/nitros_bridge"},
                    {"output_image_topic": "/kevin/stereo_camera/left/image_raw"},
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
                    {"input_bridge_topic": "/kevin/stereo_camera/right/nitros_bridge"},
                    {"output_image_topic": "/kevin/stereo_camera/right/image_raw"},
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
                    ("camera_info", "/kevin/stereo_camera/left/camera_info_decoded"),
                    ("image", "/kevin/stereo_camera/left/image_raw"),
                    ("resize/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("resize/image", "/kevin/stereo_camera/left/image_resize"),
                ],
            ),
            # Right image resize
            ComposableNode(
                package="isaac_ros_image_proc",
                plugin="nvidia::isaac_ros::image_proc::ResizeNode",
                name="right_resize",
                parameters=[{"use_sim_time": use_sim_time}, resize_params_path],
                remappings=[
                    ("camera_info", "/kevin/stereo_camera/right/camera_info_decoded"),
                    ("image", "/kevin/stereo_camera/right/image_raw"),
                    ("resize/camera_info", "/kevin/stereo_camera/right/camera_info_resize"),
                    ("resize/image", "/kevin/stereo_camera/right/image_resize"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 3.5: Camera Info Frame ID Correction (for VSLAM)
            # Republish camera_info with corrected frame_id to optical frames
            # -----------------------------------------------------------------
            # Left camera info: stereo_left_link → stereo_left_optical_frame
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::CameraInfoHeaderRewriterNode",
                name="left_camera_info_corrector",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/kevin/stereo_camera/left/camera_info_resize"},
                    {"output_topic": "/kevin/stereo_camera/left/camera_info_optical"},
                    {"target_frame_id": "stereo_left_optical_frame"},
                ],
            ),
            # Right camera info: stereo_right_link → stereo_right_optical_frame
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::CameraInfoHeaderRewriterNode",
                name="right_camera_info_corrector",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/kevin/stereo_camera/right/camera_info_resize"},
                    {"output_topic": "/kevin/stereo_camera/right/camera_info_optical"},
                    {"target_frame_id": "stereo_right_optical_frame"},
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 3.5: Camera Info Frame ID Rewriter (for VSLAM optical frames)
            # -----------------------------------------------------------------
            # Left camera info: stereo_left_link → stereo_left_optical_frame
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::CameraInfoHeaderRewriterNode",
                name="left_camera_info_rewriter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/kevin/stereo_camera/left/camera_info_resize"},
                    {"output_topic": "/kevin/stereo_camera/left/camera_info_optical"},
                    {"target_frame_id": "stereo_left_optical_frame"},
                ],
            ),
            # Right camera info: stereo_right_link → stereo_right_optical_frame
            ComposableNode(
                package="unity_nitros_bridge",
                plugin="unity_nitros_bridge::CameraInfoHeaderRewriterNode",
                name="right_camera_info_rewriter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": "/kevin/stereo_camera/right/camera_info_resize"},
                    {"output_topic": "/kevin/stereo_camera/right/camera_info_optical"},
                    {"target_frame_id": "stereo_right_optical_frame"},
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 4: ESS Disparity (TensorRT - NITROS) with four thresholds
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_ess",
                plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
                name="ess_disparity_node_t1",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    ess_params_path,
                    {"engine_file_path": 
                        "/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine"
                    },
                    {"input_layer_width": 960},
                    {"input_layer_height": 576},
                    {"threshold": ess_threshold_1},
                ],
                remappings=[
                    ("left/image_rect", "/kevin/stereo_camera/left/image_resize"),
                    ("left/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("right/image_rect", "/kevin/stereo_camera/right/image_resize"),
                    ("right/camera_info", "/kevin/stereo_camera/right/camera_info_resize"),
                    ("disparity", "/kevin/disparity_t1"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_ess",
                plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
                name="ess_disparity_node_t2",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    ess_params_path,
                    {"engine_file_path": 
                        "/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine"
                    },
                    {"input_layer_width": 960},
                    {"input_layer_height": 576},
                    {"threshold": ess_threshold_2},
                ],
                remappings=[
                    ("left/image_rect", "/kevin/stereo_camera/left/image_resize"),
                    ("left/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("right/image_rect", "/kevin/stereo_camera/right/image_resize"),
                    ("right/camera_info", "/kevin/stereo_camera/right/camera_info_resize"),
                    ("disparity", "/kevin/disparity_t2"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_ess",
                plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
                name="ess_disparity_node_t3",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    ess_params_path,
                    {"engine_file_path": 
                        "/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine"
                    },
                    {"input_layer_width": 960},
                    {"input_layer_height": 576},
                    {"threshold": ess_threshold_3},
                ],
                remappings=[
                    ("left/image_rect", "/kevin/stereo_camera/left/image_resize"),
                    ("left/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("right/image_rect", "/kevin/stereo_camera/right/image_resize"),
                    ("right/camera_info", "/kevin/stereo_camera/right/camera_info_resize"),
                    ("disparity", "/kevin/disparity_t3"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_ess",
                plugin="nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode",
                name="ess_disparity_node_t4",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    ess_params_path,
                    {"engine_file_path": 
                        "/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine"
                    },
                    {"input_layer_width": 960},
                    {"input_layer_height": 576},
                    {"threshold": ess_threshold_4},
                ],
                remappings=[
                    ("left/image_rect", "/kevin/stereo_camera/left/image_resize"),
                    ("left/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("right/image_rect", "/kevin/stereo_camera/right/image_resize"),
                    ("right/camera_info", "/kevin/stereo_camera/right/camera_info_resize"),
                    ("disparity", "/kevin/disparity_t4"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 4.5: Disparity to Depth conversion (for nvblox/tests)
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode",
                name="disparity_to_depth_node_t1",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("disparity", "/kevin/disparity_t1"),
                    ("camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("depth", "/kevin/depth_t1"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode",
                name="disparity_to_depth_node_t2",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("disparity", "/kevin/disparity_t2"),
                    ("camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("depth", "/kevin/depth_t2"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode",
                name="disparity_to_depth_node_t3",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("disparity", "/kevin/disparity_t3"),
                    ("camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("depth", "/kevin/depth_t3"),
                ],
            ),
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode",
                name="disparity_to_depth_node_t4",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("disparity", "/kevin/disparity_t4"),
                    ("camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("depth", "/kevin/depth_t4"),
                ],
            ),
            
            # -----------------------------------------------------------------
            # Stage 5: Visual SLAM (cuVSLAM - NITROS)
            # -----------------------------------------------------------------
            ComposableNode(
                package="isaac_ros_visual_slam",
                plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
                name="visual_slam_node",
                condition=IfCondition(enable_vslam),
                parameters=[{"use_sim_time": use_sim_time}, vslam_params_path],
                remappings=[
                    # TF remappings: /tf → tf (relative) → /kevin/tf (namespaced)
                    # This applies to both TF subscriptions AND tf2 broadcaster outputs
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    # Input topics
                    ("visual_slam/image_0", "/kevin/stereo_camera/left/image_resize"),
                    ("visual_slam/camera_info_0", "/kevin/stereo_camera/left/camera_info_optical"),
                    ("visual_slam/image_1", "/kevin/stereo_camera/right/image_resize"),
                    ("visual_slam/camera_info_1", "/kevin/stereo_camera/right/camera_info_optical"),
                    ("visual_slam/imu", "/kevin/mavros/imu/data"),
                    # Output topics
                    ("visual_slam/tracking/odometry", "/kevin/visual_slam/tracking/odometry"),
                    ("visual_slam/tracking/vo_pose", "/kevin/visual_slam/tracking/vo_pose"),
                    ("visual_slam/tracking/vo_pose_covariance", "/kevin/visual_slam/tracking/vo_pose_covariance"),
                    ("visual_slam/tracking/slam_path", "/kevin/visual_slam/tracking/slam_path"),
                    ("visual_slam/tracking/vo_path", "/kevin/visual_slam/tracking/vo_path"),
                    ("visual_slam/status", "/kevin/visual_slam/status"),
                    ("visual_slam/vis/observations_cloud", "/kevin/visual_slam/vis/observations_cloud"),
                    ("visual_slam/vis/landmarks_cloud", "/kevin/visual_slam/vis/landmarks_cloud"),
                    ("visual_slam/vis/loop_closure_cloud", "/kevin/visual_slam/vis/loop_closure_cloud"),
                    ("visual_slam/vis/pose_graph_nodes", "/kevin/visual_slam/vis/pose_graph_nodes"),
                    ("visual_slam/vis/pose_graph_edges", "/kevin/visual_slam/vis/pose_graph_edges"),
                    ("visual_slam/vis/pose_graph_edges2", "/kevin/visual_slam/vis/pose_graph_edges2"),
                ],
            ),
            # To toggle VSLAM on/off, use enable_vslam launch arg
        ],
    )

    # =========================================================================
    # Nvblox Container (separate process to avoid GPU initialization conflicts)
    # nvblox subscribes to standard ROS2 topics (auto-converted from NITROS)
    # =========================================================================
    nvblox_container = ComposableNodeContainer(
        name="nvblox_container",
        namespace="kevin",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=IfCondition(enable_nvblox),
        remappings=[("/tf", "/kevin/tf"), ("/tf_static", "/kevin/tf_static")],
        composable_node_descriptions=[
            ComposableNode(
                package="nvblox_ros",
                plugin="nvblox::NvbloxNode",
                name="nvblox_node",
                parameters=[{"use_sim_time": use_sim_time}, nvblox_params_path],
                remappings=[
                    # Input topics
                    ("camera_0/depth/image", "/kevin/depth"),
                    ("camera_0/depth/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    ("camera_0/color/image", "/kevin/stereo_camera/left/image_resize"),
                    ("camera_0/color/camera_info", "/kevin/stereo_camera/left/camera_info_resize"),
                    # Output topics
                    ("mesh", "/kevin/nvblox/mesh"),
                    ("static_esdf_pointcloud", "/kevin/nvblox/static_esdf_pointcloud"),
                    ("static_occupancy_layer", "/kevin/nvblox/static_occupancy_layer"),
                    ("map_slice", "/kevin/nvblox/map_slice"),
                    ("back_projected_depth", "/kevin/nvblox/back_projected_depth"),
                    ("dynamic_points", "/kevin/nvblox/dynamic_points"),
                    ("combined_esdf_pointcloud", "/kevin/nvblox/combined_esdf_pointcloud"),
                ],
            ),
        ],
    )

    # =========================================================================
    # Static Transform Publishers
    # 
    # The rosbag provides the complete robot TF tree on /kevin/tf:
    #   map -> odom -> base_link -> stereo_link -> stereo_left_link
    #                             -> imu_link    -> stereo_right_link
    #
    # We only need to add optical frame transforms for VSLAM:
    #   stereo_left_link -> stereo_left_optical_frame
    #   stereo_right_link -> stereo_right_optical_frame
    # =========================================================================
    
    # Quaternion for link -> optical frame rotation
    # This rotates from ROS body convention to camera optical convention
    # Roll: -90°, Pitch: 0°, Yaw: -90° => qx=0.5, qy=-0.5, qz=0.5, qw=-0.5
    optical_qx = 0.5
    optical_qy = -0.5
    optical_qz = 0.5
    optical_qw = 0.5
    
    # stereo_left_link -> stereo_left_optical_frame
    left_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_optical_tf",
        namespace="kevin",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", str(optical_qx), "--qy", str(optical_qy), 
            "--qz", str(optical_qz), "--qw", str(optical_qw),
            "--frame-id", "stereo_left_link",
            "--child-frame-id", "stereo_left_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "/kevin/tf"), ("/tf_static", "/kevin/tf_static")],
    )
    
    right_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_optical_tf",
        namespace="kevin",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", str(optical_qx), "--qy", str(optical_qy), 
            "--qz", str(optical_qz), "--qw", str(optical_qw),
            "--frame-id", "stereo_right_link",
            "--child-frame-id", "stereo_right_optical_frame",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "/kevin/tf"), ("/tf_static", "/kevin/tf_static")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time from rosbag/clock",
            ),
            DeclareLaunchArgument(
                "enable_vslam",
                default_value="false",
                description="Enable cuVSLAM node",
            ),
            DeclareLaunchArgument(
                "enable_nvblox",
                default_value="false",
                description="Enable Nvblox node",
            ),
            DeclareLaunchArgument(
                "ess_threshold_1",
                default_value="0.0",
                description="ESS threshold for disparity stream 1",
            ),
            DeclareLaunchArgument(
                "ess_threshold_2",
                default_value="0.35",
                description="ESS threshold for disparity stream 2",
            ),
            DeclareLaunchArgument(
                "ess_threshold_3",
                default_value="0.75",
                description="ESS threshold for disparity stream 3",
            ),
            DeclareLaunchArgument(
                "ess_threshold_4",
                default_value="0.99",
                description="ESS threshold for disparity stream 4",
            ),
            # Static TF publishers for optical frames only
            # (Robot structure TFs come from rosbag on /kevin/tf)
            left_optical_tf,
            right_optical_tf,
            # Containers have namespace="kevin" set directly, no need for GroupAction
            ess_container,
            nvblox_container,
        ]
    )
