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

cmake_minimum_required(VERSION 3.22.1)
project(unity_nitros_bridge LANGUAGES C CXX CUDA)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# NITROS dependencies
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_managed_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_nitros_camera_info_type REQUIRED)

# NITROS Bridge interfaces (for NitrosBridgeImage with CUDA IPC)
find_package(isaac_ros_nitros_bridge_interfaces REQUIRED)

# CUDA
find_package(CUDAToolkit REQUIRED)

# NVJPEG (part of CUDA toolkit)
find_library(NVJPEG_LIBRARY nvjpeg HINTS ${CUDAToolkit_LIBRARY_DIR})

# Include directories
include_directories(
  include
  ${CUDAToolkit_INCLUDE_DIRS}
)

# =============================================================================
# LEGACY NODE REMOVED - Had GPU→CPU copy, not true zero-copy
# Use NitrosBridgeJpegDecoderNode + ImageConverterNode instead
# =============================================================================

# =============================================================================
# TRUE Zero-Copy NitrosBridge JPEG Decoder Node (CUDA IPC)
# =============================================================================
add_library(nitros_bridge_jpeg_decoder_node SHARED
  src/nitros_bridge_jpeg_decoder_node.cpp
  src/ipc_buffer_manager.cpp
  src/nvjpeg_decoder.cu
)

target_include_directories(nitros_bridge_jpeg_decoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(nitros_bridge_jpeg_decoder_node
  CUDA::cudart
  CUDA::cuda_driver
  ${NVJPEG_LIBRARY}
)

ament_target_dependencies(nitros_bridge_jpeg_decoder_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  isaac_ros_nitros_bridge_interfaces
)

# Set RUNPATH to find GXF libraries at runtime (matches Isaac ROS standard)
set_target_properties(nitros_bridge_jpeg_decoder_node PROPERTIES
  INSTALL_RPATH "/opt/ros/humble/share/gxf_isaac_optimizer/gxf/lib:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/std:/opt/ros/humble/share/gxf_isaac_message_compositor/gxf/lib:/opt/ros/humble/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Register as composable node
rclcpp_components_register_nodes(nitros_bridge_jpeg_decoder_node
  "unity_nitros_bridge::NitrosBridgeJpegDecoderNode"
)

# =============================================================================
# Image Converter Node (NitrosBridgeImage → NitrosImage)
# =============================================================================
add_library(image_converter_node SHARED
  src/image_converter_node.cpp
)

target_include_directories(image_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(image_converter_node
  CUDA::cudart
  CUDA::cuda_driver
)

ament_target_dependencies(image_converter_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  isaac_ros_nitros_bridge_interfaces
  isaac_ros_nitros
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
)

# Set RUNPATH to find GXF libraries at runtime (matches Isaac ROS standard)
set_target_properties(image_converter_node PROPERTIES
  INSTALL_RPATH "/opt/ros/humble/share/gxf_isaac_optimizer/gxf/lib:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/std:/opt/ros/humble/share/gxf_isaac_message_compositor/gxf/lib:/opt/ros/humble/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Register as composable node
rclcpp_components_register_nodes(image_converter_node
  "unity_nitros_bridge::ImageConverterNode"
)

# =============================================================================
# SIMPLIFIED: Single-Node NITROS Image Decoder (No CUDA IPC needed for same-process)
# This is the recommended approach when all nodes run in the same container!
# =============================================================================
add_library(nitros_image_decoder_node SHARED
  src/nitros_image_decoder_node.cpp
  src/nvjpeg_decoder.cu
)

target_include_directories(nitros_image_decoder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(nitros_image_decoder_node
  CUDA::cudart
  ${NVJPEG_LIBRARY}
)

ament_target_dependencies(nitros_image_decoder_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  isaac_ros_nitros
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
)

# Set RUNPATH to find GXF libraries at runtime (matches Isaac ROS standard)
set_target_properties(nitros_image_decoder_node PROPERTIES
  INSTALL_RPATH "/opt/ros/humble/share/gxf_isaac_optimizer/gxf/lib:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/std:/opt/ros/humble/share/gxf_isaac_message_compositor/gxf/lib:/opt/ros/humble/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Register as composable node
rclcpp_components_register_nodes(nitros_image_decoder_node
  "unity_nitros_bridge::NitrosImageDecoderNode"
)

# =============================================================================
# Installation
# =============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS nitros_bridge_jpeg_decoder_node image_converter_node nitros_image_decoder_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config gxf
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(nitros_bridge_jpeg_decoder_node image_converter_node nitros_image_decoder_node)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  isaac_ros_nitros
  isaac_ros_managed_nitros
  isaac_ros_nitros_image_type
  isaac_ros_nitros_bridge_interfaces
)

ament_package()
