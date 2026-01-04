// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef UNITY_NITROS_BRIDGE__IMAGE_CONVERTER_NODE_HPP_
#define UNITY_NITROS_BRIDGE__IMAGE_CONVERTER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include <cuda.h>
#include <cuda_runtime.h>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_nitros_bridge_interfaces/msg/nitros_bridge_image.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

namespace unity_nitros_bridge
{

/**
 * @brief Converts NitrosBridgeImage (CUDA IPC) to NitrosImage for NITROS pipeline.
 * 
 * This node receives NitrosBridgeImage messages containing CUDA IPC handles
 * (PID + file descriptor), imports the GPU memory from the source process,
 * and publishes as NitrosImage for zero-copy NITROS pipeline consumption.
 * 
 * Data Flow:
 *   NitrosBridgeImage {PID, FD}
 *       ↓
 *   pidfd_getfd() to duplicate FD from source process
 *       ↓
 *   cuMemImportFromShareableHandle() to access GPU memory
 *       ↓
 *   NitrosImageBuilder::WithGpuData() to create NitrosImage
 *       ↓
 *   ManagedNitrosPublisher publishes to NITROS pipeline
 * 
 * Reference: NVIDIA Isaac ROS NITROS Bridge
 * https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros_bridge
 */
class ImageConverterNode : public rclcpp::Node
{
public:
  explicit ImageConverterNode(const rclcpp::NodeOptions & options);
  ~ImageConverterNode();

private:
  /// Callback for NitrosBridgeImage messages
  void bridge_image_callback(
    const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::ConstSharedPtr & msg);

  /// Import GPU memory from another process using CUDA IPC
  bool import_ipc_memory(int32_t pid, int32_t fd, size_t size, 
    CUdeviceptr* out_ptr, CUmemGenericAllocationHandle* out_handle);

  /// Clean up imported memory
  void cleanup_imported_memory(const std::string& uid);

  // Parameters
  std::string input_bridge_topic_;
  std::string output_image_topic_;
  int device_id_;

  // Subscription
  rclcpp::Subscription<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>::SharedPtr 
    bridge_sub_;

  // NITROS publisher for zero-copy output
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
    nvidia::isaac_ros::nitros::NitrosImage>> nitros_pub_;

  // Cache of imported GPU memory handles (keyed by UID)
  struct ImportedBuffer {
    CUmemGenericAllocationHandle handle;
    CUdeviceptr device_ptr;
    size_t size;
  };
  std::unordered_map<std::string, ImportedBuffer> imported_buffers_;

  // CUDA VMM properties
  CUmemAccessDesc access_desc_{};
  size_t granularity_{0};
  bool vmm_initialized_{false};

  // Statistics
  uint64_t frames_converted_{0};
  uint64_t import_errors_{0};
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__IMAGE_CONVERTER_NODE_HPP_
