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

#ifndef UNITY_NITROS_BRIDGE__NITROS_BRIDGE_JPEG_DECODER_NODE_HPP_
#define UNITY_NITROS_BRIDGE__NITROS_BRIDGE_JPEG_DECODER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "isaac_ros_nitros_bridge_interfaces/msg/nitros_bridge_image.hpp"

#include "unity_nitros_bridge/ipc_buffer_manager.hpp"
#include "unity_nitros_bridge/nvjpeg_decoder.hpp"

namespace unity_nitros_bridge
{

/**
 * @brief TRUE Zero-Copy JPEG Decoder Node using NitrosBridgeImage + CUDA IPC.
 * 
 * This node implements TRUE zero-copy JPEG decoding for Isaac ROS NITROS pipeline:
 * 
 * Data Flow:
 *   CompressedImage (CPU JPEG bytes)
 *       ↓
 *   NVJPEG decode → IPC Buffer Pool (GPU memory with shareable handles)
 *       ↓
 *   Publish NitrosBridgeImage with {PID, FD} (CUDA IPC handle)
 *       ↓
 *   [External] ImageConverterNode (isaac_ros_nitros_bridge_ros2)
 *       ↓
 *   NitrosImage (GPU pointer for NITROS pipeline) - TRUE ZERO COPY!
 * 
 * Key differences from previous implementation:
 * - NO GPU→CPU copy for publishing (was the bottleneck!)
 * - Uses CUDA Virtual Memory Management APIs for IPC
 * - Publishes NitrosBridgeImage (not NitrosImage directly)
 * - Requires ImageConverterNode downstream to convert to NitrosImage
 * 
 * Memory Management:
 * - Pre-allocated ring buffer pool of GPU memory (IPCBufferManager)
 * - Each buffer has exported POSIX file descriptor for IPC
 * - Round-robin buffer selection (no per-frame allocation!)
 */
class NitrosBridgeJpegDecoderNode : public rclcpp::Node
{
public:
  explicit NitrosBridgeJpegDecoderNode(const rclcpp::NodeOptions & options);
  ~NitrosBridgeJpegDecoderNode();

private:
  /// Callback for compressed image messages
  void compressed_image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);

  /// Callback for camera info messages (passthrough)
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // Parameters
  std::string input_compressed_topic_;
  std::string input_camera_info_topic_;
  std::string output_bridge_image_topic_;
  std::string output_camera_info_topic_;
  std::string output_encoding_;
  int num_buffers_;
  int device_id_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publisher for NitrosBridgeImage (CUDA IPC handles)
  rclcpp::Publisher<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>::SharedPtr 
    bridge_image_pub_;
  
  // Standard publisher for camera_info (passthrough)
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // GPU JPEG decoder (decodes directly to IPC buffer)
  std::unique_ptr<NvjpegDecoder> decoder_;

  // IPC Buffer Manager (pre-allocated GPU memory pool with shareable handles)
  std::unique_ptr<IPCBufferManager> ipc_buffer_manager_;

  // CUDA stream for async operations
  cudaStream_t cuda_stream_{nullptr};

  // Image dimensions (set on first decode)
  int image_width_{0};
  int image_height_{0};
  int image_channels_{3};  // RGB

  // Statistics
  uint64_t frames_decoded_{0};
  uint64_t decode_errors_{0};
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__NITROS_BRIDGE_JPEG_DECODER_NODE_HPP_
