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

#ifndef UNITY_NITROS_BRIDGE__NITROS_IMAGE_DECODER_NODE_HPP_
#define UNITY_NITROS_BRIDGE__NITROS_IMAGE_DECODER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

#include "unity_nitros_bridge/nvjpeg_decoder.hpp"

namespace unity_nitros_bridge
{

/**
 * @brief Simplified Zero-Copy JPEG Decoder for same-process NITROS pipeline.
 * 
 * This node decodes JPEG images on GPU and publishes NitrosImage directly
 * using NitrosImageBuilder. Since all NITROS nodes are in the same container,
 * we can pass GPU pointers directly without CUDA IPC.
 * 
 * Data Flow (all in same process):
 *   CompressedImage (JPEG bytes)
 *       ↓
 *   NVJPEG decode → GPU ring buffer
 *       ↓
 *   NitrosImageBuilder::WithGpuData(ptr)
 *       ↓
 *   ManagedNitrosPublisher<NitrosImage>
 *       ↓
 *   ESS/VSLAM receive GPU pointer directly (ZERO COPY!)
 * 
 * Key Features:
 * - NO GPU→CPU copy
 * - Pre-allocated GPU ring buffer (no per-frame allocation)
 * - Direct NitrosImage publishing via ManagedNitrosPublisher
 * - Works within same container (no CUDA IPC needed)
 */
class NitrosImageDecoderNode : public rclcpp::Node
{
public:
  explicit NitrosImageDecoderNode(const rclcpp::NodeOptions & options);
  ~NitrosImageDecoderNode();

private:
  /// Callback for compressed image messages
  void compressed_image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);

  /// Callback for camera info messages (passthrough)
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  /// Allocate GPU ring buffer
  bool allocate_gpu_buffers(size_t buffer_size);

  /// Get next buffer from ring (round-robin)
  uint8_t* get_next_buffer();

  // Parameters
  std::string input_compressed_topic_;
  std::string input_camera_info_topic_;
  std::string output_image_topic_;
  std::string output_camera_info_topic_;
  std::string output_encoding_;
  int num_buffers_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // NITROS publisher for zero-copy output
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
    nvidia::isaac_ros::nitros::NitrosImage>> nitros_pub_;
  
  // Standard publisher for camera_info (passthrough)
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // GPU JPEG decoder
  std::unique_ptr<NvjpegDecoder> decoder_;

  // GPU ring buffer (pre-allocated)
  std::vector<uint8_t*> gpu_buffers_;
  size_t buffer_size_{0};
  size_t current_buffer_idx_{0};

  // Image dimensions (set on first decode)
  int image_width_{0};
  int image_height_{0};
  int image_channels_{3};

  // CUDA stream
  cudaStream_t cuda_stream_{nullptr};

  // Statistics
  uint64_t frames_decoded_{0};
  uint64_t decode_errors_{0};
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__NITROS_IMAGE_DECODER_NODE_HPP_
