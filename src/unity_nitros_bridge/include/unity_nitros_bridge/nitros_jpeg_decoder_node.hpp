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

#ifndef UNITY_NITROS_BRIDGE__NITROS_JPEG_DECODER_NODE_HPP_
#define UNITY_NITROS_BRIDGE__NITROS_JPEG_DECODER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"

#include "unity_nitros_bridge/nvjpeg_decoder.hpp"

namespace unity_nitros_bridge
{

/**
 * @brief NITROS-compatible JPEG decoder node for Unity simulation.
 * 
 * This node subscribes to compressed JPEG images (from Unity or rosbag),
 * decodes them on GPU using NVJPEG, and publishes as NITROS images for
 * zero-copy downstream processing.
 * 
 * Key features:
 * - GPU-only JPEG decoding (no CPU involvement)
 * - NITROS output for zero-copy pipeline
 * - Supports both /compressed and raw image topics
 */
class NitrosJpegDecoderNode : public rclcpp::Node
{
public:
  explicit NitrosJpegDecoderNode(const rclcpp::NodeOptions & options);
  ~NitrosJpegDecoderNode();

private:
  /// Callback for compressed image messages
  void compressed_image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);

  /// Callback for camera info messages
  void camera_info_callback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);

  // Parameters
  std::string input_compressed_topic_;
  std::string input_camera_info_topic_;
  std::string output_image_topic_;
  std::string output_camera_info_topic_;
  std::string output_encoding_;
  int output_width_;
  int output_height_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // NITROS publishers for zero-copy output
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
    nvidia::isaac_ros::nitros::NitrosImage>> nitros_image_pub_;
  
  // Standard publisher for camera_info (no NITROS needed for metadata)
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

  // GPU JPEG decoder
  std::unique_ptr<NvjpegDecoder> decoder_;

  // Cached camera info
  sensor_msgs::msg::CameraInfo::SharedPtr last_camera_info_;

  // Statistics
  uint64_t frames_decoded_{0};
  uint64_t decode_errors_{0};
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__NITROS_JPEG_DECODER_NODE_HPP_
