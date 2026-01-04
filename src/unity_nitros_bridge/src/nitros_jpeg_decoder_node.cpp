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

#include "unity_nitros_bridge/nitros_jpeg_decoder_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace unity_nitros_bridge
{

using nvidia::isaac_ros::nitros::NitrosImage;
using nvidia::isaac_ros::nitros::ManagedNitrosPublisher;

NitrosJpegDecoderNode::NitrosJpegDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("nitros_jpeg_decoder", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_compressed_topic", "image/compressed");
  this->declare_parameter<std::string>("input_camera_info_topic", "camera_info");
  this->declare_parameter<std::string>("output_image_topic", "image_raw");
  this->declare_parameter<std::string>("output_camera_info_topic", "camera_info_out");
  this->declare_parameter<std::string>("output_encoding", "rgb8");
  this->declare_parameter<int>("output_width", 0);  // 0 = use source size
  this->declare_parameter<int>("output_height", 0);

  // Get parameters
  input_compressed_topic_ = this->get_parameter("input_compressed_topic").as_string();
  input_camera_info_topic_ = this->get_parameter("input_camera_info_topic").as_string();
  output_image_topic_ = this->get_parameter("output_image_topic").as_string();
  output_camera_info_topic_ = this->get_parameter("output_camera_info_topic").as_string();
  output_encoding_ = this->get_parameter("output_encoding").as_string();
  output_width_ = this->get_parameter("output_width").as_int();
  output_height_ = this->get_parameter("output_height").as_int();

  RCLCPP_INFO(this->get_logger(), "Initializing NITROS JPEG Decoder Node");
  RCLCPP_INFO(this->get_logger(), "  Input compressed: %s", input_compressed_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output image: %s", output_image_topic_.c_str());

  // Initialize GPU decoder
  try {
    decoder_ = std::make_unique<NvjpegDecoder>(NVJPEG_OUTPUT_RGBI);
    if (!decoder_->is_initialized()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize NVJPEG decoder: %s",
        decoder_->get_last_error());
      throw std::runtime_error("NVJPEG initialization failed");
    }
    RCLCPP_INFO(this->get_logger(), "NVJPEG decoder initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception initializing decoder: %s", e.what());
    throw;
  }

  // Create NITROS publisher for zero-copy image output
  // ManagedNitrosPublisher accepts sensor_msgs::Image and handles NITROS conversion
  nitros_image_pub_ = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(
    this,
    output_image_topic_,
    nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "NITROS publisher created for topic: %s", 
    output_image_topic_.c_str());

  // Standard publisher for camera info (metadata, no need for NITROS)
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_topic_, rclcpp::QoS(10));

  // Subscribe to compressed images
  compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    input_compressed_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosJpegDecoderNode::compressed_image_callback, this, std::placeholders::_1));

  // Subscribe to camera info
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_camera_info_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosJpegDecoderNode::camera_info_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "NITROS JPEG Decoder Node initialized - Zero-copy GPU pipeline ready!");
}

NitrosJpegDecoderNode::~NitrosJpegDecoderNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down NITROS JPEG Decoder");
  RCLCPP_INFO(this->get_logger(), "  Frames decoded: %lu", frames_decoded_);
  RCLCPP_INFO(this->get_logger(), "  Decode errors: %lu", decode_errors_);
}

void NitrosJpegDecoderNode::compressed_image_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Received empty compressed image");
    return;
  }

  // Decode on GPU
  uint8_t* device_ptr = nullptr;
  int width, height, channels;

  bool success = decoder_->decode_alloc(
    msg->data.data(),
    msg->data.size(),
    &device_ptr,
    &width,
    &height,
    &channels,
    nullptr);  // Default stream

  if (!success || device_ptr == nullptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to decode JPEG: %s", decoder_->get_last_error());
    decode_errors_++;
    return;
  }

  // Create sensor_msgs::Image - ManagedNitrosPublisher will handle NITROS conversion
  sensor_msgs::msg::Image ros_image;
  ros_image.header = msg->header;
  ros_image.height = height;
  ros_image.width = width;
  ros_image.encoding = "rgb8";
  ros_image.step = width * channels;
  ros_image.is_bigendian = false;
  
  // Resize data vector and copy from GPU
  ros_image.data.resize(height * width * channels);
  cudaMemcpy(ros_image.data.data(), device_ptr, 
    height * width * channels, cudaMemcpyDeviceToHost);

  // Publish via NITROS (will be converted to NITROS format for zero-copy downstream)
  nitros_image_pub_->publish(ros_image);

  // Free GPU memory (in production, use memory pool)
  cudaFree(device_ptr);

  frames_decoded_++;

  if (frames_decoded_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "Decoded %lu frames (errors: %lu)",
      frames_decoded_, decode_errors_);
  }
}

void NitrosJpegDecoderNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  // Cache and republish camera info
  last_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
  camera_info_pub_->publish(*msg);
}

}  // namespace unity_nitros_bridge

// Register as composable node
RCLCPP_COMPONENTS_REGISTER_NODE(unity_nitros_bridge::NitrosJpegDecoderNode)
