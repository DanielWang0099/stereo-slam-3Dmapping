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

#include "unity_nitros_bridge/nitros_image_decoder_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"

namespace unity_nitros_bridge
{

using nvidia::isaac_ros::nitros::NitrosImage;
using nvidia::isaac_ros::nitros::NitrosImageBuilder;
using nvidia::isaac_ros::nitros::ManagedNitrosPublisher;

constexpr int DEFAULT_NUM_BUFFERS = 8;

NitrosImageDecoderNode::NitrosImageDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("nitros_image_decoder", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_compressed_topic", "image/compressed");
  this->declare_parameter<std::string>("input_camera_info_topic", "camera_info");
  this->declare_parameter<std::string>("output_image_topic", "image_raw");
  this->declare_parameter<std::string>("output_camera_info_topic", "camera_info_out");
  this->declare_parameter<std::string>("output_encoding", "rgb8");
  this->declare_parameter<int>("num_buffers", DEFAULT_NUM_BUFFERS);

  // Get parameters
  input_compressed_topic_ = this->get_parameter("input_compressed_topic").as_string();
  input_camera_info_topic_ = this->get_parameter("input_camera_info_topic").as_string();
  output_image_topic_ = this->get_parameter("output_image_topic").as_string();
  output_camera_info_topic_ = this->get_parameter("output_camera_info_topic").as_string();
  output_encoding_ = this->get_parameter("output_encoding").as_string();
  num_buffers_ = this->get_parameter("num_buffers").as_int();

  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "NITROS Image Decoder Node (Simplified)");
  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "  Input compressed: %s", input_compressed_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output image: %s", output_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Ring buffer size: %d", num_buffers_);

  // Create CUDA stream
  cudaError_t cuda_err = cudaStreamCreate(&cuda_stream_);
  if (cuda_err != cudaSuccess) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create CUDA stream: %s",
      cudaGetErrorString(cuda_err));
    throw std::runtime_error("CUDA stream creation failed");
  }

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

  // Create NITROS publisher for zero-copy output
  nitros_pub_ = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(
    this,
    output_image_topic_,
    nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "NITROS publisher created for topic: %s", 
    output_image_topic_.c_str());

  // Standard publisher for camera info passthrough
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_topic_, rclcpp::QoS(10));

  // Subscribe to compressed images
  compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    input_compressed_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosImageDecoderNode::compressed_image_callback, this, 
      std::placeholders::_1));

  // Subscribe to camera info (passthrough)
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_camera_info_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosImageDecoderNode::camera_info_callback, this, 
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "NITROS Image Decoder ready!");
  RCLCPP_INFO(this->get_logger(), "  → NVJPEG decodes directly to GPU ring buffer");
  RCLCPP_INFO(this->get_logger(), "  → NitrosImageBuilder wraps GPU pointer");
  RCLCPP_INFO(this->get_logger(), "  → ManagedNitrosPublisher enables NITROS zero-copy");
}

NitrosImageDecoderNode::~NitrosImageDecoderNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down NITROS Image Decoder");
  RCLCPP_INFO(this->get_logger(), "  Frames decoded: %lu", frames_decoded_);
  RCLCPP_INFO(this->get_logger(), "  Decode errors: %lu", decode_errors_);

  // Free GPU buffers
  for (auto& ptr : gpu_buffers_) {
    if (ptr != nullptr) {
      cudaFree(ptr);
    }
  }
  gpu_buffers_.clear();

  if (cuda_stream_ != nullptr) {
    cudaStreamDestroy(cuda_stream_);
  }
}

bool NitrosImageDecoderNode::allocate_gpu_buffers(size_t buffer_size)
{
  buffer_size_ = buffer_size;
  gpu_buffers_.resize(num_buffers_, nullptr);

  for (int i = 0; i < num_buffers_; ++i) {
    cudaError_t err = cudaMalloc(&gpu_buffers_[i], buffer_size);
    if (err != cudaSuccess) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate GPU buffer %d: %s",
        i, cudaGetErrorString(err));
      // Cleanup already allocated
      for (int j = 0; j < i; ++j) {
        cudaFree(gpu_buffers_[j]);
        gpu_buffers_[j] = nullptr;
      }
      return false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Allocated %d GPU buffers x %.2f MB = %.2f MB total",
    num_buffers_, buffer_size / (1024.0 * 1024.0),
    (num_buffers_ * buffer_size) / (1024.0 * 1024.0));

  return true;
}

uint8_t* NitrosImageDecoderNode::get_next_buffer()
{
  uint8_t* ptr = gpu_buffers_[current_buffer_idx_];
  current_buffer_idx_ = (current_buffer_idx_ + 1) % num_buffers_;
  return ptr;
}

void NitrosImageDecoderNode::compressed_image_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Received empty compressed image");
    return;
  }

  // Get image info on first frame
  if (image_width_ == 0 || image_height_ == 0) {
    int width, height, channels, subsampling;
    if (!decoder_->get_image_info(msg->data.data(), msg->data.size(),
        &width, &height, &channels, &subsampling)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get image info: %s",
        decoder_->get_last_error());
      return;
    }
    
    image_width_ = width;
    image_height_ = height;
    image_channels_ = channels;
    
    RCLCPP_INFO(this->get_logger(), "First frame: %dx%dx%d", width, height, channels);
    
    // Allocate GPU ring buffer
    size_t buffer_size = static_cast<size_t>(width) * height * channels;
    if (!allocate_gpu_buffers(buffer_size)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate GPU buffers");
      return;
    }
  }

  // Get next buffer from ring
  uint8_t* gpu_ptr = get_next_buffer();
  if (gpu_ptr == nullptr) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "GPU buffer is null!");
    return;
  }

  // Decode JPEG directly to GPU buffer
  size_t pitch = image_width_ * image_channels_;
  bool success = decoder_->decode(
    msg->data.data(),
    msg->data.size(),
    gpu_ptr,
    pitch,
    cuda_stream_);

  if (!success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to decode JPEG: %s", decoder_->get_last_error());
    decode_errors_++;
    return;
  }

  // Sync stream before publishing
  cudaStreamSynchronize(cuda_stream_);

  // Build NitrosImage from GPU pointer and publish
  try {
    auto nitros_image = NitrosImageBuilder()
      .WithHeader(msg->header)
      .WithDimensions(image_height_, image_width_)
      .WithEncoding(output_encoding_)
      .WithGpuData(reinterpret_cast<void*>(gpu_ptr))
      .Build();

    nitros_pub_->publish(nitros_image);
    frames_decoded_++;

    if (frames_decoded_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Decoded %lu frames (GPU zero-copy)",
        frames_decoded_);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to build/publish NitrosImage: %s", e.what());
    decode_errors_++;
  }
}

void NitrosImageDecoderNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  // Simple passthrough
  camera_info_pub_->publish(*msg);
}

}  // namespace unity_nitros_bridge

// Register as composable node
RCLCPP_COMPONENTS_REGISTER_NODE(unity_nitros_bridge::NitrosImageDecoderNode)
