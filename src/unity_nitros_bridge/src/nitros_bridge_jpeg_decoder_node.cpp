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

#include "unity_nitros_bridge/nitros_bridge_jpeg_decoder_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace unity_nitros_bridge
{

// Default buffer size for 960x576 RGB8 image with some padding
constexpr size_t DEFAULT_BUFFER_SIZE = 960 * 576 * 3 * 2;  // ~3.3 MB per buffer
constexpr int DEFAULT_NUM_BUFFERS = 8;  // Ring buffer with 8 slots

NitrosBridgeJpegDecoderNode::NitrosBridgeJpegDecoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("nitros_bridge_jpeg_decoder", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_compressed_topic", "image/compressed");
  this->declare_parameter<std::string>("input_camera_info_topic", "camera_info");
  this->declare_parameter<std::string>("output_bridge_image_topic", "nitros_bridge_image");
  this->declare_parameter<std::string>("output_camera_info_topic", "camera_info_out");
  this->declare_parameter<std::string>("output_encoding", "rgb8");
  this->declare_parameter<int>("num_buffers", DEFAULT_NUM_BUFFERS);
  this->declare_parameter<int>("device_id", 0);

  // Get parameters
  input_compressed_topic_ = this->get_parameter("input_compressed_topic").as_string();
  input_camera_info_topic_ = this->get_parameter("input_camera_info_topic").as_string();
  output_bridge_image_topic_ = this->get_parameter("output_bridge_image_topic").as_string();
  output_camera_info_topic_ = this->get_parameter("output_camera_info_topic").as_string();
  output_encoding_ = this->get_parameter("output_encoding").as_string();
  num_buffers_ = this->get_parameter("num_buffers").as_int();
  device_id_ = this->get_parameter("device_id").as_int();

  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "TRUE Zero-Copy NITROS Bridge JPEG Decoder");
  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "  Input compressed: %s", input_compressed_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output bridge image: %s", output_bridge_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  IPC buffers: %d", num_buffers_);
  RCLCPP_INFO(this->get_logger(), "  Device ID: %d", device_id_);

  // Create CUDA stream for async operations
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

  // IPC Buffer Manager will be initialized on first frame (need image size)
  // We'll create it with DEFAULT_BUFFER_SIZE initially and it will handle alignment

  // Create publisher for NitrosBridgeImage (CUDA IPC handles)
  bridge_image_pub_ = this->create_publisher<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
    output_bridge_image_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "NitrosBridgeImage publisher created for topic: %s", 
    output_bridge_image_topic_.c_str());

  // Standard publisher for camera info passthrough
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    output_camera_info_topic_, rclcpp::QoS(10));

  // Subscribe to compressed images
  compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    input_compressed_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosBridgeJpegDecoderNode::compressed_image_callback, this, 
      std::placeholders::_1));

  // Subscribe to camera info (passthrough)
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    input_camera_info_topic_,
    rclcpp::QoS(10),
    std::bind(&NitrosBridgeJpegDecoderNode::camera_info_callback, this, 
      std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TRUE Zero-Copy JPEG Decoder ready!");
  RCLCPP_INFO(this->get_logger(), "  → NVJPEG decodes directly to IPC buffer pool");
  RCLCPP_INFO(this->get_logger(), "  → NitrosBridgeImage passes GPU memory via CUDA IPC");
  RCLCPP_INFO(this->get_logger(), "  → NO GPU→CPU copy in this node!");
}

NitrosBridgeJpegDecoderNode::~NitrosBridgeJpegDecoderNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down TRUE Zero-Copy JPEG Decoder");
  RCLCPP_INFO(this->get_logger(), "  Frames decoded: %lu", frames_decoded_);
  RCLCPP_INFO(this->get_logger(), "  Decode errors: %lu", decode_errors_);

  if (cuda_stream_ != nullptr) {
    cudaStreamDestroy(cuda_stream_);
  }
}

void NitrosBridgeJpegDecoderNode::compressed_image_callback(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Received empty compressed image");
    return;
  }

  // Get image info to determine buffer size (first frame)
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
    
    // Calculate required buffer size
    size_t buffer_size = static_cast<size_t>(width) * height * channels;
    
    RCLCPP_INFO(this->get_logger(), "First frame received: %dx%dx%d (%.2f MB per buffer)",
      width, height, channels, buffer_size / (1024.0 * 1024.0));
    
    // Initialize IPC buffer manager with correct size
    ipc_buffer_manager_ = std::make_unique<IPCBufferManager>(
      num_buffers_, buffer_size, device_id_);
    
    if (!ipc_buffer_manager_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize IPC buffer manager: %s",
        ipc_buffer_manager_->get_last_error());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "IPC Buffer Pool initialized: %d buffers x %.2f MB = %.2f MB total GPU memory",
      num_buffers_, buffer_size / (1024.0 * 1024.0), 
      (num_buffers_ * buffer_size) / (1024.0 * 1024.0));
  }

  // Acquire buffer from IPC pool (round-robin, no allocation!)
  auto* buffer = ipc_buffer_manager_->acquire_buffer();
  if (buffer == nullptr) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to acquire IPC buffer: %s", ipc_buffer_manager_->get_last_error());
    decode_errors_++;
    return;
  }

  // Get GPU pointer from IPC buffer
  uint8_t* device_ptr = ipc_buffer_manager_->get_device_ptr(buffer);
  size_t pitch = image_width_ * image_channels_;

  // Decode JPEG directly to IPC buffer (NO COPY!)
  bool success = decoder_->decode(
    msg->data.data(),
    msg->data.size(),
    device_ptr,
    pitch,
    cuda_stream_);

  if (!success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to decode JPEG: %s", decoder_->get_last_error());
    ipc_buffer_manager_->release_buffer(buffer);
    decode_errors_++;
    return;
  }

  // Synchronize stream to ensure decode is complete before publishing handle
  cudaStreamSynchronize(cuda_stream_);

  // Create NitrosBridgeImage message with CUDA IPC handles
  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage bridge_msg;
  bridge_msg.header = msg->header;
  bridge_msg.height = image_height_;
  bridge_msg.width = image_width_;
  bridge_msg.encoding = output_encoding_;
  bridge_msg.is_bigendian = 0;
  bridge_msg.step = image_width_ * image_channels_;
  
  // KEY: Pass PID and FD for CUDA IPC!
  // Receiver process uses these to access the GPU memory:
  //   1. pidfd_open(pid) to get process FD
  //   2. pidfd_getfd(pidfd, fd) to duplicate the memory FD
  //   3. cuMemImportFromShareableHandle() to access GPU memory
  bridge_msg.data.resize(2);
  bridge_msg.data[0] = ipc_buffer_manager_->get_pid();  // Process ID
  bridge_msg.data[1] = buffer->shareable_fd;             // File descriptor
  
  bridge_msg.uid = buffer->uid;
  bridge_msg.device_id = device_id_;
  
  // cuda_event_handle left empty for now (synchronous operation)
  
  // Publish!
  bridge_image_pub_->publish(bridge_msg);
  
  frames_decoded_++;

  if (frames_decoded_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "TRUE Zero-Copy: %lu frames decoded (no GPU→CPU copy!)",
      frames_decoded_);
  }
}

void NitrosBridgeJpegDecoderNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  // Simple passthrough - camera info is small metadata, no GPU needed
  camera_info_pub_->publish(*msg);
}

}  // namespace unity_nitros_bridge

// Register as composable node
RCLCPP_COMPONENTS_REGISTER_NODE(unity_nitros_bridge::NitrosBridgeJpegDecoderNode)
