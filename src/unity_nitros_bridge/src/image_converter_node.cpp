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

#include "unity_nitros_bridge/image_converter_node.hpp"

#include <sys/syscall.h>
#include <unistd.h>

#include <memory>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"

namespace unity_nitros_bridge
{

using nvidia::isaac_ros::nitros::NitrosImage;
using nvidia::isaac_ros::nitros::NitrosImageBuilder;
using nvidia::isaac_ros::nitros::ManagedNitrosPublisher;

ImageConverterNode::ImageConverterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_converter", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_bridge_topic", "nitros_bridge_image");
  this->declare_parameter<std::string>("output_image_topic", "image_raw");
  this->declare_parameter<int>("device_id", 0);

  // Get parameters
  input_bridge_topic_ = this->get_parameter("input_bridge_topic").as_string();
  output_image_topic_ = this->get_parameter("output_image_topic").as_string();
  device_id_ = this->get_parameter("device_id").as_int();

  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "NITROS Bridge Image Converter Node");
  RCLCPP_INFO(this->get_logger(), "===========================================");
  RCLCPP_INFO(this->get_logger(), "  Input bridge: %s", input_bridge_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Output image: %s", output_image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Device ID: %d", device_id_);

  // Initialize CUDA VMM
  CUresult cu_result = cuInit(0);
  if (cu_result != CUDA_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize CUDA driver API");
    throw std::runtime_error("CUDA initialization failed");
  }

  CUdevice cu_device;
  cu_result = cuDeviceGet(&cu_device, device_id_);
  if (cu_result != CUDA_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get CUDA device %d", device_id_);
    throw std::runtime_error("CUDA device not found");
  }

  // Setup access descriptor for imported memory
  access_desc_.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access_desc_.location.id = device_id_;
  access_desc_.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;

  // Get allocation granularity
  CUmemAllocationProp prop = {};
  prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  prop.location.id = device_id_;
  prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;

  cu_result = cuMemGetAllocationGranularity(&granularity_, &prop, 
    CU_MEM_ALLOC_GRANULARITY_RECOMMENDED);
  if (cu_result != CUDA_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get allocation granularity");
    throw std::runtime_error("CUDA VMM initialization failed");
  }

  vmm_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "CUDA VMM initialized (granularity: %zu bytes)", granularity_);

  // Create NITROS publisher
  nitros_pub_ = std::make_shared<ManagedNitrosPublisher<NitrosImage>>(
    this,
    output_image_topic_,
    nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
    nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(),
    rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "NITROS publisher created for topic: %s", 
    output_image_topic_.c_str());

  // Subscribe to NitrosBridgeImage
  bridge_sub_ = this->create_subscription<
    isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
    input_bridge_topic_,
    rclcpp::QoS(10),
    std::bind(&ImageConverterNode::bridge_image_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Image Converter ready - NitrosBridgeImage → NitrosImage!");
}

ImageConverterNode::~ImageConverterNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Image Converter");
  RCLCPP_INFO(this->get_logger(), "  Frames converted: %lu", frames_converted_);
  RCLCPP_INFO(this->get_logger(), "  Import errors: %lu", import_errors_);

  // Cleanup all imported buffers
  for (auto& [uid, buffer] : imported_buffers_) {
    if (buffer.device_ptr != 0) {
      cuMemUnmap(buffer.device_ptr, buffer.size);
      cuMemAddressFree(buffer.device_ptr, buffer.size);
    }
    if (buffer.handle != 0) {
      cuMemRelease(buffer.handle);
    }
  }
  imported_buffers_.clear();
}

void ImageConverterNode::bridge_image_callback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::ConstSharedPtr & msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Received first NitrosBridgeImage message!");
  
  if (msg->data.size() < 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Invalid NitrosBridgeImage: data array must contain [PID, FD]");
    return;
  }

  int32_t source_pid = msg->data[0];
  int32_t source_fd = msg->data[1];
  size_t image_size = static_cast<size_t>(msg->step) * msg->height;
  
  RCLCPP_INFO_ONCE(this->get_logger(), 
    "First frame: PID=%d, FD=%d, size=%zu, uid=%s",
    source_pid, source_fd, image_size, msg->uid.c_str());

  // Check if we already have this buffer imported
  CUdeviceptr gpu_ptr = 0;
  auto it = imported_buffers_.find(msg->uid);
  
  if (it != imported_buffers_.end()) {
    // Reuse previously imported buffer
    gpu_ptr = it->second.device_ptr;
  } else {
    // Import new buffer via CUDA IPC
    CUmemGenericAllocationHandle handle;
    if (!import_ipc_memory(source_pid, source_fd, image_size, &gpu_ptr, &handle)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Failed to import GPU memory from PID %d, FD %d", source_pid, source_fd);
      import_errors_++;
      return;
    }

    // Cache the imported buffer
    ImportedBuffer buffer;
    buffer.handle = handle;
    buffer.device_ptr = gpu_ptr;
    buffer.size = ((image_size + granularity_ - 1) / granularity_) * granularity_;
    imported_buffers_[msg->uid] = buffer;

    RCLCPP_DEBUG(this->get_logger(), "Imported GPU memory: PID=%d, FD=%d, ptr=0x%lx",
      source_pid, source_fd, static_cast<unsigned long>(gpu_ptr));
  }

  // Build NitrosImage from GPU pointer
  // Note: NitrosImageBuilder handles the NITROS type wrapping
  try {
    auto nitros_image = NitrosImageBuilder()
      .WithHeader(msg->header)
      .WithDimensions(msg->height, msg->width)
      .WithEncoding(msg->encoding)
      .WithGpuData(reinterpret_cast<void*>(gpu_ptr))
      .Build();

    // Publish via NITROS (zero-copy to downstream nodes in same container)
    nitros_pub_->publish(nitros_image);
    frames_converted_++;

    if (frames_converted_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Converted %lu frames (NitrosBridgeImage → NitrosImage)",
        frames_converted_);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Failed to build NitrosImage: %s", e.what());
    import_errors_++;
  }
}

bool ImageConverterNode::import_ipc_memory(
  int32_t source_pid, int32_t source_fd, size_t size,
  CUdeviceptr* out_ptr, CUmemGenericAllocationHandle* out_handle)
{
  // Step 1: Get process file descriptor for the source process
  // pidfd_open() returns a file descriptor that refers to the process
  int pidfd = syscall(SYS_pidfd_open, source_pid, 0);
  if (pidfd < 0) {
    RCLCPP_ERROR(this->get_logger(), "pidfd_open failed for PID %d: %s", 
      source_pid, strerror(errno));
    return false;
  }

  // Step 2: Duplicate the source process's file descriptor
  // pidfd_getfd() duplicates a file descriptor from another process
  int local_fd = syscall(SYS_pidfd_getfd, pidfd, source_fd, 0);
  close(pidfd);  // Don't need pidfd anymore
  
  if (local_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "pidfd_getfd failed for FD %d from PID %d: %s",
      source_fd, source_pid, strerror(errno));
    return false;
  }

  // Step 3: Import the CUDA memory using the duplicated file descriptor
  CUresult cu_result = cuMemImportFromShareableHandle(
    out_handle,
    reinterpret_cast<void*>(static_cast<intptr_t>(local_fd)),
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
  
  close(local_fd);  // Don't need the FD after import
  
  if (cu_result != CUDA_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "cuMemImportFromShareableHandle failed: %d", cu_result);
    return false;
  }

  // Step 4: Reserve virtual address and map the imported memory
  size_t aligned_size = ((size + granularity_ - 1) / granularity_) * granularity_;
  
  cu_result = cuMemAddressReserve(out_ptr, aligned_size, granularity_, 0, 0);
  if (cu_result != CUDA_SUCCESS) {
    cuMemRelease(*out_handle);
    RCLCPP_ERROR(this->get_logger(), "cuMemAddressReserve failed: %d", cu_result);
    return false;
  }

  cu_result = cuMemMap(*out_ptr, aligned_size, 0, *out_handle, 0);
  if (cu_result != CUDA_SUCCESS) {
    cuMemAddressFree(*out_ptr, aligned_size);
    cuMemRelease(*out_handle);
    RCLCPP_ERROR(this->get_logger(), "cuMemMap failed: %d", cu_result);
    return false;
  }

  cu_result = cuMemSetAccess(*out_ptr, aligned_size, &access_desc_, 1);
  if (cu_result != CUDA_SUCCESS) {
    cuMemUnmap(*out_ptr, aligned_size);
    cuMemAddressFree(*out_ptr, aligned_size);
    cuMemRelease(*out_handle);
    RCLCPP_ERROR(this->get_logger(), "cuMemSetAccess failed: %d", cu_result);
    return false;
  }

  return true;
}

void ImageConverterNode::cleanup_imported_memory(const std::string& uid)
{
  auto it = imported_buffers_.find(uid);
  if (it != imported_buffers_.end()) {
    if (it->second.device_ptr != 0) {
      cuMemUnmap(it->second.device_ptr, it->second.size);
      cuMemAddressFree(it->second.device_ptr, it->second.size);
    }
    if (it->second.handle != 0) {
      cuMemRelease(it->second.handle);
    }
    imported_buffers_.erase(it);
  }
}

}  // namespace unity_nitros_bridge

// Register as composable node
RCLCPP_COMPONENTS_REGISTER_NODE(unity_nitros_bridge::ImageConverterNode)
