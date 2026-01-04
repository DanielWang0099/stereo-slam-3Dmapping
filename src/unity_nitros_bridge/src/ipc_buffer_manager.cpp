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

#include "unity_nitros_bridge/ipc_buffer_manager.hpp"

#include <cuda.h>
#include <cuda_runtime.h>

#include <sstream>
#include <iomanip>
#include <chrono>
#include <random>
#include <cstring>

namespace unity_nitros_bridge
{

IPCBufferManager::IPCBufferManager(size_t num_buffers, size_t buffer_size, int device_id)
: num_buffers_(num_buffers),
  buffer_size_(buffer_size),
  device_id_(device_id),
  pid_(static_cast<int32_t>(getpid()))
{
  buffers_.resize(num_buffers_);
}

IPCBufferManager::~IPCBufferManager()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& buffer : buffers_) {
    destroy_ipc_buffer(buffer);
  }
  initialized_ = false;
}

bool IPCBufferManager::initialize()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    return true;
  }

  // Initialize CUDA driver API
  CUresult cu_result = cuInit(0);
  if (cu_result != CUDA_SUCCESS) {
    last_error_ = "Failed to initialize CUDA driver API";
    return false;
  }

  // Get CUDA device
  CUdevice cu_device;
  cu_result = cuDeviceGet(&cu_device, device_id_);
  if (cu_result != CUDA_SUCCESS) {
    last_error_ = "Failed to get CUDA device " + std::to_string(device_id_);
    return false;
  }

  // Check if device supports VMM (Virtual Memory Management)
  int vmm_supported = 0;
  cu_result = cuDeviceGetAttribute(&vmm_supported, 
    CU_DEVICE_ATTRIBUTE_VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED, cu_device);
  if (cu_result != CUDA_SUCCESS || !vmm_supported) {
    last_error_ = "Device does not support CUDA Virtual Memory Management";
    return false;
  }

  // Check if device supports shareable handles (POSIX FD)
  int handle_types_supported = 0;
  cu_result = cuDeviceGetAttribute(&handle_types_supported,
    CU_DEVICE_ATTRIBUTE_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED, cu_device);
  if (cu_result != CUDA_SUCCESS || !handle_types_supported) {
    last_error_ = "Device does not support POSIX file descriptor handles for IPC";
    return false;
  }

  // Setup allocation properties for shareable memory
  memset(&allocation_prop_, 0, sizeof(allocation_prop_));
  allocation_prop_.type = CU_MEM_ALLOCATION_TYPE_PINNED;
  allocation_prop_.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  allocation_prop_.location.id = device_id_;
  allocation_prop_.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;

  // Get allocation granularity
  cu_result = cuMemGetAllocationGranularity(&granularity_, &allocation_prop_,
    CU_MEM_ALLOC_GRANULARITY_RECOMMENDED);
  if (cu_result != CUDA_SUCCESS) {
    last_error_ = "Failed to get allocation granularity";
    return false;
  }

  // Round up buffer size to granularity
  size_t aligned_size = ((buffer_size_ + granularity_ - 1) / granularity_) * granularity_;
  buffer_size_ = aligned_size;

  // Setup access descriptor
  access_desc_.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
  access_desc_.location.id = device_id_;
  access_desc_.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;

  // Create all buffers
  for (size_t i = 0; i < num_buffers_; ++i) {
    if (!create_ipc_buffer(buffers_[i])) {
      // Cleanup already created buffers
      for (size_t j = 0; j < i; ++j) {
        destroy_ipc_buffer(buffers_[j]);
      }
      return false;
    }
  }

  initialized_ = true;
  return true;
}

bool IPCBufferManager::create_ipc_buffer(BufferInfo& buffer)
{
  CUresult cu_result;

  // Create physical memory allocation
  cu_result = cuMemCreate(&buffer.allocation_handle, buffer_size_, &allocation_prop_, 0);
  if (cu_result != CUDA_SUCCESS) {
    last_error_ = "cuMemCreate failed with error " + std::to_string(cu_result);
    return false;
  }

  // Reserve virtual address range
  cu_result = cuMemAddressReserve(&buffer.device_ptr, buffer_size_, granularity_, 0, 0);
  if (cu_result != CUDA_SUCCESS) {
    cuMemRelease(buffer.allocation_handle);
    last_error_ = "cuMemAddressReserve failed with error " + std::to_string(cu_result);
    return false;
  }

  // Map physical memory to virtual address
  cu_result = cuMemMap(buffer.device_ptr, buffer_size_, 0, buffer.allocation_handle, 0);
  if (cu_result != CUDA_SUCCESS) {
    cuMemAddressFree(buffer.device_ptr, buffer_size_);
    cuMemRelease(buffer.allocation_handle);
    last_error_ = "cuMemMap failed with error " + std::to_string(cu_result);
    return false;
  }

  // Set memory access permissions
  cu_result = cuMemSetAccess(buffer.device_ptr, buffer_size_, &access_desc_, 1);
  if (cu_result != CUDA_SUCCESS) {
    cuMemUnmap(buffer.device_ptr, buffer_size_);
    cuMemAddressFree(buffer.device_ptr, buffer_size_);
    cuMemRelease(buffer.allocation_handle);
    last_error_ = "cuMemSetAccess failed with error " + std::to_string(cu_result);
    return false;
  }

  // Export as POSIX file descriptor for IPC
  cu_result = cuMemExportToShareableHandle(
    &buffer.shareable_fd,
    buffer.allocation_handle,
    CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR,
    0);
  if (cu_result != CUDA_SUCCESS) {
    cuMemUnmap(buffer.device_ptr, buffer_size_);
    cuMemAddressFree(buffer.device_ptr, buffer_size_);
    cuMemRelease(buffer.allocation_handle);
    last_error_ = "cuMemExportToShareableHandle failed with error " + std::to_string(cu_result);
    return false;
  }

  buffer.size = buffer_size_;
  buffer.uid = generate_uid();
  buffer.in_use = false;

  return true;
}

void IPCBufferManager::destroy_ipc_buffer(BufferInfo& buffer)
{
  if (buffer.shareable_fd >= 0) {
    close(buffer.shareable_fd);
    buffer.shareable_fd = -1;
  }

  if (buffer.device_ptr != 0) {
    cuMemUnmap(buffer.device_ptr, buffer.size);
    cuMemAddressFree(buffer.device_ptr, buffer.size);
    buffer.device_ptr = 0;
  }

  if (buffer.allocation_handle != 0) {
    cuMemRelease(buffer.allocation_handle);
    buffer.allocation_handle = 0;
  }

  buffer.size = 0;
  buffer.in_use = false;
}

std::string IPCBufferManager::generate_uid()
{
  std::stringstream ss;
  ss << "ipc_" << pid_ << "_" << device_id_ << "_" 
     << std::setfill('0') << std::setw(8) << (uid_counter_++);
  return ss.str();
}

IPCBufferManager::BufferInfo* IPCBufferManager::acquire_buffer()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    last_error_ = "Buffer manager not initialized";
    return nullptr;
  }

  // Round-robin buffer selection
  // Note: We don't check in_use because we assume the consumer processes
  // the previous frame before we wrap around (ring buffer pattern)
  BufferInfo* buffer = &buffers_[next_buffer_idx_];
  buffer->in_use = true;
  
  next_buffer_idx_ = (next_buffer_idx_ + 1) % num_buffers_;
  
  return buffer;
}

void IPCBufferManager::release_buffer(BufferInfo* buffer)
{
  if (buffer != nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer->in_use = false;
  }
}

uint8_t* IPCBufferManager::get_device_ptr(const BufferInfo* buffer) const
{
  if (buffer == nullptr) {
    return nullptr;
  }
  return reinterpret_cast<uint8_t*>(buffer->device_ptr);
}

}  // namespace unity_nitros_bridge
