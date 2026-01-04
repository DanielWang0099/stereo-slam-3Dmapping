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

#ifndef UNITY_NITROS_BRIDGE__IPC_BUFFER_MANAGER_HPP_
#define UNITY_NITROS_BRIDGE__IPC_BUFFER_MANAGER_HPP_

#include <cuda.h>
#include <cuda_runtime.h>

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <cstdint>

#include <sys/types.h>
#include <unistd.h>

namespace unity_nitros_bridge
{

/**
 * @brief Manages a pool of GPU memory buffers with CUDA IPC handles for zero-copy
 *        inter-process sharing.
 * 
 * This class implements a ring buffer pool of GPU memory blocks that can be shared
 * across processes using CUDA's Virtual Memory Management APIs:
 * - cuMemCreate: Creates physical GPU memory allocation
 * - cuMemExportToShareableHandle: Exports as POSIX file descriptor
 * - The FD can be passed via NitrosBridgeImage to another process
 * - Receiver uses cuMemImportFromShareableHandle to access the GPU memory
 * 
 * Pattern from NVIDIA Isaac ROS NITROS Bridge:
 * https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros_bridge
 */
class IPCBufferManager
{
public:
  /**
   * @brief Information about a single IPC buffer
   */
  struct BufferInfo
  {
    CUmemGenericAllocationHandle allocation_handle{0};  ///< CUDA VMM allocation handle
    CUdeviceptr device_ptr{0};                          ///< GPU virtual address
    size_t size{0};                                     ///< Buffer size in bytes
    int shareable_fd{-1};                               ///< POSIX file descriptor for IPC
    std::string uid;                                    ///< Unique identifier for this buffer
    bool in_use{false};                                 ///< Whether buffer is currently in use
  };

  /**
   * @brief Construct IPC Buffer Manager
   * @param num_buffers Number of buffers in the pool (ring buffer)
   * @param buffer_size Size of each buffer in bytes
   * @param device_id CUDA device ID
   */
  IPCBufferManager(size_t num_buffers, size_t buffer_size, int device_id = 0);

  /**
   * @brief Destructor - releases all GPU memory and closes file descriptors
   */
  ~IPCBufferManager();

  // Prevent copying
  IPCBufferManager(const IPCBufferManager&) = delete;
  IPCBufferManager& operator=(const IPCBufferManager&) = delete;

  /**
   * @brief Initialize the buffer pool
   * @return true if initialization successful
   */
  bool initialize();

  /**
   * @brief Check if manager is initialized
   */
  bool is_initialized() const { return initialized_; }

  /**
   * @brief Get the next available buffer from the pool (round-robin)
   * @return Pointer to buffer info, or nullptr if no buffer available
   */
  BufferInfo* acquire_buffer();

  /**
   * @brief Release a buffer back to the pool
   * @param buffer Buffer to release
   */
  void release_buffer(BufferInfo* buffer);

  /**
   * @brief Get GPU device pointer for a buffer
   * @param buffer Buffer info
   * @return Device pointer (can be cast to uint8_t* for CUDA operations)
   */
  uint8_t* get_device_ptr(const BufferInfo* buffer) const;

  /**
   * @brief Get the current process ID
   */
  int32_t get_pid() const { return pid_; }

  /**
   * @brief Get the device ID
   */
  int get_device_id() const { return device_id_; }

  /**
   * @brief Get the last error message
   */
  const char* get_last_error() const { return last_error_.c_str(); }

  /**
   * @brief Get buffer size
   */
  size_t get_buffer_size() const { return buffer_size_; }

private:
  /**
   * @brief Create a single IPC buffer with shareable handle
   */
  bool create_ipc_buffer(BufferInfo& buffer);

  /**
   * @brief Destroy a single IPC buffer
   */
  void destroy_ipc_buffer(BufferInfo& buffer);

  /**
   * @brief Generate unique ID for buffer
   */
  std::string generate_uid();

  size_t num_buffers_;
  size_t buffer_size_;
  int device_id_;
  int32_t pid_;

  std::vector<BufferInfo> buffers_;
  size_t next_buffer_idx_{0};
  std::mutex mutex_;

  bool initialized_{false};
  std::string last_error_;

  // CUDA VMM properties
  CUmemAllocationProp allocation_prop_{};
  CUmemAccessDesc access_desc_{};
  size_t granularity_{0};

  // Counter for unique IDs
  uint64_t uid_counter_{0};
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__IPC_BUFFER_MANAGER_HPP_
