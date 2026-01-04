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

#ifndef UNITY_NITROS_BRIDGE__NVJPEG_DECODER_HPP_
#define UNITY_NITROS_BRIDGE__NVJPEG_DECODER_HPP_

#include <cuda_runtime.h>
#include <nvjpeg.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace unity_nitros_bridge
{

/**
 * @brief GPU-accelerated JPEG decoder using NVJPEG.
 * 
 * Decodes JPEG images entirely on GPU, producing output in device memory
 * suitable for zero-copy NITROS publishing.
 */
class NvjpegDecoder
{
public:
  /**
   * @brief Construct a new NVJPEG decoder.
   * @param output_format Output pixel format (NVJPEG_OUTPUT_RGBI for interleaved RGB)
   */
  explicit NvjpegDecoder(nvjpegOutputFormat_t output_format = NVJPEG_OUTPUT_RGBI);
  
  ~NvjpegDecoder();

  // Non-copyable
  NvjpegDecoder(const NvjpegDecoder&) = delete;
  NvjpegDecoder& operator=(const NvjpegDecoder&) = delete;

  /**
   * @brief Decode a JPEG image on GPU.
   * 
   * @param jpeg_data Pointer to JPEG data (can be host or device memory)
   * @param jpeg_size Size of JPEG data in bytes
   * @param output_device_ptr Pre-allocated device memory for decoded image
   * @param output_pitch Row pitch of output buffer
   * @param stream CUDA stream for async execution
   * @return true if decoding succeeded
   */
  bool decode(
    const uint8_t* jpeg_data,
    size_t jpeg_size,
    uint8_t* output_device_ptr,
    size_t output_pitch,
    cudaStream_t stream = nullptr);

  /**
   * @brief Decode JPEG and allocate output buffer.
   * 
   * @param jpeg_data Pointer to JPEG data
   * @param jpeg_size Size of JPEG data in bytes
   * @param[out] output_device_ptr Allocated device memory (caller must free)
   * @param[out] width Decoded image width
   * @param[out] height Decoded image height
   * @param[out] channels Number of channels (3 for RGB)
   * @param stream CUDA stream for async execution
   * @return true if decoding succeeded
   */
  bool decode_alloc(
    const uint8_t* jpeg_data,
    size_t jpeg_size,
    uint8_t** output_device_ptr,
    int* width,
    int* height,
    int* channels,
    cudaStream_t stream = nullptr);

  /**
   * @brief Get image dimensions without decoding.
   */
  bool get_image_info(
    const uint8_t* jpeg_data,
    size_t jpeg_size,
    int* width,
    int* height,
    int* channels,
    int* subsampling);

  /// Get last error message
  const char* get_last_error() const { return last_error_.c_str(); }

  /// Check if decoder is initialized
  bool is_initialized() const { return initialized_; }

private:
  bool initialized_{false};
  nvjpegHandle_t nvjpeg_handle_{nullptr};
  nvjpegJpegState_t nvjpeg_state_{nullptr};
  nvjpegOutputFormat_t output_format_;
  std::string last_error_;

  // Pinned memory for host-to-device transfer
  uint8_t* pinned_buffer_{nullptr};
  size_t pinned_buffer_size_{0};

  void set_error(const char* msg);
  bool ensure_pinned_buffer(size_t size);
};

}  // namespace unity_nitros_bridge

#endif  // UNITY_NITROS_BRIDGE__NVJPEG_DECODER_HPP_
