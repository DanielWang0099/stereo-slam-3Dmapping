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

#include "unity_nitros_bridge/nvjpeg_decoder.hpp"

#include <cstring>
#include <sstream>

namespace unity_nitros_bridge
{

// Helper macro for NVJPEG error checking
#define NVJPEG_CHECK(call) \
  do { \
    nvjpegStatus_t status = call; \
    if (status != NVJPEG_STATUS_SUCCESS) { \
      std::stringstream ss; \
      ss << "NVJPEG error " << status << " at " << __FILE__ << ":" << __LINE__; \
      set_error(ss.str().c_str()); \
      return false; \
    } \
  } while(0)

#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
      std::stringstream ss; \
      ss << "CUDA error " << cudaGetErrorString(err) << " at " << __FILE__ << ":" << __LINE__; \
      set_error(ss.str().c_str()); \
      return false; \
    } \
  } while(0)

NvjpegDecoder::NvjpegDecoder(nvjpegOutputFormat_t output_format)
: output_format_(output_format)
{
  nvjpegStatus_t status;

  // Create NVJPEG handle
  status = nvjpegCreateSimple(&nvjpeg_handle_);
  if (status != NVJPEG_STATUS_SUCCESS) {
    set_error("Failed to create NVJPEG handle");
    return;
  }

  // Create JPEG state
  status = nvjpegJpegStateCreate(nvjpeg_handle_, &nvjpeg_state_);
  if (status != NVJPEG_STATUS_SUCCESS) {
    set_error("Failed to create NVJPEG state");
    nvjpegDestroy(nvjpeg_handle_);
    nvjpeg_handle_ = nullptr;
    return;
  }

  initialized_ = true;
}

NvjpegDecoder::~NvjpegDecoder()
{
  if (pinned_buffer_) {
    cudaFreeHost(pinned_buffer_);
    pinned_buffer_ = nullptr;
  }

  if (nvjpeg_state_) {
    nvjpegJpegStateDestroy(nvjpeg_state_);
    nvjpeg_state_ = nullptr;
  }

  if (nvjpeg_handle_) {
    nvjpegDestroy(nvjpeg_handle_);
    nvjpeg_handle_ = nullptr;
  }

  initialized_ = false;
}

void NvjpegDecoder::set_error(const char* msg)
{
  last_error_ = msg;
}

bool NvjpegDecoder::ensure_pinned_buffer(size_t size)
{
  if (pinned_buffer_size_ >= size) {
    return true;
  }

  if (pinned_buffer_) {
    cudaFreeHost(pinned_buffer_);
    pinned_buffer_ = nullptr;
    pinned_buffer_size_ = 0;
  }

  // Allocate with some extra space to avoid frequent reallocations
  size_t alloc_size = size + (size / 4);  // 25% extra
  
  cudaError_t err = cudaMallocHost(&pinned_buffer_, alloc_size);
  if (err != cudaSuccess) {
    set_error("Failed to allocate pinned memory");
    return false;
  }

  pinned_buffer_size_ = alloc_size;
  return true;
}

bool NvjpegDecoder::get_image_info(
  const uint8_t* jpeg_data,
  size_t jpeg_size,
  int* width,
  int* height,
  int* channels,
  int* subsampling)
{
  if (!initialized_) {
    set_error("Decoder not initialized");
    return false;
  }

  int nComponents;
  nvjpegChromaSubsampling_t chromaSubsampling;
  int widths[NVJPEG_MAX_COMPONENT];
  int heights[NVJPEG_MAX_COMPONENT];

  NVJPEG_CHECK(nvjpegGetImageInfo(
    nvjpeg_handle_,
    jpeg_data,
    jpeg_size,
    &nComponents,
    &chromaSubsampling,
    widths,
    heights));

  if (width) *width = widths[0];
  if (height) *height = heights[0];
  if (channels) *channels = nComponents;
  if (subsampling) *subsampling = static_cast<int>(chromaSubsampling);

  return true;
}

bool NvjpegDecoder::decode(
  const uint8_t* jpeg_data,
  size_t jpeg_size,
  uint8_t* output_device_ptr,
  size_t output_pitch,
  cudaStream_t stream)
{
  if (!initialized_) {
    set_error("Decoder not initialized");
    return false;
  }

  if (!jpeg_data || jpeg_size == 0) {
    set_error("Invalid JPEG data");
    return false;
  }

  if (!output_device_ptr) {
    set_error("Invalid output buffer");
    return false;
  }

  // Get image info first
  int width, height, channels;
  if (!get_image_info(jpeg_data, jpeg_size, &width, &height, &channels, nullptr)) {
    return false;
  }

  // Prepare output structure
  nvjpegImage_t output_image;
  memset(&output_image, 0, sizeof(output_image));

  // For interleaved output (RGBI), all data goes to channel[0]
  if (output_format_ == NVJPEG_OUTPUT_RGBI || output_format_ == NVJPEG_OUTPUT_BGRI) {
    output_image.channel[0] = output_device_ptr;
    output_image.pitch[0] = output_pitch;
  } else {
    // Planar output - would need separate channel pointers
    set_error("Only interleaved output format is currently supported");
    return false;
  }

  // Copy JPEG data to pinned memory for faster GPU transfer
  if (!ensure_pinned_buffer(jpeg_size)) {
    return false;
  }
  memcpy(pinned_buffer_, jpeg_data, jpeg_size);

  // Decode
  nvjpegStatus_t status = nvjpegDecode(
    nvjpeg_handle_,
    nvjpeg_state_,
    pinned_buffer_,
    jpeg_size,
    output_format_,
    &output_image,
    stream);

  if (status != NVJPEG_STATUS_SUCCESS) {
    std::stringstream ss;
    ss << "NVJPEG decode failed with status " << status;
    set_error(ss.str().c_str());
    return false;
  }

  // Synchronize if no stream provided
  if (stream == nullptr) {
    CUDA_CHECK(cudaDeviceSynchronize());
  }

  return true;
}

bool NvjpegDecoder::decode_alloc(
  const uint8_t* jpeg_data,
  size_t jpeg_size,
  uint8_t** output_device_ptr,
  int* width,
  int* height,
  int* channels,
  cudaStream_t stream)
{
  if (!initialized_) {
    set_error("Decoder not initialized");
    return false;
  }

  // Get image dimensions
  int w, h, c;
  if (!get_image_info(jpeg_data, jpeg_size, &w, &h, &c, nullptr)) {
    return false;
  }

  // For interleaved RGB output, we need width * height * 3 bytes
  int output_channels = 3;  // RGB interleaved
  size_t image_size = w * h * output_channels;
  size_t pitch = w * output_channels;

  // Allocate device memory
  uint8_t* device_ptr = nullptr;
  CUDA_CHECK(cudaMalloc(&device_ptr, image_size));

  // Decode
  if (!decode(jpeg_data, jpeg_size, device_ptr, pitch, stream)) {
    cudaFree(device_ptr);
    return false;
  }

  // Return results
  *output_device_ptr = device_ptr;
  if (width) *width = w;
  if (height) *height = h;
  if (channels) *channels = output_channels;

  return true;
}

}  // namespace unity_nitros_bridge
