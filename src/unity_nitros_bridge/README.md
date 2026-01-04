# Unity NITROS Bridge

GPU-accelerated NITROS bridge for Unity simulation, providing **TRUE zero-copy JPEG decoding** using NVJPEG with CUDA IPC for cross-process GPU memory sharing.

## Overview

This package provides TRUE zero-copy GPU pipeline using CUDA IPC:

1. **NitrosBridgeJpegDecoderNode** - Decodes JPEG on GPU, publishes NitrosBridgeImage with CUDA IPC handles
2. **ImageConverterNode** - Imports GPU memory via CUDA IPC, publishes NitrosImage for NITROS pipeline
3. **NitrosJpegDecoderNode** (Legacy) - Direct NVJPEG decode with ManagedNitrosPublisher (has 1 CPU copy)

## TRUE Zero-Copy Architecture (CUDA IPC)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    TRUE ZERO-COPY GPU PIPELINE                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  CompressedImage ──► NitrosBridgeJpegDecoderNode ──► NitrosBridgeImage     │
│  (JPEG bytes)        │                               {PID, FD}             │
│                      │ NVJPEG decode                                        │
│                      │ → IPCBufferManager pool                              │
│                      │ → cuMemExportToShareableHandle                       │
│                      ↓                                                      │
│                  [GPU Memory with CUDA IPC Handle]                         │
│                      │                                                      │
│                      ↓                                                      │
│  NitrosBridgeImage ──► ImageConverterNode ──► NitrosImage ──► ESS/VSLAM   │
│  {PID, FD}           │                        (GPU ptr)                    │
│                      │ pidfd_getfd()                                        │
│                      │ cuMemImportFromShareableHandle                       │
│                      │ NitrosImageBuilder::WithGpuData()                    │
│                      ↓                                                      │
│                  [Same GPU Memory - ZERO COPY!]                            │
│                                                                             │
│  ❌ NO GPU→CPU copy                                                         │
│  ❌ NO CPU→GPU copy                                                         │
│  ❌ NO serialization                                                        │
│  ✅ Direct GPU pointer via CUDA IPC                                         │
│  ✅ Ring buffer pool (no per-frame allocation)                              │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Nodes

### NitrosBridgeJpegDecoderNode (Recommended - TRUE Zero-Copy)

**Composable node** for GPU JPEG decoding with CUDA IPC output.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_compressed_topic` | string | `image/compressed` | Input compressed image topic |
| `input_camera_info_topic` | string | `camera_info` | Input camera info topic |
| `output_bridge_image_topic` | string | `nitros_bridge_image` | Output NitrosBridgeImage topic |
| `output_camera_info_topic` | string | `camera_info_out` | Output camera info topic |
| `num_buffers` | int | `8` | Number of GPU buffers in ring pool |
| `device_id` | int | `0` | CUDA device ID |

### ImageConverterNode

**Composable node** that converts NitrosBridgeImage → NitrosImage via CUDA IPC.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_bridge_topic` | string | `nitros_bridge_image` | Input NitrosBridgeImage topic |
| `output_image_topic` | string | `image_raw` | Output NitrosImage topic |
| `device_id` | int | `0` | CUDA device ID |

### NitrosJpegDecoderNode (Legacy)

**Composable node** for GPU JPEG decoding with NITROS output (has 1 CPU copy).

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_compressed_topic` | string | `image/compressed` | Input compressed image topic |
| `input_camera_info_topic` | string | `camera_info` | Input camera info topic |
| `output_image_topic` | string | `image_raw` | Output decoded image topic (NITROS) |
| `output_camera_info_topic` | string | `camera_info` | Output camera info topic |
| `output_encoding` | string | `rgb8` | Output pixel format |

#### Subscribed Topics

- `<input_compressed_topic>` (`sensor_msgs/CompressedImage`)
- `<input_camera_info_topic>` (`sensor_msgs/CameraInfo`)

#### Published Topics

- `<output_image_topic>` (`sensor_msgs/Image` via NITROS)
- `<output_camera_info_topic>` (`sensor_msgs/CameraInfo`)

## Usage

### In Launch File (Composable)

```python
ComposableNode(
    package="unity_nitros_bridge",
    plugin="unity_nitros_bridge::NitrosJpegDecoderNode",
    name="jpeg_decoder",
    parameters=[
        {"input_compressed_topic": "/camera/compressed"},
        {"output_image_topic": "/camera/image_raw"},
    ],
),
```

### Building

```bash
colcon build --packages-select unity_nitros_bridge
```

## Dependencies

- CUDA Toolkit with NVJPEG
- isaac_ros_nitros
- isaac_ros_managed_nitros
- isaac_ros_nitros_image_type

## License

Apache-2.0
