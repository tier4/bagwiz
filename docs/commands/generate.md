# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                        | What it does                           |
| --------------------------------- | -------------------------------------- |
| [`video`](#bagwiz-generate-video) | Render an image topic to a video file. |

---

## `bagwiz generate video`

Render the frames of an image topic into a video file. The frame rate is
derived from the messages' own timestamps, and the container/codec is chosen
from the `<output>` file extension. An optional PointCloud2 topic can be
projected onto each frame and colored by a selected field.

### Usage

```text
bagwiz generate video [OPTIONS] <input> <image_topic> <output>
```

### Positional arguments

| Name            | Description                                                           |
| --------------- | --------------------------------------------------------------------- |
| `input`         | Input ROS 2 rosbag (directory or single-file). Must exist.            |
| `<image_topic>` | Image topic to render. Must exist in the bag and be a supported type. |
| `output`        | Output video path. Its extension selects the container/codec.         |

### Options

| Flag                | Description                                                                                                                          |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `--pcd`             | PointCloud2 topic to project onto the video. Requires a CameraInfo topic and a TF chain linking the cloud frame to the camera frame. |
| `--cam-info`        | CameraInfo topic to use for `--undistort` and point-cloud projection. When omitted, bagwiz derives it from `<image_topic>`.          |
| `--undistort`       | Apply distortion correction to each frame using the resolved CameraInfo. Requires a camera-info topic.                               |
| `--field`           | Point field used for coloring when `--pcd` is given: `x`, `y`, `z`, `distance` (default), `intensity`.                               |
| `--min`             | Manual minimum value for field normalization (default: auto-computed from the point-cloud span).                                     |
| `--max`             | Manual maximum value for field normalization (default: auto-computed from the point-cloud span).                                     |
| `--scheme`          | Color scheme for point coloring: `viridis` (default), `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`.                       |
| `--point-size`      | Diameter of drawn points in pixels (default: 2, range: 1-64).                                                                        |
| `--alpha`           | Point overlay opacity, 0.0-1.0 (default: 1.0).                                                                                       |
| `-w`, `--overwrite` | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                   |

### Supported topic types

| Message type                      | Status                                           |
| --------------------------------- | ------------------------------------------------ |
| `sensor_msgs/msg/Image`           | Rendered. Pixel encodings: `bgr8`, `rgb8`.       |
| `sensor_msgs/msg/CompressedImage` | Rendered. JPEG / PNG, decoded to BGR internally. |

Other message types stop the run with a clear error. A `CompressedImage` whose
payload is neither JPEG nor PNG (by its leading magic bytes) is likewise
rejected.

### CameraInfo auto-resolution

When `--cam-info` is not given, `generate video` attempts to find a sibling
`sensor_msgs/msg/CameraInfo` topic from the `<image_topic>` name:

| `<image_topic>` suffix         | Resolved CameraInfo topic |
| ------------------------------ | ------------------------- |
| `/image_raw/compressed`        | `<prefix>/camera_info`    |
| `/image_rect_color`            | `<prefix>/camera_info`    |
| `/image_rect_color/compressed` | `<prefix>/camera_info`    |

If the resolved topic does not exist or is not a `sensor_msgs/msg/CameraInfo`,
it is treated as unresolved. That is fine for plain rendering, but when
`--undistort` is set or `--pcd` is given, a camera-info topic is required, so
the run stops and asks you to pass `--cam-info` explicitly.

### Output format

The container and codec are inferred from the `<output>` extension:

| Extension              | Container            | Codec           |
| ---------------------- | -------------------- | --------------- |
| `.mp4`, `.mkv`, `.mov` | MP4 / Matroska / MOV | H.264 (libx264) |
| `.avi`                 | AVI                  | MJPEG           |

Any other extension stops the run. H.264 requires the FFmpeg build to ship the
`libx264` encoder; if it is missing, use an `.avi` output (MJPEG is always
available) or rebuild FFmpeg with libx264.

Some hardware decoders (notably mpv's Vulkan hwdec path) can crash on H.264
output. If mpv fails to play, try VLC or run `mpv --hwdec=no`. Use `.avi`
(MJPEG) only if lower quality is acceptable.

### Behavior

- **Frame rate** is derived from the topic's message timestamps so the video's
  duration matches the recording: `fps = (count - 1) / (last - first)`. A topic
  with fewer than two messages, or all messages at one timestamp, falls back to
  10 fps.
- **Geometry and encoding are locked to the first frame.** If a later frame has
  a different resolution or pixel encoding, the run stops with an error rather
  than producing a malformed video.
- **Streaming, storage-safe output.** Frames are decoded and encoded one at a
  time — the whole bag is never held in memory, and no intermediate frame files
  are written. The video is encoded to a sibling temporary file and atomically
  moved into place on success; a failed run leaves **no** partial output and
  **no** leftover temporary file.
- **Undistortion** (`--undistort`) reads the first `sensor_msgs/msg/CameraInfo`
  message from the resolved camera-info topic and applies OpenCV distortion
  correction to every frame before encoding. This requires a camera-info topic;
  use `--cam-info` when auto-resolution fails.
- **Point-cloud overlay** (`--pcd`) projects the nearest PointCloud2 message
  onto each image frame using the resolved CameraInfo and the TF chain from the
  cloud frame to the camera frame. Points are colored by `--field` (normalized
  with `--min`/`--max` or auto-computed across the bag) and drawn with `--scheme`,
  `--point-size`, and `--alpha`. This requires a camera-info topic and at least
  one `tf2_msgs/msg/TFMessage` topic (e.g. `/tf_static`).
- Dimensions must be even (the 4:2:0 pixel formats these codecs use require it).

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video drive.mcap /sensing/camera/image_raw out.mp4

# Render to MJPEG AVI (no libx264 needed), replacing an existing file.
bagwiz generate video drive_dir/ /sensing/camera/image_raw clip.avi -w

# Render with distortion correction (auto-resolves /sensing/camera/camera_info).
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 --undistort

# Render with distortion correction using an explicit CameraInfo topic.
bagwiz generate video drive.mcap /sensing/camera/image_raw out.mp4 \
  --undistort --cam-info /sensing/camera/camera_info

# Render with a point-cloud overlay colored by distance.
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8
```

## Exit status

| Code | Meaning                                                                    |
| ---- | -------------------------------------------------------------------------- |
| `0`  | The video was written successfully.                                        |
| `1`  | A runtime or argument error occurred. Check stderr for the specific cause. |
