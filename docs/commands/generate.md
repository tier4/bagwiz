# `bagwiz generate`

Generate non-rosbag **media** from a rosbag. Unlike `convert` or `topic` (which
read a bag and write another bag), `generate` reads a bag and produces a
different kind of artifact. Subcommands:

| Subcommand                        | What it does                           |
| --------------------------------- | -------------------------------------- |
| [`video`](#bagwiz-generate-video) | Render an image topic to a video file. |

---

## `bagwiz generate video`

Render an image topic from a rosbag to a video file. The frame rate is derived
from message timestamps, and the container/codec is chosen from the `<output>`
extension.

### Usage

```text
bagwiz generate video [OPTIONS] <input> <img_topic> <output>
```

### Positional arguments

| Name        | Description                                                                                                                        |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `input`     | Input ROS 2 rosbag (directory or single-file). Must exist.                                                                         |
| `img_topic` | Image topic to render. Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). |
| `output`    | Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                          |

### Options

| Flag                | Description                                                                                                                                                                                                                                                                                                                      |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--cam-info`        | `sensor_msgs/msg/CameraInfo` topic for `--undistort` and `--pcd`. When omitted, bagwiz derives it from `<image_topic>` (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`).                                                                      |
| `--undistort`       | Apply distortion correction to each frame using the resolved CameraInfo. Requires a camera-info topic.                                                                                                                                                                                                                           |
| `--pcd`             | `sensor_msgs/msg/PointCloud2` topic(s) to project onto each frame. Repeatable; every listed topic is projected into the camera frame and drawn with the same field, color scheme, point size, and alpha. Implies distortion correction and requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. |
| `--field`           | Point-cloud field used for coloring: `x`, `y`, `z`, `distance` (default), `intensity`.                                                                                                                                                                                                                                           |
| `--min`             | Manual minimum value for field normalization (default: auto-computed from the point-cloud span).                                                                                                                                                                                                                                 |
| `--max`             | Manual maximum value for field normalization (default: auto-computed from the point-cloud span).                                                                                                                                                                                                                                 |
| `--scheme`          | Color scheme for point coloring: `viridis` (default), `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`.                                                                                                                                                                                                                   |
| `--point-size`      | Diameter of drawn points in pixels (default: 2, range: 1-64).                                                                                                                                                                                                                                                                    |
| `--alpha`           | Point overlay opacity, 0.0-1.0 (default: 1.0).                                                                                                                                                                                                                                                                                   |
| `--resize`          | Scale the output width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so `--undistort` and `--pcd` stay aligned.                                                                                   |
| `-w`, `--overwrite` | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                               |

### Behavior

- **Frame rate** is derived from the topic's message timestamps; a topic with
  fewer than two distinct timestamps falls back to 10 fps.
- **Geometry and encoding are locked to the first frame.** A later frame with a
  different resolution or pixel encoding stops the run.
- **Streaming output.** Frames are decoded and encoded one at a time; the video
  is written to a temporary file and atomically moved into place on success. A
  failed run leaves no partial output or leftover temporary file.
- Dimensions must be even (the 4:2:0 pixel formats these codecs use require it).

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video drive.mcap /sensing/camera/image_raw out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz generate video drive_dir/ /sensing/camera/image_raw clip.avi -w

# Render with distortion correction.
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 --undistort

# Render with distortion correction using an explicit CameraInfo topic.
bagwiz generate video drive.mcap /sensing/camera/image_raw out.mp4 \
  --undistort --cam-info /sensing/camera/camera_info

# Render with a point-cloud overlay colored by distance.
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with multiple point-cloud overlays in the same camera view.
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 \
  --pcd /sensing/lidar/front/points \
  --pcd /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render at half resolution to reduce output file size.
bagwiz generate video drive.mcap /sensing/camera/image_raw/compressed out.mp4 --resize 0.5
```

## Exit status

| Code | Meaning                                                                    |
| ---- | -------------------------------------------------------------------------- |
| `0`  | The video was written successfully.                                        |
| `1`  | A runtime or argument error occurred. Check stderr for the specific cause. |
