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
bagwiz generate video -i <input> -t <topic> -o <output> [OPTIONS]
```

### Options

| Flag                      | Description                                                                                                                                                                                                                                                                                                                      |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                       |
| `-t`, `--topic <topic>`   | Image topic to render. Supported types: `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG).                                                                                                                                                                                               |
| `-o`, `--output <output>` | Output video path. Extension selects the container/codec: `.mp4`/`.mkv`/`.mov` -> H.264, `.avi` -> MJPEG.                                                                                                                                                                                                                        |
| `--cam-info`              | `sensor_msgs/msg/CameraInfo` topic for `--undistort` and `--pcd`. When omitted, bagwiz derives it from `<topic>` (`/image_raw`, `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`).                                                                            |
| `--undistort`             | Apply distortion correction to each frame using the resolved CameraInfo. Requires a camera-info topic.                                                                                                                                                                                                                           |
| `--pcd`                   | `sensor_msgs/msg/PointCloud2` topic(s) to project onto each frame. Repeatable; every listed topic is projected into the camera frame and drawn with the same field, color scheme, point size, and alpha. Implies distortion correction and requires a CameraInfo topic and a TF chain from each cloud frame to the camera frame. |
| `--field`                 | Point-cloud field used for coloring: `x`, `y`, `z`, `distance` (default), `intensity`.                                                                                                                                                                                                                                           |
| `--min`                   | Manual minimum value for field normalization (default: auto-computed from the point-cloud span).                                                                                                                                                                                                                                 |
| `--max`                   | Manual maximum value for field normalization (default: auto-computed from the point-cloud span).                                                                                                                                                                                                                                 |
| `--scheme`                | Color scheme for point coloring: `viridis` (default), `turbo`, `jet`, `plasma`, `inferno`, `magma`, `rainbow`.                                                                                                                                                                                                                   |
| `--point-size`            | Side length of drawn square points in pixels (default: 2, range: 1-64).                                                                                                                                                                                                                                                          |
| `--alpha`                 | Point overlay opacity, 0.0-1.0 (default: 1.0).                                                                                                                                                                                                                                                                                   |
| `--resize`                | Scale the output width and height by this factor while preserving aspect ratio. 1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them. Camera intrinsics are scaled accordingly so `--undistort` and `--pcd` stay aligned. (range: 0.01-10.0, default: 1.0)                                                  |
| `-w`, `--overwrite`       | Replace an existing `<output>`. Without it, an existing output path stops the run.                                                                                                                                                                                                                                               |

### Behavior

- **Frame rate** is derived from the topic's message timestamps; a topic with
  fewer than two distinct timestamps falls back to 10 fps.
- **Point-cloud overlay time alignment.** Each frame is paired with the point
  cloud whose `header.stamp` (sensor capture time) is nearest the image's own
  `header.stamp`, rather than the bag record time — so overlays stay aligned even
  when recording latency differs between the camera and lidar. If either the
  camera frame or the point-cloud topic leaves `header.stamp` unset, that pairing
  falls back to matching by bag record time on **both** sides, so the two are
  always compared on the same clock rather than mixing capture time with record
  time.
- **Geometry and encoding are locked to the first frame.** A later frame with a
  different resolution or pixel encoding stops the run.
- **Streaming output.** Frames are decoded and encoded one at a time; the video
  is written to a temporary file and atomically moved into place on success. A
  failed run leaves no partial output or leftover temporary file.
- Dimensions must be even (the 4:2:0 pixel formats these codecs use require it).

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw -o out.mp4

# Render to MJPEG AVI, replacing an existing file.
bagwiz generate video -i drive_dir/ -t /sensing/camera/image_raw -o clip.avi -w

# Render with distortion correction.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --undistort

# Render with distortion correction using an explicit CameraInfo topic.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw -o out.mp4 \
  --undistort --cam-info /sensing/camera/camera_info

# Render with a point-cloud overlay colored by distance.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render with multiple point-cloud overlays in the same camera view.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 \
  --pcd /sensing/lidar/front/points \
  --pcd /sensing/lidar/rear/points \
  --field distance --scheme turbo --point-size 3 --alpha 0.8

# Render at half resolution to reduce output file size.
bagwiz generate video -i drive.mcap -t /sensing/camera/image_raw/compressed -o out.mp4 --resize 0.5
```

## Exit status

| Code | Meaning                                                                                                                            |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The video was written successfully.                                                                                                |
| `1`  | A runtime error occurred (the topic could not be rendered or the video could not be written). Check stderr for the specific cause. |

Argument and parse errors never reach the command's runtime: CLI11 exits
directly with its own non-zero codes (e.g. 106 for a missing required
flag, 105 for a failed validation).
