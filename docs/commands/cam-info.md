# `bagwiz cam-info`

Operations on `sensor_msgs/msg/CameraInfo` topics. Subcommands:

| Subcommand                            | What it does                                                                 |
| ------------------------------------- | ---------------------------------------------------------------------------- |
| [`replace`](#bagwiz-cam-info-replace) | Overwrite a CameraInfo topic's calibration with the values from a YAML file. |

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz cam-info replace`

Replace the calibration carried by a single `sensor_msgs/msg/CameraInfo` topic
with the values from a standard ROS camera calibration YAML file — the kind
produced by the `camera_calibration` package and consumed by
`camera_info_manager`. This is the offline equivalent of re-recording the bag
with a corrected calibration: useful when a bag was captured with a wrong or
placeholder calibration.

For every message on the chosen topic, the calibration fields are taken from the
YAML while each message's own `header` timestamp, `header.frame_id` (unless
`--frame-id` is given), `binning_x` / `binning_y`, and `roi` are preserved. Every
other topic in the bag is copied verbatim.

### Usage

```text
bagwiz cam-info replace [OPTIONS] <input> <calib_yaml> <topic>
```

The operand order follows the repository convention (read-side operands first).
`<input>` doubles as the write-side target: without `-o` the bag is rewritten in
place, mirroring `bagwiz traj join` and `bagwiz convert msg geo`.

### Positional arguments

| Name         | Description                                                                         |
| ------------ | ----------------------------------------------------------------------------------- |
| `input`      | Input ROS 2 rosbag (directory or single-file). Must exist.                          |
| `calib_yaml` | Camera calibration YAML in the `camera_calibration` / `camera_info_manager` format. |
| `topic`      | The CameraInfo topic to rewrite. Its type must be `sensor_msgs/msg/CameraInfo`.     |

### Options

| Flag                 | Description                                                                                           |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| `--frame-id <id>`    | Override `header.frame_id` on the rewritten messages. When omitted, each message keeps its frame_id.  |
| `-o`, `--output <p>` | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `-w`, `--overwrite`  | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### YAML format and field mapping

The input is the standard camera calibration YAML. Each matrix block is a
mapping of `rows`, `cols`, and a flat `data` sequence in row-major order:

```yaml
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.01, -0.02, 0.003, 0.004, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

| YAML key                  | CameraInfo field   | Required size        |
| ------------------------- | ------------------ | -------------------- |
| `image_width`             | `width`            | scalar               |
| `image_height`            | `height`           | scalar               |
| `distortion_model`        | `distortion_model` | string               |
| `distortion_coefficients` | `d`                | any non-empty `data` |
| `camera_matrix`           | `k`                | 9 (`data`, 3×3)      |
| `rectification_matrix`    | `r`                | 9 (`data`, 3×3)      |
| `projection_matrix`       | `p`                | 12 (`data`, 3×4)     |

`camera_name`, if present, is informational only — it is not a CameraInfo field
and is ignored. Each block's declared `rows * cols` must match its `data` length,
and `k` / `r` / `p` must yield exactly 9 / 9 / 12 values; otherwise the run stops
with an error before the bag is touched.

### Examples

Fix a camera's intrinsics in place:

```bash
bagwiz cam-info replace drive.mcap left_camera.yaml /camera/left/camera_info
```

Write a corrected copy and also relabel the frame, leaving the input untouched:

```bash
bagwiz cam-info replace drive.mcap left.yaml /camera/left/camera_info \
  --frame-id camera_left_optical_frame -o drive_fixed.mcap
```

### Notes

- Only the named topic is rewritten; the topic's message type is unchanged, so
  the bag's other topics and metadata are preserved exactly.
- The output bag is always written uncompressed (re-compress later with
  `ros2 bag convert` if needed).
- In-place mode replaces the input atomically via a sibling temporary bag, in the
  same storage backend and layout as the input.
