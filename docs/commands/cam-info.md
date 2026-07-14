# `bagwiz cam-info`

Operations on `sensor_msgs/msg/CameraInfo` topics. Subcommands:

| Subcommand                                    | What it does                                                                           |
| --------------------------------------------- | -------------------------------------------------------------------------------------- |
| [`replace`](#bagwiz-cam-info-replace)         | Overwrite one or more CameraInfo topics' calibration with the values from a YAML file. |
| [`recompute-p`](#bagwiz-cam-info-recompute-p) | Recompute the projection matrix from the intrinsics, in a YAML file or in a bag.       |

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz cam-info replace`

Replace the calibration carried by one or more `sensor_msgs/msg/CameraInfo`
topics with the values from a standard ROS camera calibration YAML file — the
kind produced by the `camera_calibration` package and consumed by
`camera_info_manager`. This is the offline equivalent of re-recording the bag
with a corrected calibration: useful when a bag was captured with a wrong or
placeholder calibration.

When several topics are listed, the **same** YAML calibration is applied to every
one of them — handy when one calibration is shared across topics (for example a
`/camera_info` and a republished `/camera_info_throttled`). To give different
topics different calibrations, run the command once per topic/YAML pair.

For every message on the chosen topic(s), the calibration fields are taken from
the YAML while each message's own `header` timestamp, `header.frame_id` (unless
`--frame-id` is given), `binning_x` / `binning_y`, and `roi` are preserved. Every
other topic in the bag is copied verbatim.

### Usage

```text
bagwiz cam-info replace [OPTIONS] <input> <calib_yaml> <topic>...
```

The operand order follows the repository convention (read-side operands first).
`<input>` doubles as the write-side target: without `-o` the bag is rewritten in
place, mirroring `bagwiz traj join` and `bagwiz convert msg geo`. One or more
`<topic>` operands may be given; each must be a `sensor_msgs/msg/CameraInfo`
topic.

### Positional arguments

| Name         | Description                                                                                                                     |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `input`      | Input ROS 2 rosbag (directory or single-file). Must exist.                                                                      |
| `calib_yaml` | Camera calibration YAML in the `camera_calibration` / `camera_info_manager` format.                                             |
| `topic...`   | One or more CameraInfo topics to rewrite. Each type must be `sensor_msgs/msg/CameraInfo`; the same YAML applies to all of them. |

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

Apply one calibration to several CameraInfo topics in a single pass:

```bash
bagwiz cam-info replace drive.mcap shared.yaml \
  /camera/camera_info /camera/camera_info_throttled
```

### Notes

- Only the named topics are rewritten; their message type is unchanged, so the
  bag's other topics and metadata are preserved exactly.
- The same `<calib_yaml>` is applied to every listed topic. Pass different
  calibrations by running the command once per topic.
- Listing a topic more than once is harmless — duplicates are de-duplicated, and
  a listed topic that carries no messages is reported with a warning.
- The output bag is always written uncompressed (re-compress later with
  `ros2 bag convert` if needed).
- In-place mode replaces the input atomically via a sibling temporary bag, in the
  same storage backend and layout as the input.

---

## `bagwiz cam-info recompute-p`

Recompute a projection matrix from the intrinsics it belongs to. `p` is derived
as:

```text
p = [ cv::getOptimalNewCameraMatrix(k, d, (width, height), alpha) | 0 ]
```

so `k`, `d`, and the image size are the **inputs** — everything else in the file
or message is preserved. This is the same computation the `camera_calibration`
package performs when it writes a monocular calibration, so it reconstructs a
`projection_matrix` that is missing, was hand-edited to something wrong, or has
gone stale after `k` changed.

Note `p` is **not** `[k | 0]`: undistortion re-maps pixels, so the undistorted
image needs its own focal length and principal point. `alpha` chooses how.

### Usage

```text
bagwiz cam-info recompute-p [OPTIONS] <input>
```

`<input>` says where the calibration comes from, and decides whether `--topics`
applies:

| `<input>`               | Source                         | `--topics`   |
| ----------------------- | ------------------------------ | ------------ |
| a `.yaml` / `.yml` file | The file's own calibration.    | **Rejected** |
| anything else (bag/dir) | The named CameraInfo topic(s). | **Required** |

For a bag, `-o` then decides **what is produced** — a `.yaml` / `.yml` output
means "give me the calibration", not "rewrite the bag":

| `<input>` | `-o`     | Result                                                         |
| --------- | -------- | -------------------------------------------------------------- |
| YAML      | _(none)_ | The YAML is rewritten in place.                                |
| YAML      | YAML     | A recomputed copy is written; the input is untouched.          |
| YAML      | bag      | **Error** — a YAML has no messages to build a bag from.        |
| bag       | _(none)_ | The bag is rewritten in place.                                 |
| bag       | bag      | A rewritten copy is written; the input is untouched.           |
| bag       | YAML     | The topic's calibration is **exported**; the bag is untouched. |

Without `-o` the input is rewritten in place, mirroring `cam-info replace`.

### Positional arguments

| Name    | Description                                                                                                      |
| ------- | ---------------------------------------------------------------------------------------------------------------- |
| `input` | Calibration YAML (`camera_calibration` / `camera_info_manager` format) **or** an input ROS 2 rosbag. Must exist. |

### Options

| Flag                    | Description                                                                                                                          |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `-t`, `--topics <t>...` | Bag input only: one or more `sensor_msgs/msg/CameraInfo` topics whose `p` to recompute. **Required** for a bag, rejected for a YAML. |
| `-a`, `--alpha <a>`     | OpenCV free-scaling parameter in `[0, 1]`. Default `0`.                                                                              |
| `-o`, `--output <p>`    | Write the result to a new path instead of rewriting `<input>` in place.                                                              |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place.                                |

`--topics` is required for a bag rather than defaulting to every CameraInfo
topic: rewriting topics you did not name would be a guess, and `<input>` doubles
as the in-place target. Because the requirement depends on what `<input>` turns
out to be, it is reported when the run starts rather than by the argument parser:

```console
$ bagwiz cam-info recompute-p drive.mcap
[ERROR] 'drive.mcap' is a bag, so --topics is required to say which CameraInfo topic's p to
recompute. Pass -t/--topics <topic>..., or pass a .yaml calibration file as <input>.
```

Tab completion offers the bag's CameraInfo topics — and only those — at every
`--topics` value slot.

### Supported `distortion_model` values

`p` can only be recomputed for the Brown–Conrady family that
`cv::getOptimalNewCameraMatrix` implements:

| `distortion_model`    | Supported | Behavior                                                              |
| --------------------- | --------- | --------------------------------------------------------------------- |
| `plumb_bob`           | ✅        | Brown–Conrady, 5 coefficients. The ROS default.                       |
| `rational_polynomial` | ✅        | The same model with 8 coefficients.                                   |
| `""` (empty) / `none` | ✅        | Declares no lens distortion, so `p` is `[k \| 0]` whatever `d` holds. |
| `equidistant`         | ❌        | Fisheye — see below.                                                  |
| `fisheye`             | ❌        | Fisheye — see below.                                                  |
| anything else         | ❌        | Error.                                                                |

Any other model **stops the run with an error** rather than producing a
best-effort `p`. The model is validated _before_ `d` is examined, so an
unsupported model is refused even when its coefficients happen to be all zero:

```console
$ bagwiz cam-info recompute-p fisheye_cam.yaml
[ERROR] Cannot recompute p for 'fisheye_cam.yaml': distortion_model 'equidistant' is a fisheye
model; its projection matrix comes from cv::fisheye::estimateNewCameraMatrixForUndistortRectify
(which takes a `balance`, not an alpha) and is not supported yet
```

Fisheye is a known gap, not an oversight: its projection matrix comes from
`cv::fisheye::estimateNewCameraMatrixForUndistortRectify`, which is different
maths parameterized by a `balance` rather than an `alpha`. (`bagwiz`'s
point-cloud projector does handle `equidistant`, so the asymmetry is deliberate.)
Nothing is written when a model is rejected.

### Choosing `alpha`

`alpha` trades black borders against cropping in the undistorted image:

| `alpha` | Meaning                                                                                                            |
| ------- | ------------------------------------------------------------------------------------------------------------------ |
| `0`     | Keep only valid pixels — zoom until no black border remains. The `camera_calibration` default, and this command's. |
| `1`     | Retain every source pixel — nothing is cropped, but the edges show black borders.                                  |
| between | A linear trade-off between the two.                                                                                |

### Sub-pixel changes are expected

`cv::getOptimalNewCameraMatrix` gives slightly different answers across OpenCV
versions — between 4.5.4 and 4.13.0 the result moves by up to **0.77 px** on a
1920×1280 `plumb_bob` calibration — and bagwiz builds against each ROS distro's
own OpenCV. So recomputing a `p` that an older `camera_calibration` wrote
**changes it slightly** rather than reproducing it exactly, and the same input
can give marginally different output on different distros.

This is benign: the recomputed `p` is consistent with the OpenCV that this binary
will later feed it to (`generate video --undistort`, the point-cloud overlay).
The run reports how far `p` moved so a small change is legible as version drift
rather than a correction:

```text
p changed by at most 0.767 px (alpha=0.00, OpenCV 4.13.0). A sub-pixel change like this is
cv::getOptimalNewCameraMatrix differing across OpenCV versions, not a corrected calibration.
```

A genuinely wrong `p` is off by tens or hundreds of pixels, so the two are easy
to tell apart.

### When it refuses

Recomputing `p` from `k` is **wrong**, not merely imprecise, in these cases, so
the run stops before anything is written:

| Condition                              | Why                                                                                                                                                                                                                        |
| -------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `r` is a genuine non-identity rotation | The camera is stereo-rectified: its `p` comes from `cv::stereoRectify` against the paired camera. Recomputing would break rectification. An all-zero (unset) `r` is treated as identity, matching the undistortion helper. |
| `p[3]` or `p[7]` is non-zero           | `p` carries a stereo baseline (`p[3] = -fx · baseline`); `[newK \| 0]` would zero it and lose the extrinsic.                                                                                                               |
| `distortion_model` is unsupported      | Only `plumb_bob`, `rational_polynomial`, and an empty / `none` model can be recomputed — see [Supported `distortion_model` values](#supported-distortion_model-values).                                                    |
| `width` or `height` is 0               | No image size means no valid new camera matrix.                                                                                                                                                                            |
| `k` is degenerate or non-finite        | `fx`/`fy` must be positive and every entry finite.                                                                                                                                                                         |

When the model is Brown–Conrady but `d` is empty or all-zero there is nothing to
undistort, so the result is exactly `[k | 0]`.

### Examples

Fix a calibration file's projection matrix in place:

```bash
bagwiz cam-info recompute-p camera_info.yaml
```

Write a corrected copy, keeping all source pixels:

```bash
bagwiz cam-info recompute-p camera_info.yaml --alpha 1.0 -o fixed.yaml
```

Compose with `replace` to push a corrected calibration into a bag:

```bash
bagwiz cam-info recompute-p camera_info.yaml -o fixed.yaml
bagwiz cam-info replace drive.mcap fixed.yaml /camera/camera_info
```

Recompute `p` directly on a bag's CameraInfo topics:

```bash
bagwiz cam-info recompute-p drive.mcap --topics /camera/camera_info -o drive_fixed.mcap
```

Export a bag's calibration as a YAML instead of rewriting the bag — give `-o` a
`.yaml` path:

```bash
bagwiz cam-info recompute-p drive.mcap --topics /camera/camera_info -o camera_info.yaml
```

### Notes

- **YAML mode is a re-emit, not an edit.** Values are preserved (including
  `camera_name`), but comments, key order, and incidental formatting are
  normalized. Use `-o` to keep the original file untouched.
- Giving `-o` a `.yaml` / `.yml` path with a **bag** `<input>` exports that
  topic's calibration instead of rewriting the bag — the bag is opened read-only.
  Exactly one `--topics` entry is required, since a calibration YAML holds one
  calibration. The first message's calibration is used, and a topic whose
  calibration changes mid-bag is reported with a warning naming which was taken.
- An exported YAML has no `camera_name`: it is not a CameraInfo field, so the bag
  cannot supply one. The key is optional, and inventing a name from the topic or
  `frame_id` would be a guess.
- `--topics` accepts several topics at once
  (`--topics /cam1/camera_info /cam2/camera_info`); each is recomputed from its
  own intrinsics, so unlike `replace` they need not share a calibration.
- In bag mode each message's `p` is recomputed from that **same message's** own
  `k` / `d` / `width` / `height`, so a stream whose calibration changes mid-bag is
  handled correctly. A constant stream is the common case and is memoized, so
  OpenCV is consulted once rather than per message.
- Only the named topics are rewritten. Each message's `header` (stamp and
  `frame_id`), `binning_x` / `binning_y`, `roi`, and every other topic are copied
  verbatim — only `p` changes.
- The output bag is always written uncompressed (re-compress later with
  `ros2 bag convert` if needed).
- In-place mode replaces the input atomically: a bag via a sibling temporary bag,
  a YAML via a sibling temporary file.
