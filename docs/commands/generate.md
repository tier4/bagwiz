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
from the `<output>` file extension.

### Usage

```text
bagwiz generate video [OPTIONS] <input> <topic> <output>
```

### Positional arguments

| Name     | Description                                                           |
| -------- | --------------------------------------------------------------------- |
| `input`  | Input ROS 2 rosbag (directory or single-file). Must exist.            |
| `topic`  | Image topic to render. Must exist in the bag and be a supported type. |
| `output` | Output video path. Its extension selects the container/codec.         |

### Options

| Flag                | Description                                                                        |
| ------------------- | ---------------------------------------------------------------------------------- |
| `-w`, `--overwrite` | Replace an existing `<output>`. Without it, an existing output path stops the run. |

### Supported topic types

| Message type                      | Status                                           |
| --------------------------------- | ------------------------------------------------ |
| `sensor_msgs/msg/Image`           | Rendered. Pixel encodings: `bgr8`, `rgb8`.       |
| `sensor_msgs/msg/CompressedImage` | Rendered. JPEG / PNG, decoded to BGR internally. |

Other message types stop the run with a clear error. A `CompressedImage` whose
payload is neither JPEG nor PNG (by its leading magic bytes) is likewise
rejected.

### Output format

The container and codec are inferred from the `<output>` extension:

| Extension              | Container            | Codec           |
| ---------------------- | -------------------- | --------------- |
| `.mp4`, `.mkv`, `.mov` | MP4 / Matroska / MOV | H.264 (libx264) |
| `.avi`                 | AVI                  | MJPEG           |

Any other extension stops the run. H.264 requires the FFmpeg build to ship the
`libx264` encoder; if it is missing, use an `.avi` output (MJPEG is always
available) or rebuild FFmpeg with libx264.

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
- Dimensions must be even (the 4:2:0 pixel formats these codecs use require it).

### Examples

```bash
# Render a camera topic to an MP4 (H.264).
bagwiz generate video drive.mcap /sensing/camera/image_raw out.mp4

# Render to MJPEG AVI (no libx264 needed), replacing an existing file.
bagwiz generate video drive_dir/ /sensing/camera/image_raw clip.avi -w
```

## Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                                                                    |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `0`  | The video was written successfully.                                                                                                                                                                                                                                                                                                        |
| `1`  | The input could not be opened; the topic was not found or is an unsupported type; the topic has no messages; the image encoding is unsupported; the output extension is unsupported or its codec is unavailable; `<output>` exists without `-w`/`--overwrite`; a frame changed geometry mid-stream; or a read/encode/write error occurred. |
