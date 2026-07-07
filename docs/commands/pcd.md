# `bagwiz pcd`

PointCloud2 topic processing.

## `bagwiz pcd concat`

Merge multiple `sensor_msgs/msg/PointCloud2` topics into one new topic. Each
input topic is rigidly transformed into a common `--frame` using the bag's
static TF, and messages are paired against the first `--pcd` topic — the
reference — nearest-in-time within a tolerance. The result is written as a new
rosbag with `-o`, or the input bag is rewritten in place when `-o` is omitted.

This is the offline (rosbag) counterpart of Autoware's point-cloud concatenate
node. It does **not** apply motion compensation — points from different sensor
sweeps are placed by the static extrinsic only. Run `pcd undistort` beforehand
if you need per-cloud deskew; per-point timestamps are preserved so a downstream
undistort still sees correct absolute times.

```text
bagwiz pcd concat <input> <output_topic_name> --pcd <t1> <t2> [<t3> ...] \
    [--frame <frame>] [-o|--output <path>] [--tolerance <val>] \
    [--stamp-offset <topic>=<val>]... [--drop-inputs] [--force] [-w|--overwrite]
```

| Argument / option              | Required | Description                                                                                                                                                                                                                                                                                                   |
| ------------------------------ | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `<input>`                      | ✔        | Input bag (file or directory).                                                                                                                                                                                                                                                                                |
| `<output_topic_name>`          | ✔        | Name of the new concatenated PointCloud2 topic.                                                                                                                                                                                                                                                               |
| `--pcd <t...>`                 | ✔        | PointCloud2 topics to concatenate (2 or more). The first topic is the reference; concatenation order follows this list.                                                                                                                                                                                       |
| `--frame <frame>`              |          | Target frame all clouds are transformed into. Default: `base_link`. Required when the default is not reachable from every `--pcd` frame via the bag's static TF.                                                                                                                                              |
| `-o, --output <path>`          |          | Output bag. When omitted, the input bag is rewritten in place (atomic tmp swap).                                                                                                                                                                                                                              |
| `--tolerance <val>`            |          | Nearest-match tolerance for pairing the other topics to the first `--pcd` topic. Takes an optional unit `ns`/`us`/`ms`/`s` (no unit = ms), e.g. `50ms`. Default: half the first topic's median period.                                                                                                        |
| `--stamp-offset <topic>=<val>` |          | Per-topic offset **added to `header.stamp` for matching only** (the real stamp and per-point times are never rewritten). `<val>` takes an optional unit `ns`/`us`/`ms`/`s` (no unit = ms), signed, e.g. `=-50ms`, `=500ns`. Repeatable. Use it when a sensor triggers early/late relative to the first topic. |
| `--drop-inputs`                |          | Drop the source `--pcd` topics from the output (default: keep them).                                                                                                                                                                                                                                          |
| `--force`                      |          | Proceed even if `<output_topic_name>` already exists in the bag (replaces that topic).                                                                                                                                                                                                                        |
| `-w, --overwrite`              |          | Overwrite an existing `-o/--output` path.                                                                                                                                                                                                                                                                     |

### Behaviour notes

- **Field layout:** all `--pcd` topics must share an identical PointField
  layout (fields and `point_step`); a mismatch is an error.
- **Missing sensor (partial emit):** if a topic has no message within tolerance
  for a given reference message, it is skipped for that output message and the
  remaining topics are still concatenated.
- **Per-point time preservation:** each point's original absolute acquisition
  time is preserved. A header-relative FLOAT time field is re-based in place by
  `(source_header_stamp - output_stamp)`; a header-relative UINT32 time field is
  emitted as FLOAT32 seconds (same 4-byte width) and re-based (its value may go
  negative when an input is earlier than the output stamp); an absolute-encoded
  field is copied verbatim.
- **Output header:** `frame_id` = `--frame`, `stamp` = the reference message's
  real stamp. The merged cloud is unorganized (`height = 1`).
- **Determinism:** CPU-only, no GLIM — deterministic output for a given input.

### Example

Concatenate four Seyond LiDARs (front/rear/left/right) into `base_link`, with
the left/right sensors triggering ~50 ms early:

```bash
bagwiz pcd concat drive.mcap /sensing/lidar/concatenated/points \
  --frame base_link \
  --pcd /sensing/lidar/front/seyond_points /sensing/lidar/rear/seyond_points \
        /sensing/lidar/left/seyond_points  /sensing/lidar/right/seyond_points \
  --stamp-offset /sensing/lidar/left/seyond_points=50ms \
  --stamp-offset /sensing/lidar/right/seyond_points=50ms \
  -o concatenated.mcap
```
