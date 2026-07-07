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

---

## `bagwiz pcd undistort`

Motion-deskew (undistort) one or more `sensor_msgs/msg/PointCloud2` topics
using an external pose topic as the motion source. `pcd undistort` never
estimates motion itself — no SLAM, no scan matching — it only reads a
pose/trajectory topic that is already in the bag (or joined into it
beforehand, e.g. with `traj join`) and moves every point back to one
reference time per scan. Given the same input, it always produces the same
output.

This is the per-cloud counterpart to `pcd concat` above, which does not
compensate for motion at all. Since deskew rewrites only xyz and per-point
time, running `undistort` before `concat` still leaves per-point timestamps
intact for the downstream merge.

```text
bagwiz pcd undistort <input> <pose_topic> --pcd <topic> [--pcd <topic>]... \
    [--from <frame>] [--to <frame>] [-o|--output <path>] [-w|--overwrite] \
    [--no-progress]
```

### Positional arguments

| Name         | Description                                                                                                                                                                                           |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input`      | Input bag (file or directory).                                                                                                                                                                        |
| `pose_topic` | Self-position source topic already in the bag. Type must be one of `tf2_msgs/msg/TFMessage`, `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/PoseWithCovarianceStamped`. |

### Options

| Flag                  | Default      | Description                                                                                                                                                                      |
| --------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--pcd <topic>`       | _(required)_ | PointCloud2 topic to deskew. Repeatable — pass `--pcd` once per topic (e.g. `--pcd /a --pcd /b`) to deskew several topics against the same trajectory. At least one is required. |
| `--from <frame>`      | `map`        | Reference frame the trajectory is resolved in (same convention as `traj dump`).                                                                                                  |
| `--to <frame>`        | `base_link`  | Tracked body frame. The trajectory is obtained as `T_from_to` (e.g. `T_map_base_link`).                                                                                          |
| `-o, --output <path>` | _(unset)_    | Output bag. When omitted, `<input>` is rewritten in place (atomic tmp swap).                                                                                                     |
| `-w, --overwrite`     | `false`      | Replace `-o/--output` if it already exists. Has no effect in in-place mode.                                                                                                      |
| `--no-progress`       | `false`      | Suppress the completion summary log line. There is no live progress bar during the rewrite.                                                                                      |

### Behavior

1. **Resolve the trajectory (Pass 1).** `<input>` must have a `...tf_static`
   topic — it is loaded together with `<pose_topic>` to resolve `--from` →
   `--to`. Only `<pose_topic>` and the bag's static TF feed the trajectory; no
   other topic (e.g. a bag's own dynamic `/tf`) is read automatically. The
   composition mirrors `traj dump`'s: for `TFMessage`, the `--from` → `--to`
   chain is resolved against tf_static plus the edges carried on
   `<pose_topic>` itself, then sampled at every stamp published on that
   chain; for `Odometry` /
   `PoseStamped` / `PoseWithCovarianceStamped`, each message's own pose is
   bridged into `--from`/`--to` via the bag's static TF when its frames don't
   already match (`T_from_to = T_from_header · T_header_body · T_body_to`).
   One difference from `traj dump`: an unresolvable bridge is fatal here,
   where `traj dump` would just skip that one sample. An unresolvable
   `--from` → `--to` overall is likewise fatal — checked before anything is
   written.
2. **Resolve each topic's extrinsic.** For every `--pcd` topic, the sensor
   extrinsic `E = T_to_C` (`C` = that topic's cloud `frame_id`) is resolved
   from the same frame sources as `--from` → `--to`: the bag's `*tf_static`,
   plus `<pose_topic>` itself when it is a `TFMessage` topic (identity when
   `C == --to`). For a statically-mounted sensor — the normal case — this
   extrinsic comes from `tf_static` alone. A missing chain is fatal. Each
   topic's first message must also already carry a per-point time
   field (checked by name: `t`, `time`, `time_stamp`, or `timestamp`); a topic
   without one is fatal, since undistort cannot deskew without per-point time.
3. **Rewrite (Pass 2).** Every message is copied through unchanged except
   `--pcd` topics. For each cloud on a `--pcd` topic:
   - per-point time is normalized to an absolute timestamp per point
     (relative-to-header vs. absolute is auto-detected);
   - each point's xyz is moved from its own timestamp's pose to the pose at
     `t_ref = header.stamp` via
     `p' = E⁻¹·(T_from_to(t_ref)⁻¹·T_from_to(t_i))·E·p`, interpolating the
     trajectory (SLERP + lerp) and clamping to the nearest endpoint pose for
     points outside its time span;
   - the per-point time field is rewritten to the `t_ref`-equivalent value
     (`0` for a relative field, `header.stamp` for an absolute one), so a
     later `undistort` or `concat` run can't double-deskew the cloud;
   - non-finite (NaN/Inf) points are left byte-for-byte unchanged;
   - every other field, the point count, `point_step`/`row_step` (organized
     clouds included), and `frame_id` are unchanged — points never leave
     their own cloud's frame;
   - `FLOAT32` and `FLOAT64` xyz are both supported (computed internally as
     `double`). A malformed cloud — big-endian, missing/misshapen x/y/z, or an
     inconsistent point/row layout — aborts the whole run rather than being
     skipped; a cloud that merely fails to parse as `PointCloud2` is instead
     copied through unchanged with a warning.
   - The trajectory is built once and shared by every `--pcd` topic; only the
     extrinsic `E` changes per topic, so sensors with different mount points
     can be deskewed together in one run.
4. **Output.** `-o` writes a new bag inheriting `<input>`'s storage format;
   omitting it rewrites `<input>` in place through a tmp file and an atomic
   swap, so a mid-pass failure leaves the original bag untouched.
5. **Determinism.** No SLAM and no threading/backend choices are involved —
   the same input always produces the same output.

To deskew against SLAM-derived poses rather than an existing localization
topic, compose it from `map slam` and `traj join`: generate a trajectory,
embed it into the bag as a topic, then point `pcd undistort` at that topic
(see the third example below).

### Examples

```bash
# Deskew one topic in place, using an existing localization pose (Odometry).
bagwiz pcd undistort drive.mcap /localization/kinematic_state \
  --pcd /sensing/lidar/top/pointcloud

# Deskew multiple topics against the same pose, into a new bag.
bagwiz pcd undistort drive.mcap /localization/kinematic_state \
  --pcd /sensing/lidar/top/pointcloud --pcd /sensing/lidar/left/pointcloud \
  -o undistorted.mcap

# Composition workflow: derive a trajectory with SLAM, embed it as a topic,
# then deskew against it.
bagwiz map slam drive.mcap /points out/                 # -> out/traj.tum
bagwiz traj join drive.mcap out/traj.tum /slam/tf --from map --to base_link
bagwiz pcd undistort drive.mcap /slam/tf --pcd /points -o undistorted.mcap
```

### Errors

| Situation                                                                                                                 | Result                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| No `--pcd` given                                                                                                          | Error.                                                                                                    |
| `pose_topic` absent from `<input>`, or not one of the four supported types                                                | Error.                                                                                                    |
| A `--pcd` topic absent from `<input>`, or not `PointCloud2`                                                               | Error.                                                                                                    |
| `<input>` has no `...tf_static` topic                                                                                     | Fatal — needed to resolve `--from` → `--to` and every `--pcd` topic's extrinsic.                          |
| `--from` → `--to` cannot be resolved from `pose_topic` + the bag's static TF                                              | Fatal.                                                                                                    |
| A `--pcd` topic's first message has no per-point time field                                                               | Fatal.                                                                                                    |
| `--to` → a `--pcd` topic's cloud frame is not reachable via `*tf_static` + `<pose_topic>`                                 | Fatal.                                                                                                    |
| A cloud reaching the rewrite step is malformed (big-endian, missing/misshapen x/y/z, or an inconsistent point/row layout) | Aborts the run (a cloud that merely fails to _parse_ is copied through unchanged with a warning instead). |
| `-o` output path already exists without `-w`/`--overwrite`                                                                | Error.                                                                                                    |

### Exit status

| Code | Meaning                                                                                                 |
| ---- | ------------------------------------------------------------------------------------------------------- |
| `0`  | The rewrite completed: every `--pcd` topic validated in Pass 1, and Pass 2 finished.                    |
| `1`  | Any of the error/fatal conditions above, a writer/I/O failure, or a cloud that failed the rewrite step. |
