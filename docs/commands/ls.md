# `bagwiz ls`

List the topics contained in a single ROS 2 rosbag. By default only topic
names and types are printed; pass `-l` / `--long` to add per-topic message
counts and average frequencies. ROS 1 `*.bag` inputs are not supported.

## Usage

```text
bagwiz ls -i <input> [OPTIONS]
```

## Examples

```bash
# List every topic in a directory-layout rosbag2 (names + types only).
bagwiz ls -i path/to/rosbag2_2025_01_01-12_00_00/

# Single-file MCAP.
bagwiz ls -i capture.mcap

# Long listing: add per-topic message counts and average Hz.
bagwiz ls -i capture.mcap -l

# Filter by topic name — column-oriented output makes this trivial.
bagwiz ls -i capture.mcap | grep /sensors/

# Filter by message type.
bagwiz ls -i capture.mcap | grep sensor_msgs/msg/PointCloud2

# Filter a long listing.
bagwiz ls -i capture.mcap -l | grep lidar
```

## Options

| Flag                    | Description                                                                                                                                      |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-i`, `--input <input>` | **Required.** ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. zstd-compressed `*.db3.zstd` inputs are also accepted. |
| `-l`, `--long`          | Add the `COUNT` and `HZ` columns. This requires a statistics pass over the bag (see Performance).                                                |

## Output

A table written to `stdout`, sorted by topic name.

Without `-l`, two columns sourced from the bag's topic list:

```text
TOPIC    TYPE
```

With `-l`, two more columns are appended:

```text
TOPIC    TYPE    COUNT    HZ
```

- `COUNT` is the total number of messages on that topic in the bag.
- `HZ` is the average publish rate, computed as
  `(count - 1) / (last_stamp - first_stamp)` over the entire bag's
  message-time range. Topics with `count <= 1` or a zero-duration bag
  print `0.00`.
- Column widths are computed from the actual data, so long topic /
  type names do not push later columns out of alignment.

## Performance

The default (topics-only) listing never scans message records — it reads just
the topic table — so it is fast and `O(1)` in the number of messages,
regardless of bag size or storage format.

`-l` is different. `COUNT` requires counting messages per topic:

- **MCAP** and **directory-layout** rosbag2 bags answer from the file's
  summary section / `metadata.yaml`, so `-l` stays fast.
- A **single-file `*.db3`** carries its summary in the SQLite `metadata` table.
  Every bagwiz-written `*.db3` fills it in, as does any bag recorded by rosbag2
  iron or newer, so `-l` stays fast there too. Bags recorded by older rosbag2
  (humble) leave that table empty, and `COUNT` falls back to scanning the
  `messages` table, which can be slow on large bags. Prefer the directory
  layout, or run plain `ls` when you only need the topic list.

  The summary is cross-checked against the actual row count before it is
  trusted, so a bag that was appended to behind bagwiz's back falls back to the
  scan rather than reporting stale numbers.

## Exit status

| Code | Meaning                                   |
| ---- | ----------------------------------------- |
| `0`  | Success (including a bag with no topics). |
| `1`  | Failed — check stderr for the cause.      |
