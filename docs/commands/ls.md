# `bagwiz ls`

List the topics contained in a single ROS 2 rosbag, with per-topic message
counts and average frequencies. ROS 1 `*.bag` inputs are not supported.

## Usage

```text
bagwiz ls <input>
```

## Positional arguments

| Name    | Description                                                                 |
| ------- | --------------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path: a rosbag2 directory or a single-file `*.mcap` / `*.db3`. |

## Output

A four-column table written to `stdout`, sorted by topic name:

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

## Examples

```bash
# List every topic in a directory-layout rosbag2.
bagwiz ls path/to/rosbag2_2025_01_01-12_00_00/

# Single-file MCAP.
bagwiz ls capture.mcap

# Filter with grep — column-oriented output makes this trivial.
bagwiz ls capture.mcap | grep /sensors/
bagwiz ls capture.mcap | grep sensor_msgs/msg/PointCloud2
bagwiz ls capture.mcap | grep lidar
```

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success (including empty match set). |
| `1`  | Failed to open `<input>`.            |
