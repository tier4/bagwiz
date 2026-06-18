# `bagwiz slam`

Run LiDAR SLAM on a single `sensor_msgs/msg/PointCloud2` topic, entirely
in-process. bagwiz reads and decodes the bag and feeds the GLIM odometry,
sub-mapping, and global-mapping modules directly, with no ROS node / pub-sub.

## Usage

```text
bagwiz slam [OPTIONS] <input> <pcd_topic> <output_root>
```

## Positional arguments

| Name          | Description                                                                                                                                              |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input`       | Input ROS 2 rosbag (directory or single-file `*.mcap` / `*.db3`). Must exist.                                                                            |
| `<pcd_topic>` | `sensor_msgs/msg/PointCloud2` topic to run SLAM on.                                                                                                      |
| `output_root` | Output directory. `traj.tum` is always written; `map.ply` is written unless `--without-global-optim` is given. Parent directories are created as needed. |

## Options

| Flag                     | Description                                                                                                                                                                                                                                                                   |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--imu <topic>`          | Optional `sensor_msgs/msg/Imu` topic; switches GLIM to LiDAR-IMU odometry. The LiDAR↔IMU extrinsic is resolved from the bag's static TF (`/tf_static`) using the cloud and IMU header frame_ids.                                                                              |
| `--cam <topic>`          | Optional camera image topic used to colorize `map.ply`. Supported types are `sensor_msgs/msg/Image` (`bgr8`, `rgb8`) and `sensor_msgs/msg/CompressedImage` (JPEG/PNG). The LiDAR↔camera extrinsic is resolved from the bag's static TF.                                       |
| `--cam-info <topic>`     | Optional `sensor_msgs/msg/CameraInfo` topic for `--cam`. When omitted, bagwiz derives it from `<topic>` using the same rules as `bagwiz generate video`: `/image_raw/compressed`, `/image_rect_color`, and `/image_rect_color/compressed` map their prefix to `/camera_info`. |
| `--map-res <m>`          | Exported map voxel size in meters (default: 0.2). Controls only the exported map's density, never the optimization or trajectory.                                                                                                                                             |
| `--without-global-optim` | Skip global mapping and write only the raw odometry trajectory (`traj.tum`); no `map.ply` is produced.                                                                                                                                                                        |
| `-w`, `--overwrite`      | Replace existing output files. Without it, an existing `traj.tum` or `map.ply` stops the run.                                                                                                                                                                                 |

## Camera colorization

When `--cam` is given, each point in the exported map is projected into the
nearest camera image within a 0.1 s window and the underlying pixel color is
sampled. Points whose projection falls outside the image bounds or behind the
camera are colored black. Rectified image topics (e.g. `/image_rect_color`)
use the CameraInfo `P` matrix for projection; raw topics use `K`.

The camera intrinsics are read from the resolved `sensor_msgs/msg/CameraInfo`
topic. The LiDAR↔camera extrinsic is resolved from the bag's static TF using the
camera's `header.frame_id` and the cloud's `header.frame_id`. If the two frames
are identical, an identity extrinsic is used.

## Output

- `traj.tum` — optimized 6-DoF LiDAR trajectory in TUM format, one pose per line:
  `timestamp x y z qx qy qz qw`.
- `map.ply` — binary-little-endian PLY point cloud with `x y z` float32
  properties. An `intensity` property is included when the input scans carry
  intensities; `red green blue` uchar properties are included when `--cam` is
  used.

## Examples

```bash
# Basic LiDAR-only SLAM.
bagwiz slam drive.mcap /sensing/lidar/front/points slam_out/

# LiDAR-IMU SLAM.
bagwiz slam drive.mcap /sensing/lidar/front/points slam_out/ \
  --imu /sensing/imu/imu_raw

# Colorize the map with a rectified camera topic.
bagwiz slam drive.mcap /sensing/lidar/front/points slam_out/ \
  --cam /sensing/camera/front/image_rect_color/compressed

# Colorize with an explicit CameraInfo topic.
bagwiz slam drive.mcap /sensing/lidar/front/points slam_out/ \
  --cam /sensing/camera/front/image_raw/compressed \
  --cam-info /sensing/camera/front/camera_info

# Odometry-only trajectory, no global map.
bagwiz slam drive.mcap /sensing/lidar/front/points slam_out/ \
  --without-global-optim
```

## Exit status

| Code | Meaning                                                                    |
| ---- | -------------------------------------------------------------------------- |
| `0`  | Trajectory (and map, when applicable) written successfully.                |
| `1`  | A runtime or argument error occurred. Check stderr for the specific cause. |
