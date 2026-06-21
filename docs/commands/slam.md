# `bagwiz slam`

In-process LiDAR SLAM over a rosbag. Unlike `convert` or `topic` (which read a
bag and write another bag), `slam` reads a bag and produces SLAM artifacts — a
trajectory and, by default, an optimized point-cloud map. bagwiz reads and
decodes the bag and feeds [GLIM](https://github.com/koide3/glim)'s modules
directly, with no ROS node or pub/sub. Subcommands:

| Subcommand                | What it does                                                        |
| ------------------------- | ------------------------------------------------------------------- |
| [`run`](#bagwiz-slam-run) | Estimate a trajectory (and optimized map) from a PointCloud2 topic. |

> **Optional build.** `slam` is not part of a normal `pixi run build`; it links
> the GLIM stack and is compiled only with `-DBAGWIZ_WITH_SLAM=ON`. Build it with:
>
> ```bash
> pixi run -e humble build-slam   # or: jazzy
> ```
>
> The first `build-slam` builds the GLIM dependency stack (GTSAM, gtsam_points,
> GLIM) into `install/<distro>/glim-deps` — a slow one-time step — then compiles
> bagwiz with SLAM enabled. Later builds reuse the cached deps. A binary built
> without SLAM does not expose `bagwiz slam`.

---

## `bagwiz slam run`

Run LiDAR (or LiDAR-IMU) SLAM over a single `sensor_msgs/msg/PointCloud2` topic
and write the results under an output directory. By default the marginalized
frames flow through GLIM's SubMapping → GlobalMapping, so the output is the
globally-optimized 6-DoF trajectory (`traj.tum`) plus an optimized world-frame
point-cloud map (`map.pcd`).

### Usage

```text
bagwiz slam run [OPTIONS] <input> <pcd_topic> <output_root>
```

### Positional arguments

| Name          | Description                                                                                               |
| ------------- | --------------------------------------------------------------------------------------------------------- |
| `input`       | Input ROS 2 rosbag (directory or single-file). Must exist.                                                |
| `pcd_topic`   | `sensor_msgs/msg/PointCloud2` topic to run SLAM on.                                                       |
| `output_root` | Output directory. Receives `traj.tum` and, unless `--without-global-optim`, `map.pcd`. Created if absent. |

### Options

| Flag                     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--imu <topic>`          | `sensor_msgs/msg/Imu` topic. Switches odometry to LiDAR-IMU (GLIM's `OdometryEstimationCPU`). The LiDAR←IMU extrinsic is resolved from the bag's static TF (`…tf_static`) using the cloud's and IMU's header `frame_id`s.                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--gnss <topic>`         | `sensor_msgs/msg/NavSatFix` topic. Adds GNSS global constraints (horizontal translation priors on submap poses) during global mapping to pin the world frame to GNSS and curb drift. Fixes are projected to a local ENU frame internally; the antenna lever-arm is resolved from the bag's static TF (cloud ← NavSatFix `frame_id`) and removed so the prior constrains the sensor origin, not the antenna (a missing TF only warns). Requires global mapping — rejected with `--without-global-optim`.                                                                                                                                                               |
| `--map-resolution <m>`   | Exported map voxel size in meters (default `0.2`; must be positive). Controls only the exported map's density, never the optimization or trajectory. The LiDAR preprocessor's ~0.15 m input voxel bounds the real density.                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| `--without-global-optim` | Skip global mapping and write only the raw odometry trajectory (`traj.tum`). No point-cloud map is produced.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `--vis`                  | After writing `map.pcd`, serve it over a loopback HTTP server and open the default browser to a Three.js point-cloud viewer. Blocks until interrupted (`Ctrl-C`). Cannot be combined with `--without-global-optim`.                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `--upsample-traj <spec>` | Resample **`traj.tum` only** (the map is unaffected) onto a uniform, denser time grid. `<spec>` is a positive magnitude with an optional, case-insensitive suffix: `x`/`X` = a multiple of the trajectory's native rate (e.g. `2x`); `hz`/`HZ`/`Hz` or no suffix = an absolute frequency in Hz (e.g. `20` or `20hz`). Position is interpolated linearly and orientation by SLERP, only within the original time span (no extrapolation). A target at or below the native rate writes the trajectory unchanged (warned; never down-sampled); gaps between poses wider than ~3× the median spacing are left un-interpolated so a sensor dropout is not fabricated over. |
| `-w`, `--overwrite`      | Overwrite the output file(s) if they already exist. Without it, an existing output file stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

### Outputs

Written under `<output_root>`:

| File       | When                                             | Format                                                                       |
| ---------- | ------------------------------------------------ | ---------------------------------------------------------------------------- |
| `traj.tum` | Always.                                          | TUM trajectory — one `timestamp tx ty tz qx qy qz qw` line per pose.         |
| `map.pcd`  | Default (omitted with `--without-global-optim`). | Binary PCD world-frame point cloud, voxel-downsampled to `--map-resolution`. |

### Behavior

- **In-process, no ROS graph.** GLIM modules are fed directly from the decoded
  bag; nothing is published or subscribed.
- **Default (global mapping).** Marginalized frames flow through GLIM's
  SubMapping → GlobalMapping; `traj.tum` is the globally-optimized trajectory and
  `map.pcd` the optimized map.
- **`--without-global-optim`.** Only the raw odometry trajectory is written
  (`traj.tum`); no map is produced.
- **LiDAR-only vs LiDAR-IMU.** Without `--imu`, odometry runs from the cloud
  alone. With `--imu`, GLIM's `OdometryEstimationCPU` fuses the IMU, and the
  LiDAR←IMU extrinsic is resolved from the bag's static TF:
  - When the cloud and IMU share a `frame_id`, the extrinsic is identity.
  - Otherwise it is looked up across the static-TF tree. The run aborts (before
    writing anything) if the bag has no static TF topic, if either frame is
    absent from the static tree, or if no static chain connects them.
- **GNSS (`--gnss`).** Each `NavSatFix` fix is projected to a local ENU frame
  (origin = first valid fix; `STATUS_NO_FIX` samples are dropped), interpolated at
  each submap's mid-timestamp, and — once the SLAM baseline exceeds ~10 m — turned
  into a horizontal translation prior on that submap's pose via a 2-D rigid
  world↔GNSS alignment. This curbs global drift but keeps the output in the SLAM
  world frame (it is not georeferenced). The antenna lever-arm is resolved from the
  bag's static TF (cloud ← `NavSatFix` `frame_id`) and removed, so the prior
  constrains the sensor origin rather than the antenna; when that TF is absent the
  run still succeeds (warned) using the raw antenna position. Each prior is
  weighted by that fix's reported `position_covariance` — rotated into the world
  frame, inflated, floored, and wrapped in a Huber robust kernel — so a meter-level
  SBAS fix is trusted less than a centimeter-level RTK fix; a fix whose covariance
  type is `UNKNOWN` falls back to a fixed per-axis precision. Height stays
  effectively unconstrained (GNSS's weakest axis). With too little motion or no
  temporal overlap, no constraints are added and the run warns but still succeeds.
  Requires global mapping (rejected with `--without-global-optim`).
- **Deskewing.** Clouds with a per-point time field are deskewed by GLIM; clouds
  without one are treated as already motion-undistorted (all points
  simultaneous).
- **Output directory.** A file at `<output_root>` is an error; an existing
  directory is accepted and the directory is created when absent. Each output
  file is guarded independently — an existing `traj.tum`/`map.pcd` stops the run
  unless `-w`/`--overwrite` is given, and the map stream is opened before the
  trajectory is committed so a map path that cannot be opened leaves no orphaned
  trajectory.
- **`--vis` (browser map viewer).** Once `map.pcd` is written, bagwiz starts a
  loopback-only HTTP server (`127.0.0.1`, an OS-assigned port) that serves an
  embedded Three.js viewer page plus the `map.pcd` bytes, and opens the host's
  default browser at it (Linux `xdg-open`, macOS `open`, Windows `start`). The
  command then **blocks until you interrupt it** (`Ctrl-C`), at which point the
  server stops and the run exits. The map is streamed from disk, not buffered, so
  large clouds load incrementally. Points are colored by height. The Three.js
  library itself loads from a CDN, so the viewer needs internet access at view
  time; run without `--vis` to skip it entirely. `--vis` is rejected together
  with `--without-global-optim`, which produces no map.
- **CPU backend.** SLAM runs on GLIM's CPU backend; a GPU backend is a later
  milestone.

### Examples

```bash
# LiDAR-only SLAM: optimized trajectory + map under out/.
bagwiz slam run drive.mcap /sensing/lidar/concatenated/pointcloud out/

# LiDAR-IMU SLAM (extrinsic resolved from the bag's /tf_static).
bagwiz slam run drive.mcap /sensing/lidar/concatenated/pointcloud out/ \
  --imu /sensing/imu/imu_data

# LiDAR-IMU SLAM with GNSS global constraints to curb drift.
bagwiz slam run drive.mcap /sensing/lidar/concatenated/pointcloud out/ \
  --imu /sensing/imu/imu_data --gnss /sensing/gnss/nav_sat_fix

# Trajectory only, no map (skip global optimization).
bagwiz slam run drive_dir/ /points out/ --without-global-optim

# Denser exported map, overwriting previous outputs.
bagwiz slam run drive.mcap /points out/ --map-resolution 0.1 --overwrite

# Upsample traj.tum to 100 Hz (the map is unaffected).
bagwiz slam run drive.mcap /points out/ --upsample-traj 100hz

# Upsample traj.tum to 4x its native rate.
bagwiz slam run drive.mcap /points out/ --upsample-traj 4x

# Build the map, then open it in the browser (blocks until Ctrl-C).
bagwiz slam run drive.mcap /points out/ --vis
```

## Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | SLAM completed and the output(s) were written.                                                                                                                                                                                                                                                                                                                                                                               |
| `1`  | The input could not be opened; `<pcd_topic>` (or `--imu`/`--gnss`) was absent or had the wrong type; `--gnss` was combined with `--without-global-optim`; `<output_root>` was a file or could not be created; an output file collided without `-w`/`--overwrite`; in IMU mode the LiDAR←IMU static-TF chain (or a frame) was absent; no PointCloud2 message decoded; SLAM produced no poses; or a read/write error occurred. |
