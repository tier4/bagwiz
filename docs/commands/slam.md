# `bagwiz slam`

In-process LiDAR SLAM over a rosbag. Unlike `convert` or `topic` (which read a
bag and write another bag), `slam` reads a bag and produces SLAM artifacts — a
trajectory and, by default, an optimized point-cloud map. bagwiz reads and
decodes the bag and feeds [GLIM](https://github.com/koide3/glim)'s modules
directly, with no ROS node or pub/sub. Subcommands:

| Subcommand                      | What it does                                                         |
| ------------------------------- | -------------------------------------------------------------------- |
| [`run`](#bagwiz-slam-run)       | Estimate a trajectory (and optimized map) from a PointCloud2 topic.  |
| [`viewer`](#bagwiz-slam-viewer) | Open the browser map viewer for an existing `map.pcd` (no SLAM run). |

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

| Name          | Description                                                             |
| ------------- | ----------------------------------------------------------------------- |
| `input`       | Input ROS 2 rosbag (directory or single-file). Must exist.              |
| `pcd_topic`   | `sensor_msgs/msg/PointCloud2` topic to run SLAM on.                     |
| `output_root` | Output directory. Receives `traj.tum` and `map.pcd`. Created if absent. |

### Options

| Flag                            | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `--imu <topic>`                 | `sensor_msgs/msg/Imu` topic. Switches odometry to LiDAR-IMU (GLIM's `OdometryEstimationCPU`). The LiDAR←IMU extrinsic is resolved from the bag's static TF using the cloud's and IMU's header `frame_id`s.                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--gnss <topic>`                | `sensor_msgs/msg/NavSatFix` topic. Adds GNSS global constraints during global mapping to pin the world frame to GNSS and curb drift. The antenna lever-arm is resolved from the bag's static TF and removed (a missing TF only warns). Requires global mapping.                                                                                                                                                                                                                                                                                                                                                          |
| `--map-resolution <m>`          | Exported map voxel size in meters (default `0.2`; must be positive). Controls only the exported map's density, never the optimization or trajectory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `--removert`                    | Remove dynamic (moving-object) points from the exported map with an original Removert-style filter: each scan and the merged map are projected into dense range images using the configured FOV, and a map point is dropped when `abs(scan_range - map_range)` exceeds the adaptive coefficient times `scan_range`. Affects `map.pcd` only.                                                                                                                                                                                                                                                                              |
| `--removert-revert`             | Enable the multi-resolution consensus revert pass (default `on`). Removed points are re-checked at coarser resolutions and recovered if they show surface support at any of them. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                           |
| `--removert-vfov <deg>`         | Total vertical field of view for the Removert range images (default `50.0`; positive). For a sensor with +/-25 deg elevation, use `50`. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `--removert-hfov <deg>`         | Horizontal field of view for the Removert range images (default `360.0`; positive). Use `360` for spinning LiDARs. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `--removert-remove-resolutions` | Comma-separated magnifier ratios (pixels per degree) for the remove pass (default `2.0`). Processed in order; each resolution operates on the map left by the previous one. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `--removert-revert-resolutions` | Comma-separated magnifier ratios for the consensus revert pass (default `1.0`). A removed point is recovered if it is not dynamic at any of these resolutions. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `--removert-adaptive-coeff <c>` | Adaptive discrepancy coefficient (default `0.05`; positive). A map point is dynamic when `abs(scan_range - map_range) > c * scan_range`. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `--removert-valid-diff-max <m>` | Upper bound on range difference for a valid pixel comparison (default `200.0`; positive). Pixels with larger differences are treated as no-point pixels. Only used with `--removert`.                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `--viewer`                      | After writing `map.pcd`, serve it over a loopback HTTP server and open the default browser to a Three.js point-cloud viewer. Blocks until interrupted (`Ctrl-C`).                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--upsample-traj <spec>`        | Densify `traj.tum` only (the map is unaffected) to a higher rate. Every original pose is kept verbatim and interpolated samples are inserted between consecutive poses. `<spec>` is a positive magnitude: `x`/`X` = multiple of the native rate (e.g. `2x`); `hz` or no suffix = absolute frequency in Hz (e.g. `20` or `20hz`). Position is interpolated linearly and orientation by SLERP within the original time span (no extrapolation). A target at or below the native rate writes the trajectory unchanged (warned; never down-sampled); segments wider than ~3x the median spacing are not interpolated across. |
| `-w`, `--overwrite`             | Overwrite the output file(s) if they already exist. Without it, an existing output file stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `--no-progress`                 | Disable the live progress bar. The bar is also auto-suppressed when stderr is not a terminal or `NO_COLOR` is set, so this flag is only needed to silence it on an interactive terminal.                                                                                                                                                                                                                                                                                                                                                                                                                                 |

### Outputs

Written under `<output_root>`:

| File       | When    | Format                                                                       |
| ---------- | ------- | ---------------------------------------------------------------------------- |
| `traj.tum` | Always. | TUM trajectory — one `timestamp tx ty tz qx qy qz qw` line per pose.         |
| `map.pcd`  | Always. | Binary PCD world-frame point cloud, voxel-downsampled to `--map-resolution`. |

### Behavior

- **In-process, no ROS graph.** GLIM modules are fed directly from the decoded
  bag; nothing is published or subscribed.
- **Global mapping.** Marginalized frames flow through GLIM's
  SubMapping → GlobalMapping; `traj.tum` is the globally-optimized trajectory and
  `map.pcd` the optimized map.
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
- **Dynamic-point removal (`--removert`).** After the map is rebuilt from
  every optimized scan's points, an offline original Removert-style pass cleans
  moving-object trails (pedestrians, passing vehicles). Each scan's points are
  projected into a dense range image from that scan's sensor origin using the
  configured FOV (`--removert-vfov`, `--removert-hfov`), keeping the closest
  return per pixel. The merged map is projected into the same images and a map
  point is flagged as dynamic when `abs(scan_range - map_range)` is larger than
  `--removert-adaptive-coeff * scan_range` (and below `--removert-valid-diff-max`),
  the signature of a point seen through to a farther surface. The remove pass
  runs through `--removert-remove-resolutions` sequentially, each resolution
  operating on the static map left by the previous one. By default, a consensus
  **revert** pass then re-checks the removed points at
  `--removert-revert-resolutions`; a point is recovered if it is not dynamic at
  any of those resolutions. The filter is purely geometric (no learning, no
  semantic labels), runs on the CPU, and is deterministic; it touches `map.pcd`
  only and leaves `traj.tum` identical. The counts of removed and reverted points
  are reported on completion.
- **Deskewing.** Clouds with a per-point time field are deskewed by GLIM; clouds
  without one are treated as already motion-undistorted (all points
  simultaneous).
- **Output directory.** A file at `<output_root>` is an error; an existing
  directory is accepted and the directory is created when absent. Each output
  file is guarded independently — an existing `traj.tum`/`map.pcd` stops the run
  unless `-w`/`--overwrite` is given, and the map stream is opened before the
  trajectory is committed so a map path that cannot be opened leaves no orphaned
  trajectory.
- **`--viewer` (browser map viewer).** Once `map.pcd` is written, bagwiz starts a
  loopback-only HTTP server (`127.0.0.1`, an OS-assigned port) that serves an
  embedded Three.js viewer page plus the `map.pcd` bytes, and opens the host's
  default browser at it (Linux `xdg-open`, macOS `open`, Windows `start`). The
  command then **blocks until you interrupt it** (`Ctrl-C`), at which point the
  server stops and the run exits. The map is streamed from disk, not buffered, so
  large clouds load incrementally. Points are colored by height. A toolbar toggles
  between a 3D perspective view and a 2D top-down bird's-eye view — orthographic by
  default (no perspective foreshortening, switchable to perspective), so coincident
  points overlay exactly to help check cross-LiDAR alignment from directly above.
  Three.js and the
  UI fonts load from a CDN, so the viewer needs internet access at view time (the
  fonts fall back to the system stack if they cannot load); run without `--viewer`
  to skip it entirely.
- **Progress bar.** On an interactive terminal, a progress bar is drawn on
  stderr: a determinate bar (with ETA) over the bag read while scans are fed to
  GLIM, then an indeterminate spinner during the blocking global optimization
  (`finish()`). It is rendered with the [`indicators`](https://github.com/p-ranav/indicators)
  library and auto-suppressed when stderr is not a TTY, when `NO_COLOR` is set,
  or with `--no-progress`, so piped/CI runs stay clean and the `traj.tum`/`map.pcd`
  summary still prints to stdout.
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

# Denser exported map, overwriting previous outputs.
bagwiz slam run drive.mcap /points out/ --map-resolution 0.1 --overwrite

# Remove moving-object (dynamic) points from the exported map.
bagwiz slam run drive.mcap /points out/ --removert

# More aggressive removal with finer remove resolutions.
bagwiz slam run drive.mcap /points out/ --removert \
  --removert-remove-resolutions 3.0,2.5,2.0,1.5

# Upsample traj.tum to 100 Hz (the map is unaffected).
bagwiz slam run drive.mcap /points out/ --upsample-traj 100hz

# Upsample traj.tum to 4x its native rate.
bagwiz slam run drive.mcap /points out/ --upsample-traj 4x

# Build the map, then open it in the browser (blocks until Ctrl-C).
bagwiz slam run drive.mcap /points out/ --viewer
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                                                                                                 |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | SLAM completed and the output(s) were written.                                                                                                                                                                                                                                                                                                                          |
| `1`  | The input could not be opened; `<pcd_topic>` (or `--imu`/`--gnss`) was absent or had the wrong type; `<output_root>` was a file or could not be created; an output file collided without `-w`/`--overwrite`; in IMU mode the LiDAR←IMU static-TF chain (or a frame) was absent; no PointCloud2 message decoded; SLAM produced no poses; or a read/write error occurred. |

---

## `bagwiz slam viewer`

Open the browser map viewer for a **previously written** `map.pcd` — the same
viewer as `slam run --viewer`, but without re-running SLAM. It is the cheap,
repeatable way to revisit a map produced by an earlier `slam run`.

### Usage

```text
bagwiz slam viewer <map>
```

### Positional arguments

| Name  | Description                                                                                                    |
| ----- | -------------------------------------------------------------------------------------------------------------- |
| `map` | Path to a `map.pcd` file, or a directory containing `map.pcd` (e.g. a `slam run` `<output_root>`). Must exist. |

### Behavior

- **File or directory.** When `<map>` is a directory, bagwiz serves `map.pcd`
  from inside it, so `slam viewer out/` mirrors `slam run … out/`; a `.pcd` file
  path is served directly. A missing file stops the command.
- **Same viewer as `--viewer`.** Serves the map over a loopback-only HTTP server
  (`127.0.0.1`, an OS-assigned port) with the embedded Three.js viewer page and
  opens the host's default browser at it (Linux `xdg-open`, macOS `open`, Windows
  `start`). The command **blocks until you interrupt it** (`Ctrl-C`), then stops
  the server and exits. The map is streamed from disk (not buffered), points are
  colored by height, and Three.js plus the UI fonts load from a CDN, so viewing
  needs internet access (the fonts fall back to the system stack if they cannot
  load). This is identical to the [`--viewer`](#bagwiz-slam-run) flag of
  `slam run`; see that section for the full description.
- **Requires the map-viewer build.** Available only when bagwiz is built with the
  map viewer (on by default with `-DBAGWIZ_WITH_SLAM=ON`); a binary built without
  it reports that `slam viewer` is unavailable.

### Examples

```bash
# Open a `slam run` output directory (bagwiz finds out/map.pcd).
bagwiz slam viewer out/

# Or point at the map file directly.
bagwiz slam viewer out/map.pcd
```

### Exit status

| Code | Meaning                                                                                                                          |
| ---- | -------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The viewer served the map and exited cleanly after an interrupt (`Ctrl-C`).                                                      |
| `1`  | `<map>` (or `map.pcd` within it) was not found; no loopback port could be bound; or the binary was built without the map viewer. |

---

## Special thanks

`bagwiz slam` would not exist without [**GLIM**](https://github.com/koide3/glim)
and its companion library [**gtsam_points**](https://github.com/koide3/gtsam_points),
created by [Kenji Koide (koide3)](https://github.com/koide3). bagwiz does the bag
reading and decoding, but every bit of the actual SLAM — the LiDAR and LiDAR-IMU
odometry, the SubMapping and GlobalMapping factor graphs, the GNSS fusion, and the
deskewing — is GLIM doing the heavy lifting. We feed its modules directly and stand
entirely on that work.

Our deepest gratitude to the author and contributors for building such a capable,
well-engineered, and openly available SLAM framework, and for sharing it with the
community. Please consider citing and starring GLIM if this feature is useful to you:

- GLIM: <https://github.com/koide3/glim>
- gtsam_points: <https://github.com/koide3/gtsam_points>

Thank you. 🙏
