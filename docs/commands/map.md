# `bagwiz map`

In-process LiDAR SLAM and map post-processing. `map` is a command group with
three actions:

| Subcommand                     | What it does                                                                          |
| ------------------------------ | ------------------------------------------------------------------------------------- |
| [`slam`](#bagwiz-map-slam)     | Estimate a trajectory (and optimized map) from a PointCloud2 topic.                   |
| [`viewer`](#bagwiz-map-viewer) | Open the browser map viewer for an existing `map.pcd` (no SLAM run).                  |
| [`filter`](#bagwiz-map-filter) | Post-process an existing map. The first filter is `removert` (dynamic-point removal). |

> **Build.** The `map` command group links the GLIM stack (compiled with
> `-DBAGWIZ_WITH_SLAM=ON`) and belongs to the **full** build (`build-full`), which
> is the default on **humble**/**jazzy**:
>
> ```bash
> pixi run -e humble build-full   # or: jazzy
> ```
>
> The first `build-full` builds the GLIM dependency stack (GTSAM, gtsam_points,
> GLIM) into `install/<distro>/glim-deps` — a slow one-time step (tens of minutes) —
> then compiles bagwiz with SLAM enabled. Later builds reuse the cached deps and are
> fast. The **core** build (`pixi run -e <distro> build-core`) omits the `map`
> command group entirely and skips the GLIM stack, so it is much faster but exposes
> neither `bagwiz map slam` nor `bagwiz map filter`.
>
> **GPU fast path.** For the optional CUDA backend (`--backend cuda`), build the
> full CUDA build in a `*-cuda` environment — the CUDA toolkit is pixi-managed
> (conda-forge), so no system CUDA install is needed, only an NVIDIA driver + GPU:
>
> ```bash
> pixi run -e humble-cuda build-full   # or: jazzy-cuda
> ```
>
> The `humble-cuda` environment is the distro plus the conda CUDA toolkit (12.8:
> nvcc, cudart, cusolver). This builds a CUDA GLIM stack (sm_86) into
> `install/humble-cuda/glim-deps-cuda` and compiles bagwiz with
> `-DBAGWIZ_WITH_SLAM_CUDA=ON` into `install/humble-cuda`. CUDA comes entirely from
> `$CONDA_PREFIX` (no `/usr/local/cuda`). Run it with
> `pixi run -e humble-cuda run -- map slam …`, or put it on your PATH with
> `pixi run -e humble-cuda install`. The CPU environments stay CUDA-free, and
> the CPU build, prefix, and reproducibility guarantee are untouched.

---

## `bagwiz map slam`

Run LiDAR (or LiDAR-IMU) SLAM over a single `sensor_msgs/msg/PointCloud2` topic
and write the results under an output directory. By default the marginalized
frames flow through GLIM's SubMapping → GlobalMapping, so the output is the
globally-optimized 6-DoF trajectory (`traj.tum`) plus an optimized world-frame
point-cloud map (`map.pcd`).

Dynamic-point removal is **not** part of this command; use
[`bagwiz map filter removert`](#bagwiz-map-filter-removert) afterwards.

### Usage

```text
bagwiz map slam [OPTIONS] <input> <pcd_topic> <output_root>
```

### Positional arguments

| Name          | Description                                                             |
| ------------- | ----------------------------------------------------------------------- |
| `input`       | Input ROS 2 rosbag (directory or single-file). Must exist.              |
| `pcd_topic`   | `sensor_msgs/msg/PointCloud2` topic to run SLAM on.                     |
| `output_root` | Output directory. Receives `traj.tum` and `map.pcd`. Created if absent. |

### Options

| Flag                     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--imu <topic>`          | `sensor_msgs/msg/Imu` topic. Switches odometry to LiDAR-IMU (GLIM's `OdometryEstimationCPU`, or `OdometryEstimationGPU` when combined with `--backend cuda`). The LiDAR←IMU extrinsic is resolved from the bag's static TF using the cloud and IMU header `frame_id`s.                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--gnss <topic>`         | `sensor_msgs/msg/NavSatFix` topic. Adds GNSS global constraints during global mapping to pin the world frame to GNSS and curb drift. The antenna lever-arm is resolved from the bag's static TF and removed (a missing TF only warns). Requires global mapping.                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--map-res <m>`          | Exported map voxel size in meters (default `0.2`; must be positive). Controls only the exported map's density, never the optimization or trajectory. The LiDAR preprocessor's ~0.15 m input voxel bounds the effective resolution, so values below ~0.15 m do not produce a denser map.                                                                                                                                                                                                                                                                                                                                                                                       |
| `-t, --threads N`        | Number of CPU threads for GLIM (default: 4). The host's hardware concurrency is the effective maximum.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--backend <mode>`       | SLAM backend: `auto` (default), `cpu`, or `cuda`. `auto` uses the CUDA GPU backend when this binary was built with `-DBAGWIZ_WITH_SLAM_CUDA` (`pixi run -e humble-cuda build-full`) **and** a CUDA device is visible, otherwise it falls back to CPU. `cuda` forces the GPU backend (errors on a non-CUDA build or with no device). `cpu` forces the CPU backend (the reproducibility-guaranteed path). CUDA backend = GPU LiDAR-IMU odometry with `--imu` (CT odometry without it, as GLIM has no GPU LiDAR-only backend), GPU VGICP registration in sub/global mapping, and GPU export-map voxelization. **The CUDA backend is outside the CPU reproducibility guarantee.** |
| `--viewer`               | After writing `map.pcd`, serve it over a loopback HTTP server and open the default browser to a Three.js point-cloud viewer. Blocks until interrupted (`Ctrl-C`).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--upsample <spec>`      | Densify `traj.tum` only (the map is unaffected) to a higher rate. `<spec>` is a positive magnitude with an optional, case-insensitive unit suffix: `hz`/`Hz` or no suffix = absolute frequency in Hz (e.g. `20` or `20hz`); `x`/`X` = a multiple of the trajectory's native rate (e.g. `2x`). Position is interpolated linearly and orientation by SLERP within the original time span.                                                                                                                                                                                                                                                                                       |
| `--no-warmup-recovery`   | Disable initialization-window ('start') pose recovery (default **on**; automatically disabled without `--imu`). GLIM's LiDAR-IMU odometry emits no pose until IMU init converges (~1 s), so `traj.tum` otherwise has no samples over its opening window. By default the pre-init IMU + scan stamps are buffered and, once the first frame's converged state is known, the IMU is integrated backward from it to recover those early poses (re-anchored onto the optimized map). Affects `traj.tum`'s opening window only; the map is unaffected.                                                                                                                              |
| `--no-cooldown-recovery` | Disable cooldown-window ('end') pose recovery (default **on**; automatically disabled without `--imu`) — the symmetric counterpart of `--no-warmup-recovery`. The newest scans stay inside the odometry smoother window at end-of-sequence, so `traj.tum` otherwise stops one window short of the last input scan. By default a trailing ring of IMU + scan stamps is buffered and the IMU is integrated forward from the last estimated frame to recover those trailing poses (re-anchored onto the optimized map). Affects `traj.tum`'s closing window only; the map is unaffected.                                                                                         |
| `-w`, `--overwrite`      | Overwrite the output file(s) if they already exist. Without it, an existing output file stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--no-progress`          | Disable the live progress bar. The bar is also auto-suppressed when stderr is not a terminal or `NO_COLOR` is set, so this flag is only needed to silence it on an interactive terminal.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

### Outputs

Written under `<output_root>`:

| File       | When    | Format                                                                |
| ---------- | ------- | --------------------------------------------------------------------- |
| `traj.tum` | Always. | TUM trajectory — one `timestamp tx ty tz qx qy qz qw` line per pose.  |
| `map.pcd`  | Always. | Binary PCD world-frame point cloud, voxel-downsampled to `--map-res`. |

### Behavior

- **In-process, no ROS graph.** GLIM modules are fed directly from the decoded
  bag; nothing is published or subscribed.
- **Global mapping.** Marginalized frames flow through GLIM's
  SubMapping → GlobalMapping; `traj.tum` is the globally-optimized trajectory and
  `map.pcd` the optimized map.
- **LiDAR-only vs LiDAR-IMU.** Without `--imu`, odometry runs from the cloud
  alone. With `--imu`, GLIM's `OdometryEstimationCPU` (or `OdometryEstimationGPU`
  under `--backend cuda`) fuses the IMU, and the
  LiDAR←IMU extrinsic is resolved from the bag's static TF.
- **Initialization ("start") window (default on; `--no-warmup-recovery` to
  disable).** GLIM's LiDAR-IMU odometry emits no pose until its IMU initialization
  converges (~1 s), so `traj.tum` normally starts a beat late. By default the
  pre-init IMU and scan stamps are buffered; once the first estimated frame's
  converged state (pose, velocity, biases) is known, the IMU is integrated
  backward from it to recover per-scan poses for that window, re-anchored onto the
  optimized map. LiDAR-IMU only (automatically disabled without `--imu`); affects
  `traj.tum`'s opening window only.
- **Cooldown ("end") window (default on; `--no-cooldown-recovery` to disable).** The
  symmetric counterpart of start recovery. The newest scans are still inside the
  odometry smoother window at end-of-sequence and never reach a finalized submap,
  so `traj.tum` normally stops one window (a beat) short of the last input scan.
  By default a trailing ring of IMU and scan stamps is buffered; the IMU is
  integrated forward from the last estimated frame's converged state to recover
  per-scan poses for that window, re-anchored onto the optimized map. Because that
  boundary is a fully-converged mid-run frame, forward recovery is typically more
  accurate than start recovery. LiDAR-IMU only (automatically disabled without
  `--imu`); affects `traj.tum`'s closing window only.
- **GNSS (`--gnss`).** Fixes are projected to a local ENU frame,
  interpolated at each submap's mid-timestamp, and turned into horizontal
  translation priors once the SLAM baseline exceeds ~10 m. Each prior is
  weighted by the fix's reported position covariance, falling back to a fixed
  precision when the covariance is unknown. The antenna lever-arm
  is resolved from the bag's static TF and removed.
- **Dynamic-point removal.** Removed from `map slam`. Run
  `bagwiz map filter removert` afterwards if you need a cleaned map.
- **Deskewing.** Clouds with a per-point time field are deskewed by GLIM; clouds
  without one are treated as already motion-undistorted.
- **Output directory.** A file at `<output_root>` is an error; an existing
  directory is accepted and the directory is created when absent. Each output
  file is guarded independently — an existing `traj.tum`/`map.pcd` stops the run
  unless `-w`/`--overwrite` is given.
- **`--viewer`.** See the viewer section below. Since `traj.tum` is always
  written alongside `map.pcd`, the viewer's Trajectory toggle is available
  here too (off by default).
- **Progress bar.** On an interactive terminal, a progress bar is drawn on stderr.
- **CPU backend.** SLAM runs on GLIM's CPU backend.

### Examples

```bash
# LiDAR-only SLAM: optimized trajectory + map under out/.
bagwiz map slam drive.mcap /sensing/lidar/concatenated/pointcloud out/

# LiDAR-IMU SLAM (extrinsic resolved from the bag's /tf_static).
bagwiz map slam drive.mcap /sensing/lidar/concatenated/pointcloud out/ \
  --imu /sensing/imu/imu_data

# LiDAR-IMU SLAM with GNSS global constraints to curb drift.
bagwiz map slam drive.mcap /sensing/lidar/concatenated/pointcloud out/ \
  --imu /sensing/imu/imu_data --gnss /sensing/gnss/nav_sat_fix

# Denser exported map, overwriting previous outputs.
bagwiz map slam drive.mcap /points out/ --map-res 0.1 --overwrite

# Upsample traj.tum to 100 Hz (the map is unaffected).
bagwiz map slam drive.mcap /points out/ --upsample 100hz

# Build the map, then open it in the browser (blocks until Ctrl-C).
bagwiz map slam drive.mcap /points out/ --viewer

# Clean dynamic points in a separate step.
bagwiz map slam drive.mcap /points out/
bagwiz map filter removert out/ drive.mcap /points out/traj.tum filtered/ --overwrite
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | SLAM completed and the output(s) were written.                                                                                                                                                                                                                                                                                                                                                                                                             |
| `1`  | The input could not be opened; `<pcd_topic>` (or `--imu`/`--gnss`) was absent or had the wrong type; `<output_root>` was a file or could not be created; an output file collided without `-w`/`--overwrite`; in IMU mode the LiDAR←IMU static-TF chain (or a frame) was absent; `--backend cuda` was requested on a non-CUDA build or with no visible CUDA device; no PointCloud2 message decoded; SLAM produced no poses; or a read/write error occurred. |

---

## `bagwiz map viewer`

Open the browser map viewer for a **previously written** `map.pcd` — the same
viewer as `map slam --viewer`, but without re-running SLAM. It is the cheap,
repeatable way to revisit a map produced by an earlier `map slam`.

### Usage

```text
bagwiz map viewer <map>
```

### Positional arguments

| Name  | Description                                                                                                    |
| ----- | -------------------------------------------------------------------------------------------------------------- |
| `map` | Path to a `map.pcd` file, or a directory containing `map.pcd` (e.g. a `map slam` `<output_root>`). Must exist. |

### Behavior

- **File or directory.** When `<map>` is a directory, bagwiz serves `map.pcd`
  from inside it, so `map viewer out/` mirrors `map slam … out/`; a `.pcd` file
  path is served directly.
- **Same viewer as `--viewer`.** Serves the map over a loopback-only HTTP server
  and opens the host's default browser. The command **blocks until you interrupt
  it** (`Ctrl-C`).
- **Trajectory overlay.** When a `traj.tum` file sits next to the served
  `map.pcd` (as written by `bagwiz map slam`), the viewer's inspector gains a
  Trajectory panel with a "Show trajectory" toggle (off by default). Enabling it
  draws each pose as an X/Y/Z axis triad — oriented by the pose's quaternion and
  colored to match the corner orientation gizmo (X red, Y green, Z blue) — with
  the triads' origins joined by a neutral backbone line. Triads sit at actual
  poses spaced evenly along the path; **Axis length** and **Axis spacing**
  sliders tune their size and density. The vehicle's forward axis is X, and a
  teal ring / blue node mark the first and last pose, so direction of travel and
  both ends of the path read at a glance. No `traj.tum` next to the map means no
  panel is shown.
- **Requires the map-viewer build.** Available only when bagwiz is built with the
  map viewer.

### Examples

```bash
# Open a `map slam` output directory (bagwiz finds out/map.pcd).
bagwiz map viewer out/

# Or point at the map file directly.
bagwiz map viewer out/map.pcd
```

### Exit status

| Code | Meaning                                                                                                                          |
| ---- | -------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The viewer served the map and exited cleanly after an interrupt (`Ctrl-C`).                                                      |
| `1`  | `<map>` (or `map.pcd` within it) was not found; no loopback port could be bound; or the binary was built without the map viewer. |

---

## `bagwiz map filter`

Post-process an existing point-cloud map. `filter` is a command group; each
filter type is an action subcommand. The first supported filter is `removert`;
more filters will be added as additional subcommands.

### `bagwiz map filter removert`

Remove dynamic (moving-object) points from an existing `map.pcd` using an
original Removert-style filter. The optimized `traj.tum` from `bagwiz map slam`
is used to reproject each raw scan from `<input>`'s `<pcd_topic>` into the world
frame; the merged map is then filtered against those scan views.

#### Usage

```text
bagwiz map filter removert [OPTIONS] <map> <input> <pcd_topic> <traj> <output>
```

#### Positional arguments

| Name        | Description                                                                          |
| ----------- | ------------------------------------------------------------------------------------ |
| `map`       | Map to filter: a `map.pcd` file or a directory containing `map.pcd`.                 |
| `input`     | Source ROS 2 rosbag (file or directory) that carries the original PointCloud2 scans. |
| `pcd_topic` | `sensor_msgs/msg/PointCloud2` topic whose scans will be reprojected using `<traj>`.  |
| `traj`      | TUM trajectory file produced by `bagwiz map slam` (one pose per scan).               |
| `output`    | Output map path (`.pcd` file) or directory (receives `map.pcd`). Created if absent.  |

#### Options

| Flag                          | Description                                                                                                                                                                                              |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--revert` / `--no-revert`    | Enable the multi-resolution consensus revert pass (default on). Removed points are re-checked at coarser resolutions and recovered if they are not dynamic at any of them. Use `--no-revert` to disable. |
| `--vfov <deg>`                | Total vertical field of view for the Removert range images (default `50.0`; positive).                                                                                                                   |
| `--hfov <deg>`                | Horizontal field of view for the Removert range images (default `360.0`; positive).                                                                                                                      |
| `--remove-resolutions <list>` | Comma-separated magnifier ratios (pixels per degree) for the remove pass (default `2.0`). Processed in order; each resolution operates on the map left by the previous one.                              |
| `--revert-resolutions <list>` | Comma-separated magnifier ratios for the consensus revert pass (default `1.0`).                                                                                                                          |
| `--adaptive-coeff <c>`        | Adaptive discrepancy coefficient (default `0.05`; positive). A map point is dynamic when `abs(scan_range - map_range) > c * scan_range`.                                                                 |
| `--valid-diff-max <m>`        | Upper bound on range difference for a valid pixel comparison (default `200.0`; positive). Pixels with larger differences are treated as no-point pixels.                                                 |
| `-w`, `--overwrite`           | Overwrite the output file(s) if they already exist.                                                                                                                                                      |
| `--no-progress`               | Disable the live progress bar.                                                                                                                                                                           |

#### Behavior

- The filter is purely geometric, deterministic, and runs on the CPU.
- It touches only the output map; the input `map.pcd`, trajectory, and source bag
  are left untouched.
- Scans whose optimized pose cannot be found in `<traj>` are skipped with a
  warning.
- Intensity is preserved when the input map carries per-point intensity.

#### Examples

```bash
# Basic dynamic-point removal.
bagwiz map filter removert out/ drive.mcap /points out/traj.tum filtered/

# More aggressive removal with finer remove resolutions.
bagwiz map filter removert out/ drive.mcap /points out/traj.tum filtered/ \
  --remove-resolutions 3.0,2.5,2.0,1.5 --overwrite
```

#### Exit status

| Code | Meaning                                                                                                                                                                                          |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `0`  | The filtered map was written.                                                                                                                                                                    |
| `1`  | The input map/bag/trajectory could not be opened; the topic was absent or had the wrong type; the map was empty; no scans could be decoded; the output collided; or a read/write error occurred. |

---

## Special thanks

`bagwiz map slam` would not exist without [**GLIM**](https://github.com/koide3/glim)
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
