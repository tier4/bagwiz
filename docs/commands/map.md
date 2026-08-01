# `bagwiz map`

In-process LiDAR SLAM and map viewing. `map` is a command group with two
actions:

| Subcommand                     | What it does                                                         |
| ------------------------------ | -------------------------------------------------------------------- |
| [`slam`](#bagwiz-map-slam)     | Estimate a trajectory (and optimized map) from a PointCloud2 topic.  |
| [`viewer`](#bagwiz-map-viewer) | Open the browser map viewer for an existing `map.pcd` (no SLAM run). |

## Requirements

**Build.** The `map` command group links the GLIM stack (compiled with
`-DBAGWIZ_WITH_SLAM=ON`) and belongs to the **full** build (`build-full`), which
is the default on **humble**/**jazzy**:

```bash
pixi run -e humble build-full   # or: jazzy
```

The first `build-full` builds the GLIM dependency stack (GTSAM, gtsam_points,
GLIM) into `install/<distro>/glim-deps` — a slow one-time step (tens of minutes) —
then compiles bagwiz with SLAM enabled. Later builds reuse the cached deps and are
fast. The **core** build (`pixi run -e <distro> build-core`) omits the `map`
command group entirely and skips the GLIM stack, so it is much faster but does
not expose `bagwiz map slam`.

**GPU fast path.** For the optional CUDA backend (`--backend cuda`), build the
full CUDA build in a `*-cuda` environment — the CUDA toolkit is pixi-managed
(conda-forge), so no system CUDA install is needed, only an NVIDIA driver + GPU:

```bash
pixi run -e humble-cuda build-full   # or: jazzy-cuda
```

The `humble-cuda` environment is the distro plus the conda CUDA toolkit (12.8:
nvcc, cudart, cusolver). This builds a CUDA GLIM stack (sm_86) into
`install/humble-cuda/glim-deps-cuda` and compiles bagwiz with
`-DBAGWIZ_WITH_SLAM_CUDA=ON` into `install/humble-cuda`. CUDA comes entirely from
`$CONDA_PREFIX` (no `/usr/local/cuda`). Run it with
`pixi run -e humble-cuda run -- map slam …`, or put it on your PATH with
`pixi run -e humble-cuda install`. The CPU environments stay CUDA-free, and
the CPU build and prefix are untouched.

---

## `bagwiz map slam`

Run LiDAR (or LiDAR-IMU) SLAM over a single `sensor_msgs/msg/PointCloud2` topic
and write the results under an output directory. By default the marginalized
frames flow through GLIM's SubMapping → GlobalMapping, so the output is the
globally-optimized 6-DoF trajectory (`traj.tum`) plus an optimized world-frame
point-cloud map (`map.pcd`).

Ghost points left by moving objects can be removed with `--remove-dynamic`
(DUFOMap-style void-region ray casting); it is off by default.

### Usage

```text
bagwiz map slam -i <input> --pcd <pcd_topic> -o <output_root> [OPTIONS]
```

### Examples

```bash
# LiDAR-only SLAM: optimized trajectory + map under out/.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/

# LiDAR-IMU SLAM (extrinsic resolved from the bag's /tf_static).
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --imu /sensing/imu/imu_data

# LiDAR-IMU SLAM with GNSS global constraints to curb drift.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --imu /sensing/imu/imu_data --gnss /sensing/gnss/nav_sat_fix

# Output the trajectory in base_link instead of the sensor frame.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --frame base_link

# Finer input voxel: denser map AND finer SLAM detail (also changes the
# trajectory), overwriting previous outputs.
bagwiz map slam -i drive.mcap --pcd /points -o out/ --input-res 0.1 --overwrite

# Restrict the LiDAR range fed to SLAM to 2–60 m (drops near/far returns).
bagwiz map slam -i drive.mcap --pcd /points -o out/ --min-range 2.0 --max-range 60.0 --overwrite

# Drop the ghost points left by moving objects from the exported map.
bagwiz map slam -i drive.mcap --pcd /points -o out/ --remove-dynamic

# Drop isolated points more aggressively than the default (5 within 0.5 m):
# a map point needs 8 others within 0.6 m to survive.
bagwiz map slam -i drive.mcap --pcd /points -o out/ --remove-outliers --outlier-r 0.6 --outlier-k 8

# Colorize the map from a camera: map.pcd gains an rgb field (CameraInfo
# auto-resolved from the image topic name, extrinsic from the bag's static TF).
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --color /sensing/camera/camera8/image_raw/compressed

# Colorize from two cameras; where their views overlap, the colors are
# blended, gain-aligned to the first listed topic. Listing both topics after
# one --color and repeating the flag are equivalent.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --color /sensing/camera/camera8/image_raw/compressed \
          /sensing/camera/camera0/image_raw/compressed

# Pin one camera's CameraInfo topic explicitly (the other auto-resolves).
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --color /sensing/camera/camera8/image_raw/compressed \
          /sensing/camera/camera0/image_raw/compressed \
  --cam-info /sensing/camera/camera8/image_raw/compressed=/sensing/camera/camera8/camera_info

# Thin the camera stream to keyframes at least 1.5 m (or 10 degrees) apart,
# keeping the sharpest frame of each interval: much faster colorization on
# stop-and-go recordings, and motion-blurred frames are dropped outright.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --color /sensing/camera/camera8/image_raw/compressed \
  --color-min-dist 1.5 --color-keyframe-blur

# Feed three ring cameras as SLAM visual constraints (co-visibility factors
# between submap poses, curbing drift); none of them is colorized, since none
# is also listed under --color.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --cam /sensing/camera/camera6/image_raw/compressed \
        /sensing/camera/camera7/image_raw/compressed \
        /sensing/camera/camera8/image_raw/compressed

# A topic can serve both roles: camera8 is both colorized (--color) and used
# as a visual constraint (--cam) alongside two constraint-only cameras. Its
# CameraInfo and extrinsic are resolved once and shared between the two roles.
bagwiz map slam -i drive.mcap --pcd /sensing/lidar/concatenated/pointcloud -o out/ \
  --color /sensing/camera/camera8/image_raw/compressed \
  --cam /sensing/camera/camera6/image_raw/compressed \
        /sensing/camera/camera7/image_raw/compressed \
        /sensing/camera/camera8/image_raw/compressed

# Build the map, then open it in the browser (blocks until Ctrl-C).
bagwiz map slam -i drive.mcap --pcd /points -o out/ --viewer
```

### Options

| Flag                           | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-i`, `--input <input>`        | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--pcd <pcd_topic>`            | **Required.** `sensor_msgs/msg/PointCloud2` topic to run SLAM on.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `-o`, `--output <output_root>` | **Required.** Output directory. Receives `traj.tum` and `map.pcd`. Created if absent.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `--imu <topic>`                | `sensor_msgs/msg/Imu` topic. Switches odometry to LiDAR-IMU (GLIM's `OdometryEstimationCPU`, or `OdometryEstimationGPU` when the GPU backend is active — `--backend cuda`, or the default `auto` on a CUDA build with a visible device). The LiDAR←IMU extrinsic is resolved from the bag's static TF using the cloud and IMU header `frame_id`s.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `--gnss <topic>`               | `sensor_msgs/msg/NavSatFix` topic. Adds GNSS global constraints during global mapping to pin the world frame to GNSS and curb drift. The antenna lever-arm is resolved from the bag's static TF and removed (a missing TF only warns).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `--color <topic>...`           | Camera image topic(s) (`sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`) to colorize the map from; list several after one flag and/or repeat the flag. The accepted types are exactly those `walk`'s image preview and `generate video` decode (the shared image decoder). After the global optimization, the map points are colorized by projecting them into each camera's images, and `map.pcd` gains an `rgb` field — see [Camera colorization](#camera-colorization---color) for how each point's color is computed and blended. A topic listed more than once is an error.                                                                                                                                                                                                                                                                                   |
| `--cam <topic>...`             | Camera image topic(s) (`sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`) to use as SLAM visual constraints; list several after one flag and/or repeat the flag. The accepted types are exactly those `walk`'s image preview and `generate video` decode (the shared image decoder — the same set `--color` accepts). Each camera's KLT feature tracks become co-visibility constraints between submap poses in the global optimization, curbing drift — see [Camera constraints](#camera-constraints---cam) for how tracks are anchored, gated, and applied. Independent of `--color`: a topic may serve both roles (its `CameraInfo` and extrinsic are then resolved once and shared), but a `--cam`-only topic is never colorized. A topic listed more than once is an error.                                                                                    |
| `--visual-max-features <N>`    | Target live KLT feature-track count per `--cam` camera (must be positive; requires `--cam`). More features give denser co-visibility — stronger, more numerous constraints — at more CPU per tracked frame; fewer trades constraint strength for tracking speed. Default: `200`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `--cam-info <image>=<info>...` | Explicit `sensor_msgs/msg/CameraInfo` topic per camera, as `<image_topic>=<info_topic>` pairs (applies to the union of `--color` and `--cam` topics; requires at least one of them; several after one flag and/or repeated). Cameras without an entry auto-resolve their `CameraInfo` from the image topic name using the standard suffix rules. An entry whose `<image_topic>` is not a listed `--color` or `--cam` topic, a malformed entry, and a duplicate `<image_topic>` are errors.                                                                                                                                                                                                                                                                                                                                                                                     |
| `--color-min-dist <m>`         | Keyframe gate for `--color` colorization (requires `--color`): an image is colorized only when the interpolated pose has moved at least this many meters — or rotated at least 10° — since the image that opened the current gate interval on that topic. Consecutive frames are near-duplicates on vehicle data (a platform stopped at a light contributes hundreds of frames of identical information), so thinning cuts colorize time roughly in proportion to the frames dropped with no visible quality loss; `1`–`2` m is a good starting point. Thinned frames skip the decode too (unless `--color-keyframe-blur` is on, which must decode every candidate to score it). The per-camera colorize summary logs the thinned count. Which images the gate keeps is a discrete decision and does not vary with `--threads` or the backend. Default: `0` (use every image). |
| `--color-keyframe-blur`        | Refine the `--color-min-dist` gate by sharpness (requires `--color-min-dist`): instead of keeping the FIRST image of each gate interval, every candidate inside the interval is decoded and scored with a whole-image sharpness metric (mean Sobel gradient magnitude) and the sharpest is kept, so motion-blurred frames are dropped rather than merely down-weighted (the per-observation sharpness weight only discounts blurry pixels after the frame was already rasterized). Costs a decode per candidate image; the projection/sweep work is still saved. Which image the gate keeps is a discrete decision and does not vary with `--threads` or the backend.                                                                                                                                                                                                          |
| `--no-color-propagate`         | By default, map points no camera observed inherit the color of the nearest observed neighbor within an automatic radius (4× the median point spacing, clamped to [0.05, 5] m), so `map.pcd` comes out fully colored. This flag disables the propagation: unobserved points keep a neutral gray instead. Has no effect without `--color`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--input-res <m>`              | Voxel size in meters (must be positive) used for **both** the GLIM LiDAR input downsample and the exported-map merge — the single map-resolution knob. Smaller = denser map and finer SLAM detail, at the cost of more points and runtime. Unlike a pure export voxel it feeds the optimizer, so changing it also changes the trajectory (not just the map's appearance). The range crop (`--min-range`/`--max-range`) still bounds which returns enter the pipeline. Default: `0.15` (GLIM's stock downsample, so the default trajectory matches GLIM's).                                                                                                                                                                                                                                                                                                                     |
| `--min-range <m>`              | Discard LiDAR returns closer than this many meters before SLAM (must be positive and `< --max-range`). Points dropped here never enter the trajectory or the map. Default: `1.0`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `--max-range <m>`              | Discard LiDAR returns farther than this many meters before SLAM (must be `> --min-range`). Points dropped here never enter the trajectory or the map. Default: `100.0`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `--submap-keyframes <N>`       | Keyframes per GLIM submap before it is finalized (must be positive). Smaller = more, smaller submaps: finer loop-closure granularity and more GNSS-covered submaps (can unblock GNSS priors on short runs), at super-linearly more sub-mapping cost and thinner, weaker submaps. Larger = fewer, larger submaps: a cheaper global graph but coarser correction. Feeds the optimizer, so it also changes the trajectory. Default: `15`.                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `--remove-outliers`            | Remove isolated points from the finished map before colorization and export (radius outlier removal): a map point is dropped when fewer than `--outlier-k` other map points lie within `--outlier-r` meters. Off by default, so the exported map is unchanged without it. Filters `map.pcd` only; the trajectory is untouched. Runs multithreaded under `--threads`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `--outlier-r <m>`              | Neighborhood radius in meters for `--remove-outliers` (must be positive; requires `--remove-outliers`). Tune together with `--input-res`: the exported map is voxel-merged at that resolution, so the radius should span a few voxels (`0.5` ≈ 3 voxels at the stock `0.15` resolution). Default: `0.5`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| `--outlier-k <N>`              | Minimum number of other map points within `--outlier-r` a point needs to survive `--remove-outliers` (must be positive; requires `--remove-outliers`). Higher = more aggressive removal. Default: `5`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `--remove-dynamic`             | Remove ghost points left by moving objects from the finished map (DUFOMap-style void-region ray casting): every scan's rays mark the voxels they traverse as seen-free, and a scan point falling in a voxel that was ever seen free is dropped before the map merge. Off by default (the exported map is unchanged without it). Filters `map.pcd` only; the trajectory is untouched. Runs multithreaded under `--threads`, and the kept/dropped decision is identical for any `--threads` value, because the free-space marking is a monotone OR — a reduction that is exactly commutative and associative.                                                                                                                                                                                                                                                                    |
| `--dynamic-res <m>`            | Voxel size in meters of the free-space grid for `--remove-dynamic` (must be positive; requires `--remove-dynamic`). Independent of `--input-res`: coarser costs less memory and absorbs more pose noise; finer separates ghosts closer to static surfaces. Default: `0.2`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| `--dynamic-ds <m>`             | Sensor back-off in meters for `--remove-dynamic` (must be non-negative; requires `--remove-dynamic`): each ray stops this far short of its hit so range noise cannot mark the hit surface's neighborhood as free. Raise it if ground or thin structure thins out. Default: `0.15`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--dynamic-dp <N>`             | Pose-error guard in voxels for `--remove-dynamic` (`0`–`8`; requires `--remove-dynamic`): a voxel counts as void only when it and every voxel within this Chebyshev radius were seen free. `0` disables the guard; higher is more conservative on static points. Default: `1`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `-j, --threads N`              | Number of CPU threads for GLIM and trajectory endpoint fill (`0` uses the host's hardware concurrency). Values above the host's hardware concurrency are rejected. Default: 8.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `--backend <mode>`             | SLAM backend: `auto`, `cpu`, or `cuda`. `auto` uses the CUDA GPU backend when this binary was built with `-DBAGWIZ_WITH_SLAM_CUDA` (`pixi run -e humble-cuda build-full`) **and** a CUDA device is visible, otherwise it falls back to CPU. `cuda` forces the GPU backend (errors on a non-CUDA build or with no device). `cpu` forces the CPU backend. CUDA backend = GPU LiDAR-IMU odometry with `--imu` (CT odometry without it, as GLIM has no GPU LiDAR-only backend), GPU VGICP registration in sub/global mapping, GPU export-map voxelization, and GPU-accelerated `--color` colorization. **The two backends do not produce bit-identical numbers; the difference is bounded by the tolerances in AGENTS.md "Numerical Reproducibility".** Default: `auto`.                                                                                                           |
| `--frame <frame_id>`           | Output trajectory frame. Defaults to the `header.frame_id` of the PointCloud2 topic, so the trajectory is expressed in the sensor frame. A different value is resolved through the bag's static TF (`*tf_static`); each pose is transformed so it expresses the requested frame in the SLAM world. The map (`map.pcd`) stays in the SLAM world frame regardless of this option.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `--viewer`                     | After writing `map.pcd`, serve it over a loopback HTTP server and open the default browser to a Three.js point-cloud viewer. Blocks until interrupted (`Ctrl-C`). Requires bagwiz to be built with the map viewer; otherwise this flag errors out.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `--no-warmup-fill`             | Disable initialization-window ('start') pose fill. GLIM's odometry emits no pose over its opening window (the LiDAR-IMU init, ~1 s), so `traj.tum` otherwise has no samples there. By default those pre-init scans are buffered and filled in by scan-matching each against the optimized map (so it works in LiDAR-only mode too); with `--imu` the buffered IMU additionally seeds each registration's initial guess and is the fallback if a fit is rejected. Affects `traj.tum`'s opening window only; the map is unaffected. Default: on.                                                                                                                                                                                                                                                                                                                                 |
| `--no-cooldown-fill`           | Disable cooldown-window ('end') pose fill — the symmetric counterpart of `--no-warmup-fill`. The newest scans stay inside the odometry smoother window at end-of-sequence, so `traj.tum` otherwise stops one window short of the last input scan. By default those trailing scans are buffered and filled in by scan-matching each against the optimized map (LiDAR-only included); with `--imu` the buffered IMU additionally seeds each initial guess and is the fallback. Affects `traj.tum`'s closing window only; the map is unaffected. Default: on.                                                                                                                                                                                                                                                                                                                     |
| `--fill-min-inliers`           | Inlier-fraction acceptance gate (must be greater than `0`, up to `1`) for warmup/cooldown pose-fill scan-matching. Higher = stricter (endpoints may stay unfilled); lower = looser (a bad fit can degrade the fill). No effect when both fills are disabled. Default: `0.7`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `-w`, `--overwrite`            | Overwrite the output file(s) if they already exist. Without it, an existing output file stops the run.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `--no-progress`                | Disable the live progress bars. The bars are also auto-suppressed when stderr is not a terminal or `NO_COLOR` is set, so this flag is only needed to silence them on an interactive terminal.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

### SLAM pipeline

In-process, no ROS graph: GLIM modules are fed directly from the decoded bag;
nothing is published or subscribed.

Marginalized frames flow through GLIM's SubMapping → GlobalMapping; `traj.tum`
is the globally-optimized trajectory and `map.pcd` the optimized map.

Clouds with a per-point time field are deskewed by GLIM; clouds without one
are treated as already motion-undistorted.

### LiDAR-only vs LiDAR-IMU

Without `--imu`, odometry runs from the cloud alone. With `--imu`, GLIM's
`OdometryEstimationCPU` (or `OdometryEstimationGPU` whenever the GPU backend is
active — see `--backend`) fuses the IMU, and the LiDAR←IMU extrinsic is
resolved from the bag's static TF.

### CPU vs CUDA backend

The default `auto` backend uses CUDA when the binary was built with
`-DBAGWIZ_WITH_SLAM_CUDA` and a device is visible, otherwise CPU. Force a
specific backend with `--backend cpu` or `--backend cuda`.

The two backends do not produce bit-identical numbers, and neither does one
backend across different `-j, --threads` values — `map.pcd` in particular is
not byte-identical across thread counts, because the parallel voxelizer sums
in a different order. The difference is bounded rather than arbitrary: see
AGENTS.md "Numerical Reproducibility" for the per-quantity tolerances. What
does not vary is the discrete decisions: which images the `--color` keyframe
gate keeps, and which points `--remove-dynamic` drops.

### Output frame (`--frame`)

By default `traj.tum` expresses the PointCloud2 topic's `header.frame_id` (the
sensor frame). Passing `--frame` resolves the sensor → target transform from
the bag's static TF and rewrites each pose so it expresses the requested frame
in the SLAM world. `map.pcd` remains in the SLAM world frame.

### Trajectory endpoint fill

GLIM's odometry emits no pose over its opening window (the LiDAR-IMU init,
~1 s), so `traj.tum` normally starts a beat late. By default those pre-init
scans are buffered and filled in by scan-matching each against the optimized
map: a target is seeded from the boundary-neighboring optimized frames and
each window scan is registered outward from it (GICP), gated on the fit's
inlier fraction. Works in LiDAR-only mode; with `--imu` the buffered IMU
additionally seeds each registration's initial guess and is the fallback if a
fit is rejected. Affects `traj.tum`'s opening window only. Disable with
`--no-warmup-fill`.

The cooldown window is the symmetric counterpart of the start fill. The newest
scans are still inside the odometry smoother window at end-of-sequence and
never reach a finalized submap, so `traj.tum` normally stops one window (a
beat) short of the last input scan. By default those trailing scans are
buffered and filled in by scan-matching each against the optimized map (the
target seeded from the last optimized frames, the chain running forward).
Works in LiDAR-only mode; with `--imu` the buffered IMU additionally seeds
each initial guess and is the fallback. Affects `traj.tum`'s closing window
only. Disable with `--no-cooldown-fill`.

### GNSS constraints (`--gnss`)

Fixes are projected to a local ENU frame, interpolated at each submap's
mid-timestamp, and turned into horizontal translation priors once the 3-D
baseline between the first and last GNSS-associated submap origins exceeds
~10 m (the priors themselves remain horizontal). Each prior is weighted by the
fix's reported position covariance, falling back to a fixed precision when the
covariance is unknown. The antenna lever-arm is resolved from the bag's static
TF and removed.

### Dynamic-point removal (`--remove-dynamic`)

Runs inside the map finalization, after the global optimization and before the
export voxel merge (so colorization and `--remove-outliers` see the cleaned
map). A DUFOMap-style (RA-L 2024) void-region pass over the per-scan frames:
every scan's rays are cast from its optimized sensor pose and mark the voxels
they traverse as seen-free, stopping `--dynamic-ds` short of each hit; a scan
point falling in a voxel that was ever seen-free — space some scan verifiably
saw through — must belong to something that moved, and is dropped. Pose error
is absorbed by `--dynamic-dp` (a voxel counts as void only when its whole
Chebyshev neighborhood was seen free), so static structure is not eaten by a
slightly drifted run. Needs no ground-plane or sensor-pattern assumption.
Filters `map.pcd` only; the trajectory is untouched. The removal decision is
independent of `--threads` (the free-space marking is a pure monotone OR); the
pass is multithreaded and its runtime and dropped/considered counts are
logged. Residual ghost slices directly adjacent to the ground can survive
(their below-surface neighbors are never observed free) — lower
`--dynamic-dp` to `0` to remove more aggressively, or raise `--dynamic-res` /
`--dynamic-ds` if static ground or thin structure thins out instead.

### Camera colorization (`--color`)

Runs after the global optimization: for each camera image whose stamp falls
inside the trajectory's time span, the camera pose is interpolated from the
optimized trajectory and composed with the static cloud ← camera extrinsic,
and the map points are projected onto the raw image through the `CameraInfo`
intrinsics and the camera's distortion model. Intrinsics come from each
camera's `CameraInfo` topic (auto-resolved from the image topic name; see
`--cam-info`), and each camera extrinsic is resolved from the bag's static TF
(cloud ← camera optical frame), erroring if that chain is absent. Images are
assumed raw (unrectified): the `CameraInfo` distortion model is applied during
projection.

With `--color-min-dist`, images are first thinned to keyframes — kept only
when the pose moved (or rotated) enough since the image that opened the
current interval on that topic, and with `--color-keyframe-blur` each gate
interval keeps its sharpest member instead of its first — before any of the
below runs; colorize cost is linear in the image count, so this is the main
throughput knob on slow-moving or stop-and-go recordings. Each point is
splatted into a per-pixel depth buffer over the footprint its local surface
covers on screen — the disc of half its local point spacing, lying in the
plane its normal defines, projected into the image — so occluded points are
rejected and sparse clouds still cover the pixels they pass in front of.
Projecting a disc gives an ellipse, which is what keeps grazing geometry (the
road ahead, facades running alongside) from occluding itself: those surfaces
project to slivers whose samples pile up on screen, and a circular footprint
made every point cull its own neighbors. Each surviving observation is
weighted — close, front-on (surface normals from a local PCA), sharp (image
gradient), and away from the image border all raise the weight — corrected by
a per-image gain estimate that tracks auto-exposure / white-balance drift. The
gain only ever lifts an underexposed frame toward the established reference
(never drags a brighter frame down, and only appearance-stable points vote),
so genuine brightening — driving out of shade — cannot ratchet the stored
colors toward black. Each accepted observation is stored in a bounded
per-point reservoir of up to 16 entries. A map point that sits well behind the
return of the LiDAR scan nearest an image — a vehicle or pedestrian that left
no geometry in the accumulated map — is skipped for that image, so moving
traffic does not stain the colors. The per-camera color then trims the
observations that deviate too far from the 75th-luminance-percentile
observation — shadows are illumination, not surface color, so the lit-mode
cluster wins — and takes a weighted average of the survivors, so moving
objects and occlusion leaks do not stain the map. All color averaging and
every gain ratio run in linear light — the sRGB pixel values are decoded
before the arithmetic and re-encoded on export — so blending observations of
different exposures estimates the surface's radiance instead of the
systematically darker gamma-space mean.

With several `--color` topics, every camera runs its own accumulation over one
shared bag pass and the results are blended: each camera is gain-aligned to
the first listed topic and each point takes the weighted average over the
cameras that observed it. Points no image ever observed inherit the color of
the nearest observed neighbor, so `map.pcd` normally comes out fully colored;
`--no-color-propagate` leaves them a neutral gray instead. The image topics,
`CameraInfo`s, and extrinsics are validated before SLAM starts (fail fast),
but a colorization problem found after SLAM (e.g. no decodable image) only
warns and writes `map.pcd` without colors rather than discarding the run. The
viewer shows the colors through its `rgb` color field, selected by default
when present.

### Camera constraints (`--cam`)

Runs during the SLAM pass itself, not after: each `--cam` camera gets its own
KLT feature tracker fed on one dedicated worker thread per camera, so the
per-frame decode and tracking stay off the bag-read/odometry critical path.
Once every worker has drained (at map finalization), the buffered per-track
observations are turned into co-visibility constraints between submap poses in
the global optimization graph, curbing drift the same way `--gnss`'s priors
do. Each observation is anchored to the submap whose LiDAR keyframe span
covers its stamp by interpolating that submap's intra-submap LiDAR trajectory
(SE(3), slerp rotation / lerp translation) at the image's own capture stamp,
so a `--cam` camera's images need not be synchronized with the LiDAR scans. A
track that never leaves a single submap constrains nothing between poses and
is dropped before it costs a triangulation — so constraint yield grows with
the number of submap boundaries a bag crosses: a lower `--submap-keyframes`
(more, smaller submaps) or a longer/faster-moving recording both raise the
fraction of tracks that survive to a factor. A surviving track's landmark is
triangulated at the submaps' pose estimates; the triangulation itself rejects
a track whose worst reprojection error is too large, which is what catches
moving objects (traffic or pedestrians the submap's LiDAR never captured
there) and drifted tracks. The triangulated point is then separately
cross-checked against those submaps' LiDAR point clouds (a voxel-occupancy
gate: it must fall in an observed voxel in at least one submap it was seen
from), which rejects points with no LiDAR backing at all — sky, reflections,
distant background — that would otherwise tie submaps together through
geometry the LiDAR map never saw. Applying zero constraints (too little
parallax, every track confined to one submap, a reprojection-error rejection,
or the LiDAR gate rejecting every landmark) only warns; the run still
completes with a map unconstrained by the cameras, exactly like an
unproductive `--gnss`.

Intrinsics come from each camera's `CameraInfo` topic (see `--cam-info`), and
each camera extrinsic is resolved from the bag's static TF (cloud ← camera
optical frame), erroring if that chain is absent. Images are assumed raw
(unrectified) and global-shutter, with the header stamp trusted as the instant
of capture — no rolling-shutter or camera↔LiDAR time-offset modeling (a
documented limitation tracked in #376, the same assumption `--color`
colorization makes per #366). `--cam` is independent of `--color`: the two
topic lists may be disjoint, identical, or overlapping; a topic in both roles
has its `CameraInfo`/extrinsic resolved once and shared, but a `--cam`-only
topic is never colorized and a `--color`-only topic never contributes a visual
constraint.

### Outputs

Written under `<output_root>`:

| File       | When    | Format                                                                                                                                                       |
| ---------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `traj.tum` | Always. | TUM trajectory — one `timestamp tx ty tz qx qy qz qw` line per pose.                                                                                         |
| `map.pcd`  | Always. | Binary PCD world-frame point cloud, voxel-downsampled to `--input-res`. With `--color` it additionally carries an `rgb` field (PCL packed-float convention). |

A file at `<output_root>` is an error; an existing directory is accepted and
the directory is created when absent. Each output file is guarded
independently — an existing `traj.tum`/`map.pcd` stops the run unless
`-w`/`--overwrite` is given.

With `--cam`, the run also logs a summary of the visual constraints applied
(no file output — `traj.tum`/`map.pcd` are unaffected by whether any were
found): a per-camera `Visual tracking: fed <N> image(s) from '<topic>' (<M>
failed to decode or track)` line, a `visual constraints: <N> factors from <M>
tracks over <K> submaps (dropped: ...)` breakdown from the constraint-building
pass, and a closing `Applied <N> visual constraint(s) from <M> track(s)
across <K> --cam camera(s)` line (a WARN instead when zero constraints were
applied). See [Camera constraints](#camera-constraints---cam) for what drives
those counts.

With `--viewer`, the written `map.pcd` is served in the browser viewer — see
[`bagwiz map viewer`](#bagwiz-map-viewer). Since `traj.tum` is always written
alongside `map.pcd`, the viewer's Trajectory toggle is available here too (off
by default).

### Progress

On an interactive terminal, a determinate progress bar tracks the
bag-read/feed phase on stderr, and an indeterminate "Finalizing map" spinner
is shown during finalization (global optimization, endpoint fill, and map
export). Both are auto-suppressed when stderr is not a terminal or `NO_COLOR`
is set.

---

## `bagwiz map viewer`

Open the browser map viewer for a **previously written** `map.pcd` — the same
viewer as `map slam --viewer`, but without re-running SLAM. It is the cheap,
repeatable way to revisit a map produced by an earlier `map slam`.

### Usage

```text
bagwiz map viewer -i|--input <map>
```

### Examples

```bash
# Open a `map slam` output directory (bagwiz finds out/map.pcd).
bagwiz map viewer -i out/

# Or point at the map file directly.
bagwiz map viewer -i out/map.pcd
```

### Options

| Flag                  | Description                                                                                                                  |
| --------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <map>` | **Required.** Path to a `map.pcd` file, or a directory containing `map.pcd` (e.g. a `map slam` `<output_root>`). Must exist. |

### Serving

When `<map>` is a directory, bagwiz serves `map.pcd` from inside it, so
`map viewer -i out/` mirrors `map slam … -o out/`; a `.pcd` file path is
served directly.

The map is served over a loopback-only HTTP server and the host's default
browser is opened — the same viewer as `map slam --viewer`. The command
**blocks until you interrupt it** (`Ctrl-C`). Available only when bagwiz is
built with the map viewer.

### Trajectory overlay

When a `traj.tum` file sits next to the served `map.pcd` (as written by
`bagwiz map slam`), the viewer's inspector gains a Trajectory panel with a
"Show trajectory" toggle (off by default). Enabling it draws an X/Y/Z axis
triad at selected poses — oriented by the pose's quaternion and colored to
match the corner orientation gizmo (X red, Y green, Z blue) — with every
recorded pose origin joined by a neutral backbone line (the triads sit on a
subset of those poses). Triads are placed at every 10th actual pose, plus the
last pose so the path's end always carries one, with a fixed 1.5 m axis
length; there are no sliders or other tuning controls for them. The vehicle's
forward axis is X, and a teal ring / blue node mark the first and last pose,
so direction of travel and both ends of the path read at a glance. No
`traj.tum` next to the map means no panel is shown.

### Color fields

The inspector's Field selector offers `x`/`y`/`z` (plus `intensity` when the
PCD carries it) rendered through the selected colormap. A map with an `rgb`
field (written by `map slam --color`) additionally offers `rgb` — the true
camera colors, selected by default — with the colormap and range controls
disabled while active.

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

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
