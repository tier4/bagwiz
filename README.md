# bagwiz

A fast CLI for analyzing, processing, and extracting data from ROS 2 rosbags
entirely offline — without spinning up a ROS graph. rosbag2 inputs are read
through a unified backend that spans three independent dimensions:

- **Storage format** — MCAP (`*.mcap`) or SQLite3 (`*.db3`)
- **Layout** — a rosbag2 directory or a bare single file
- **Compression** — uncompressed, rosbag2 `compression_mode: MESSAGE`, MCAP
  per-chunk compression, or whole-database `compression_mode: FILE` zstd
  envelopes (`*.db3.zstd`)

Any combination of these is accepted transparently.

## Installation

bagwiz is built and run through [pixi](https://pixi.sh) — no system ROS 2
install needed.

1. Install pixi once, then reopen your shell so `pixi` is on `PATH`:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Build bagwiz for a distro. bagwiz supports ROS 2 Humble and Jazzy; each
   has CPU and CUDA environments.

   ```bash
   pixi run -e humble build-core   # basic features
   pixi run -e humble build-full   # includes advanced features such as `map`
   ```

   Use `build-full` only when you need features like `bagwiz map`. The default
   environment is Humble, so `pixi run build-core` is equivalent to
   `pixi run -e humble build-core`.

3. Install a `bagwiz` launcher on your `PATH` so you can run it from anywhere:

   ```bash
   pixi run -e humble install   # installs ~/.local/bin/bagwiz
   ```

   This also installs shell completion for your current shell. It does not
   build; run the build from step 2 first. Use the same `-e <distro>` you built
   with. To switch distros, run `pixi run -e <distro> install` again.

4. Verify the install:

   ```bash
   bagwiz --help
   ```

   If the command is not found, make sure the install directory (default
   `~/.local/bin`) is on your `PATH`.

### Using your own message packages (overlays)

Bags whose topics use message types beyond the standard stack need the matching
ROS 2 message packages available at run time. Build those packages in your own
colcon workspace and source its `install/setup.bash` before running bagwiz:

```bash
source /path/to/my_msgs_ws/install/setup.bash
bagwiz walk my.mcap /topic
```

Sourcing the overlay sets `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH`, so bagwiz
finds the `msg/*.msg` definitions and can dlopen() the introspection typesupport
at runtime without a rebuild. Build overlays against the same distro so their
libraries stay ABI compatible with bagwiz.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                                                                                                                       |
| ---------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag (add `-l` for per-topic counts and average frequencies).                                                                                             |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                                                                                                                       |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between storage backends/layouts, or convert topic message types (NavSatFix → pose).                                                                         |
| [`bagwiz topic`](docs/commands/topic.md)       | Keep (`keep`), drop (`drop`), or rename (`rename`) topics in a ROS 2 rosbag.                                                                                                       |
| [`bagwiz cam-info`](docs/commands/cam-info.md) | Replace (`replace`) one or more CameraInfo topics' calibration with values from a camera_calibration YAML file.                                                                    |
| [`bagwiz generate`](docs/commands/generate.md) | Generate non-rosbag media from a rosbag — e.g. render an image topic to a video (`video`).                                                                                         |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag.                                                                                                  |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect the TF frame tree in a ROS 2 rosbag.                                                                                                                                       |
| [`bagwiz pcd`](docs/commands/pcd.md)           | PointCloud2 topic processing: concatenate multiple LiDAR topics into one (`concat`) via static TF + time sync, or motion-deskew a topic from an external pose topic (`undistort`). |
| [`bagwiz map`](docs/commands/map.md)           | LiDAR map generation and filtering: `map slam`, `map viewer`, `map filter`. Optional build.                                                                                        |
| [`bagwiz check`](docs/commands/check.md)       | Find rosbags whose storage is corrupt / unreadable, and optionally delete them.                                                                                                    |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                                                                                                                        |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Environment variables

bagwiz reads a handful of **optional** environment variables to override
defaults — logging verbosity (`BAGWIZ_LOG_LEVEL`), decoder backend
(`BAGWIZ_DECODER`), color output (`NO_COLOR`), message-package overlays
(`AMENT_PREFIX_PATH`), and the installed launcher's distro
(`BAGWIZ_DEFAULT_DISTRO`), among others. None are required for normal use. See
the full reference in [docs/environment.md](docs/environment.md).

## License

Apache-2.0. See [`LICENSE`](LICENSE).
