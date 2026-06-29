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

bagwiz provides first-class support for the long-term-support (LTS) ROS 2
distributions — Humble, Jazzy, and Lyrical; `pixi.toml` exposes one environment
per distro.

1. Install pixi once, then reopen your shell so `pixi` is on `PATH`:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Build bagwiz against a distro. The first build downloads that distro's
   packages and compiles bagwiz; later builds are incremental:

   ```bash
   pixi run -e humble build     # or: jazzy | lyrical
   ```

   `pixi run build` (no `-e`) targets Humble, the default environment. Each distro
   builds into its own `build/<distro>` and `install/<distro>`, so builds for
   several distros can coexist.

3. Install a `bagwiz` launcher on your `PATH` — plus tab completion — so you can
   run it from anywhere without typing `pixi run`:

   ```bash
   pixi run -e humble install   # builds, then installs ~/.local/bin/bagwiz + completion
   ```

   This installs the launcher and shell completion for your current shell in one
   step, always overwriting any existing copies. Use the same `-e <distro>` you
   built with (a bare `pixi run install` targets Humble, the default
   environment). Set `BAGWIZ_INSTALL_DIR` to change the destination, or
   `BAGWIZ_DISTRO` to target a different built distro. At run time,
   `BAGWIZ_DISTRO=<distro>` still switches which built distro the launcher uses.

4. Verify the install — `bagwiz` should now be on your `PATH`:

   ```bash
   bagwiz --help
   ```

   If the command is not found, make sure the install directory (default
   `~/.local/bin`) is on your `PATH`.

### Using your own message packages (overlays)

Bags whose topics use message types beyond the standard stack need the matching
ROS 2 message packages available at run time. Build those packages in your own
colcon workspace, then point `BAGWIZ_OVERLAY` at it (colon-separated for several
workspaces) before running bagwiz:

```bash
BAGWIZ_OVERLAY=/path/to/my_msgs_ws pixi run -e humble run -- walk my.mcap /topic
```

The overlay's `install/setup.bash` is layered on top of the distro, so bagwiz
finds your custom message definitions and typesupport at run time without a
rebuild. Build overlays against the same distro so their libraries stay ABI
compatible with bagwiz.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                                                    |
| ---------------------------------------------- | --------------------------------------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag with counts and average frequencies.                                              |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                                                    |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between storage backends/layouts, or convert topic message types (NavSatFix → pose).      |
| [`bagwiz topic`](docs/commands/topic.md)       | Keep (`keep`), drop (`drop`), or rename (`rename`) topics in a ROS 2 rosbag.                                    |
| [`bagwiz cam-info`](docs/commands/cam-info.md) | Replace (`replace`) one or more CameraInfo topics' calibration with values from a camera_calibration YAML file. |
| [`bagwiz generate`](docs/commands/generate.md) | Generate non-rosbag media from a rosbag — e.g. render an image topic to a video (`video`).                      |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag.                               |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect the TF frame tree in a ROS 2 rosbag.                                                                    |
| [`bagwiz map`](docs/commands/map.md)           | LiDAR map generation and filtering: `map slam`, `map viewer`, `map filter`. Optional build.                     |
| [`bagwiz check`](docs/commands/check.md)       | Find rosbags whose storage is corrupt / unreadable, and optionally delete them.                                 |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                                                     |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Shell completion

`pixi run install` already installs completion for your current shell (see step 3
of [Installation](#installation)). To install it manually, or for a different
shell, `bagwiz complete <shell>` generates a completion script for `bash`, `zsh`,
or `fish`. Pass `--install` to write it to the shell's standard location
automatically (parent directories are created):

```bash
bagwiz complete bash --install
bagwiz complete zsh  --install   # remember to put ~/.zsh/completions on $fpath
bagwiz complete fish --install
```

Without `--install` the script is printed to stdout, so you can pipe it
anywhere:

```bash
bagwiz complete bash > ~/.local/share/bash-completion/completions/bagwiz
```

Open a new shell, or source the generated file in the current shell:

```bash
source ~/.local/share/bash-completion/completions/bagwiz
```

## License

Apache-2.0. See [`LICENSE`](LICENSE).
