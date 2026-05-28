# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. The inspection and export subcommands (`ls`, `walk`, `tf`,
`traj`) read rosbag2 inputs — directory layouts and single-file `*.mcap`
/ `*.db3` — through a unified backend. The `convert` subcommand repacks
rosbag2 between MCAP and SQLite3 storage. All of this happens without
spinning up a ROS graph.

## Run with Docker

If you would rather not install ROS 2 on the host, run bagwiz from its
container image. The image bundles ROS 2 and the message packages bagwiz
depends on, so the only requirement on the host is Docker. Images are
published to the GitHub Container Registry for both Humble and Jazzy and
for the `linux/amd64` and `linux/arm64` architectures:

- `ghcr.io/tier4/bagwiz:jazzy` (also tagged `latest`)
- `ghcr.io/tier4/bagwiz:humble`

Pull the tag you want:

```bash
docker pull ghcr.io/tier4/bagwiz:latest
```

### Building the image locally

To build the image yourself instead of pulling it, run `./build.sh
--docker` from the repository root. This mode does not require a sourced
ROS environment; the build happens inside the container. It builds for the
host architecture and tags the result `bagwiz:<distro>`:

```bash
./build.sh --docker                  # builds bagwiz:jazzy
./build.sh --docker --distro humble  # builds bagwiz:humble
./build.sh --docker --tag my/bagwiz:dev
```

The release tuning flags `--native` and `--unroll` are forwarded into the
image build, so a local image can use the same optimizations as a native
build (`./build.sh --docker --native --unroll`). Avoid `--native` for
images you share: it bakes the build host's CPU instruction set into the
binary and will crash on other machines. See `./build.sh --help` for the
full option list.

Point the wrapper at a locally built image with `BAGWIZ_IMAGE`, for example
`BAGWIZ_IMAGE=bagwiz:jazzy bagwiz ls recording.mcap`.

### The bagwiz wrapper

`docker/bagwiz` is a small wrapper that runs the image like the native
binary. It mounts the current directory at the same path inside the
container, maps your user so output files are not owned by root, and
allocates a terminal so the interactive walk and the pager work. Install
it on your PATH:

```bash
sudo install -m 0755 docker/bagwiz /usr/local/bin/bagwiz
```

Then call bagwiz against bags under the current directory, exactly as you
would the native binary:

```bash
bagwiz ls recording.mcap
bagwiz tf recording.mcap
```

Set `BAGWIZ_IMAGE` to choose a different tag, for example
`BAGWIZ_IMAGE=ghcr.io/tier4/bagwiz:humble bagwiz ls recording.mcap`.

Bash completion is preinstalled. An interactive shell in the container
(for example `docker run --rm -it ghcr.io/tier4/bagwiz:latest`) has
`bagwiz` tab-completion ready with no setup.

### Adding custom message types

Bags that embed their message definitions (MCAP records carry the schema)
decode out of the box. For bags whose types are not embedded, or to make a
custom package available to every command, provide the message packages
through a standard colcon overlay. Build the packages once inside the
container so their ABI matches the image, then source the overlay and run
bagwiz.

Place your message package sources in a workspace, for example
`~/msgs_ws/src/my_msgs`. Create the package the usual way (for example
with `ros2 pkg create --build-type ament_cmake my_msgs`) so it registers
on `AMENT_PREFIX_PATH` when sourced. Build the overlay and use it in a
single interactive session:

```bash
docker run --rm -it \
  -v ~/msgs_ws:/overlay \
  -v "$PWD:$PWD" -w "$PWD" \
  ghcr.io/tier4/bagwiz:latest bash
# inside the container, build the overlay once, then source and run:
(cd /overlay && colcon build)
source /overlay/install/setup.bash
bagwiz walk recording.mcap /my_topic   # decodes types from ~/msgs_ws
```

The overlay's `install/` directory persists on the host because the
workspace is mounted, so later runs can skip the build. The wrapper can
mount and auto-source a prebuilt overlay for non-interactive use: point
`BAGWIZ_WS` at the workspace and the entrypoint sources its
`install/setup.bash` on every invocation:

```bash
export BAGWIZ_WS=~/msgs_ws
bagwiz walk recording.mcap /my_topic
```

If `colcon build` reports missing dependencies for your message package,
install them inside the container first (start the shell as root with
`--user 0:0` for that step) or bake them into a derived image with a
`FROM ghcr.io/tier4/bagwiz:latest` Dockerfile.

## Installation

You need ROS 2 installed (Ubuntu packages under `/opt/ros/<distro>` are
typical). Install [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/)
if you do not have it yet (`sudo apt install python3-rosdep`). Run
`sudo rosdep init && rosdep update` once if rosdep has never been set up on
the machine.

From the repository root:

1. Load ROS 2 into your shell (replace `humble` with your distro):

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Install build dependencies declared in `package.xml` (run from the repo root):

   ```bash
   ./setup.sh
   ```

   Optional message packages (e.g. for bags that use types beyond the
   standard stack) are not vendored in this repository: install them with
   `apt` or build them in a separate workspace and `source` that install
   space so they appear on `AMENT_PREFIX_PATH` when you run bagwiz.

3. Build — again with ROS sourced (repeat step 1 if you opened a new terminal):

   ```bash
   ./build.sh
   ```

Optional flags for `./build.sh` (build type, parallelism, clean rebuild) are
described in `./build.sh --help`. `CLI11`, `fmt`, and `rang` are pulled in
automatically when you build; no extra install step for those.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                      |
| ---------------------------------------------- | --------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag with counts and average frequencies.                |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                      |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between MCAP and SQLite3 storage backends.                  |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag. |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect TF in a ROS 2 rosbag (frame tree or interactive walk).                    |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                       |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Shell completion

`bagwiz complete <shell>` generates a completion script for `bash`, `zsh`, or
`fish`. Pass `--install` to write it to the shell's standard location
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

Open a new shell, or source the generated file in the current shell. The
completion hook suggests command names, nested subcommands, selected option
values, and topics for `bagwiz walk <input> <topic>` once `<input>` points to
a readable ROS 2 rosbag, including paths written with `~/`. The shared
completion engine is independent of the shell script format — all shells
delegate to the same hidden candidate-generation protocol. See
[`docs/commands/complete.md`](docs/commands/complete.md) for per-shell
install instructions and troubleshooting.

## Contributing

See [`AGENTS.md`](AGENTS.md) for repository-wide contribution
conventions (commit message format, branch naming, pre-commit hooks,
CI etiquette).

## License

Apache-2.0. See [`LICENSE`](LICENSE).
