# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. The inspection and export subcommands (`ls`, `walk`, `tf`,
`traj`) read rosbag2 inputs — directory layouts and single-file `*.mcap`
/ `*.db3` — through a unified backend. The `convert` subcommand repacks
rosbag2 between MCAP and SQLite3 storage. All of this happens without
spinning up a ROS graph.

## Installation

bagwiz is **tested in CI on ROS 2 Humble and Jazzy**. Other ROS 2 distros
are not exercised in CI but should work — bagwiz builds against whichever
distro you have sourced.

You need ROS 2 installed (Ubuntu packages under `/opt/ros/<distro>` are
typical). Install [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/)
if you do not have it yet (`sudo apt install python3-rosdep`). Run
`sudo rosdep init && rosdep update` once if rosdep has never been set up on
the machine.

From the repository root:

1. Load ROS 2 into your shell (replace `<distro>` with your installed ROS 2 distro, e.g. `humble` or `jazzy`):

   ```bash
   source /opt/ros/<distro>/setup.bash
   ```

   `./setup.sh` and `./build.sh` require a sourced ROS 2 environment. If you
   forget, they stop and print the exact `source .../setup.bash` command for
   each ROS 2 distro installed under `/opt/ros` (and tell you to install ROS 2
   if none is found). There is no distro allow-list — they use whichever distro
   you source.

2. Install build dependencies declared in `package.xml` (run from the repo root):

   ```bash
   ./setup.sh
   ```

   Optional message packages (e.g. for bags that use types beyond the
   standard stack) are not vendored in this repository: install them with
   `apt` or build them in a separate workspace and `source` that install
   space so they appear on `AMENT_PREFIX_PATH` when you run bagwiz.

3. Build:

   ```bash
   ./build.sh
   ```

   The freshly built binary lives at `install/bagwiz/bin/bagwiz`.

4. (Optional) Install the binary onto your `PATH`. `install.sh` copies it to
   `~/.local/bin/bagwiz` (override the destination with `--install-dir <dir>`):

   ```bash
   ./install.sh
   ```

   `install.sh` creates the target directory if needed and warns when it is not
   on your `PATH`. The binary is dynamically linked against ROS, so source ROS 2
   (as in step 1) in any shell where you run `bagwiz`.

Optional flags for `./build.sh` (build type, parallelism, clean rebuild) are
described in `./build.sh --help`; `./install.sh --help` covers the install
destination. `CLI11`, `fmt`, and `rang` are pulled in automatically when you
build; no extra install step for those.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                      |
| ---------------------------------------------- | --------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag with counts and average frequencies.                |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                      |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between MCAP and SQLite3 storage backends.                  |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag. |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect the TF frame tree in a ROS 2 rosbag.                                      |
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
