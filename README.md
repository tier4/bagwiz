# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. The inspection and export subcommands (`ls`, `walk`, `tf`,
`traj`) read rosbag2 inputs — directory layouts and single-file `*.mcap`
/ `*.db3` — through a unified backend. The `convert` subcommand
additionally bridges to and from ROS 1 `*.bag` for cross-format work.
All of this happens without spinning up a ROS graph.

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
| [`bagwiz convert`](docs/commands/convert.md)   | Convert between ROS 1 and ROS 2, or repack ROS 2 between MCAP / SQLite3.          |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag. |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect TF in a ROS 2 rosbag (frame tree or interactive walk).                    |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                       |

`bagwiz <subcommand> --help` is always available and reflects the same
options documented in the per-command pages.

## Shell completion

`bagwiz complete <shell>` emits an install-ready script for `bash`, `zsh`, or
`fish`. Pick the section for your shell:

```bash
# bash
mkdir -p ~/.local/share/bash-completion/completions
bagwiz complete bash > ~/.local/share/bash-completion/completions/bagwiz

# zsh (ensure ~/.zsh/completions is on $fpath before `compinit`)
mkdir -p ~/.zsh/completions
bagwiz complete zsh > ~/.zsh/completions/_bagwiz

# fish (autoloaded)
mkdir -p ~/.config/fish/completions
bagwiz complete fish > ~/.config/fish/completions/bagwiz.fish
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
