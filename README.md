# bagwiz

Fast CLI for analyzing, processing, and extracting data from ROS 2
rosbags. The inspection and export subcommands (`ls`, `walk`, `tf`,
`traj`) read rosbag2 inputs — directory layouts and single-file `*.mcap`
/ `*.db3` — through a unified backend. zstd-compressed inputs are
accepted transparently: rosbag2 `compression_mode: MESSAGE` bags, MCAP
chunk compression, and whole-database `compression_mode: FILE` SQLite3
envelopes (`*.db3.zstd`, including the bare single file). The `convert`
subcommand repacks rosbag2 between MCAP and SQLite3 storage. All of this
happens without spinning up a ROS graph.

> **Note on `*.db3.zstd` (FILE-mode) inputs.** Because SQLite must open a
> real file, bagwiz stream-decompresses each `.db3.zstd` shard to a
> temporary `.db3` (in the system temp directory) the first time it is read, and removes
> it when the reader closes — so reading needs free temp space roughly the
> size of the decompressed database. Metadata-only commands (`ls`) answer
> from `metadata.yaml` and never decompress. bagwiz's writers always emit
> uncompressed bags, so in-place rewrites of a FILE-compressed bag are
> refused; pass an explicit `-o` output instead.

## Installation

bagwiz runs through [pixi](https://pixi.sh), so you do not need a system ROS 2
install. pixi provisions the ROS 2 toolchain and message packages from
[RoboStack](https://robostack.github.io/) (one conda channel per distro) and the
C/C++ build toolchain from conda-forge into a project-local environment, and you
pick the ROS 2 distro per command.

bagwiz is tested in CI on ROS 2 Humble, Jazzy, Kilted, and Lyrical; `pixi.toml`
exposes one environment per distro.

1. Install pixi once, then reopen your shell so `pixi` is on `PATH`:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   ```

2. Build bagwiz against a distro. The first build downloads that distro's
   packages and compiles bagwiz; later builds are incremental:

   ```bash
   pixi run -e jazzy build      # or: humble | kilted | lyrical
   ```

   `pixi run build` (no `-e`) targets Jazzy, the default environment. Each distro
   builds into its own `build/<distro>` and `install/<distro>`, so builds for
   several distros can coexist.

3. Run bagwiz, either one command at a time:

   ```bash
   pixi run -e jazzy run -- ls path/to/bag.mcap
   ```

   or from an interactive shell with `bagwiz` on `PATH`:

   ```bash
   pixi shell -e jazzy
   bagwiz ls path/to/bag.mcap
   ```

4. (Optional) Install a `bagwiz` launcher on your `PATH` so you can run it from
   anywhere without typing `pixi run`. The launcher delegates to the pixi
   environment, so the binary still runs with ROS correctly set up:

   ```bash
   pixi run -e jazzy build      # build the distro the launcher targets first
   ./install.sh                 # installs ~/.local/bin/bagwiz (targets Jazzy)
   ```

   `./install.sh --help` covers the install destination (`--install-dir`), the
   target distro (`--distro`), and `--overwrite`. At run time, set
   `BAGWIZ_DISTRO=<distro>` to switch which built distro the launcher uses.

### Using your own message packages (overlays)

Bags whose topics use message types beyond the standard stack need the matching
ROS 2 message packages available at run time. Build those packages in your own
colcon workspace, then point `BAGWIZ_OVERLAY` at it (colon-separated for several
workspaces) before running bagwiz:

```bash
BAGWIZ_OVERLAY=/path/to/my_msgs_ws pixi run -e jazzy run -- walk my.mcap /topic
```

The overlay's `install/setup.bash` is layered on top of the distro, so bagwiz
finds your custom message definitions and typesupport at run time without a
rebuild. Build overlays against the same distro so their libraries stay ABI
compatible with bagwiz.

`pixi task list` and the comments in `pixi.toml` describe the available build
options. `CLI11`, `fmt`, and `rang` are fetched automatically at build time; no
extra install step for those.

## Subcommands

`bagwiz` is a single executable that dispatches to one subcommand per
invocation. Click through for full usage, options, and examples:

| Command                                        | What it does                                                                                               |
| ---------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| [`bagwiz ls`](docs/commands/ls.md)             | List topics in a ROS 2 rosbag with counts and average frequencies.                                         |
| [`bagwiz walk`](docs/commands/walk.md)         | Interactively walk a ROS 2 topic's messages as decoded YAML.                                               |
| [`bagwiz convert`](docs/commands/convert.md)   | Repack a ROS 2 rosbag between storage backends/layouts, or convert topic message types (NavSatFix → pose). |
| [`bagwiz traj`](docs/commands/traj.md)         | Dump a topic's pose trajectory to TUM, or join a trajectory file back into a bag.                          |
| [`bagwiz tf`](docs/commands/tf.md)             | Inspect the TF frame tree in a ROS 2 rosbag.                                                               |
| [`bagwiz check`](docs/commands/check.md)       | Find rosbags whose storage is corrupt / unreadable, and optionally delete them.                            |
| [`bagwiz complete`](docs/commands/complete.md) | Generate a shell completion script (`bash`, `zsh`, `fish`).                                                |

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
