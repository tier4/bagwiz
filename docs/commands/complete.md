# `bagwiz complete`

Generate a shell completion script for `bagwiz`.

## Usage

```text
bagwiz complete <shell> [--install] [-w|--overwrite]
```

## Positional arguments

| Name    | Description                                              |
| ------- | -------------------------------------------------------- |
| `shell` | Shell completion format to emit (`bash`, `zsh`, `fish`). |

## Options

| Flag                | Description                                                                               |
| ------------------- | ----------------------------------------------------------------------------------------- |
| `--install`         | Write the script to the shell's standard completion directory instead of stdout.          |
| `-w`, `--overwrite` | Overwrite an existing file when used with `--install`. Has no effect without `--install`. |

## Supported shells

| Shell  | Status    |
| ------ | --------- |
| `bash` | Supported |
| `zsh`  | Supported |
| `fish` | Supported |

The completion candidate engine is shell-independent. All shells call the
hidden `bagwiz __complete` protocol to enumerate candidates, so adding more
shells only requires a new script renderer.

## Quick install

`--install` writes the script to the shell's standard location, creating any
missing parent directories:

```bash
bagwiz complete bash --install
bagwiz complete zsh  --install
bagwiz complete fish --install
```

On success it prints the installed path, notes that completion becomes active in
new terminal sessions, and shows the command to enable it in the current shell
right away:

```text
installed: /home/you/.local/share/bash-completion/completions/bagwiz
Completion will be active in new terminal sessions.
To enable it in the current shell now, run:
  source /home/you/.local/share/bash-completion/completions/bagwiz
```

Default targets (XDG variables are honored when set):

| Shell  | Path                                                                  |
| ------ | --------------------------------------------------------------------- |
| `bash` | `${XDG_DATA_HOME:-~/.local/share}/bash-completion/completions/bagwiz` |
| `zsh`  | `~/.zsh/completions/_bagwiz`                                          |
| `fish` | `${XDG_CONFIG_HOME:-~/.config}/fish/completions/bagwiz.fish`          |

Re-running `--install` against an existing file fails unless `-w`/`--overwrite`
is passed.

## Installing bash completion manually

Generate the script into bash's per-user completion directory:

```bash
mkdir -p ~/.local/share/bash-completion/completions
bagwiz complete bash > ~/.local/share/bash-completion/completions/bagwiz
```

Open a new shell for bash-completion to load the file. To use it immediately
in the current shell, source the generated file:

```bash
source ~/.local/share/bash-completion/completions/bagwiz
```

## Installing zsh completion

Generate the script into a directory on `$fpath`. A common per-user choice:

```bash
mkdir -p ~/.zsh/completions
bagwiz complete zsh > ~/.zsh/completions/_bagwiz
```

Add the directory to `$fpath` and initialize the completion system in
`~/.zshrc` (before any other `compinit` call):

```zsh
fpath=(~/.zsh/completions $fpath)
autoload -Uz compinit && compinit
```

Open a new shell to load completions. To use them immediately, source the
file directly:

```bash
source ~/.zsh/completions/_bagwiz
```

If completions fail to refresh after install, remove the cache and re-run
`compinit`:

```bash
rm -f ~/.zcompdump* && compinit
```

## Installing fish completion

Fish autoloads completion files from `~/.config/fish/completions/`:

```bash
mkdir -p ~/.config/fish/completions
bagwiz complete fish > ~/.config/fish/completions/bagwiz.fish
```

Open a new fish shell, or source the file in the current session:

```fish
source ~/.config/fish/completions/bagwiz.fish
```

## Behavior

- Top-level commands and known nested subcommands are completed statically.
- Typing `-` and pressing TAB lists the option flags available at the current
  position. Every command and subcommand responds, including ones that take
  only positional arguments — in that case the listing falls back to the
  `-h` / `--help` flags that CLI11 auto-injects. At the bagwiz top level,
  `-<TAB>` also surfaces `--version`. The covered positions are:
  - `bagwiz -<TAB>` → `--help`, `--version`, `-h`
  - `bagwiz <cmd> -<TAB>` for every command (`cam-info`, `check`, `complete`,
    `convert`, `generate`, `ls`, `map`, `pcd`, `tf`, `topic`, `traj`, `walk`);
    `walk -<TAB>` also surfaces `--cam-info`
  - `bagwiz <cmd> <subcommand> -<TAB>` for every nested subcommand
    (`cam-info replace`, `check broken`, `convert format`, `convert msg`,
    `convert msg geo`, `generate video`, `map slam`, `map viewer`,
    `pcd concat`, `pcd undistort`, `tf static calc`, `tf static cp`,
    `tf tree`, `tf walk`, `topic drop`, `topic keep`, `topic rename`,
    `traj dump`, `traj join`);
    `cam-info replace -<TAB>` surfaces `--frame-id`, `--output`/`-o`,
    `--topics`/`-t`, and `-w`/`--overwrite`; `check broken -<TAB>` surfaces
    `--rm` and `--deep`; `topic drop -<TAB>` / `topic keep -<TAB>` surface
    `--output`/`-o`, `--overwrite`/`-w`, and `--topics`/`-t` (`topic rename
-<TAB>` does not, since its `<src_topic>`/`<dst_topic>` are positional);
    `tf tree -<TAB>` surfaces `--topics`/`-t` (the flag is optional there —
    omitting it merges every TF topic); `tf static calc -<TAB>` also surfaces
    `--json`, and `tf static cp -<TAB>` surfaces `--output`/`-o` and
    `-w`/`--overwrite`.
    `tf static` is itself a command group, so `tf static <TAB>` completes its
    actions (`calc`, `cp`) and `tf static -<TAB>` lists just the help flags.
    `cam-info`, `check`, `generate`, `map`, `pcd`, and `topic` are likewise
    command groups: `cam-info <TAB>` completes `replace`, `check <TAB>`
    completes `broken`, `generate <TAB>` completes `video`, `map <TAB>`
    completes `slam`, `viewer`, `pcd <TAB>` completes `concat`, `undistort`,
    and `topic <TAB>` completes `drop`, `keep`, `rename`
- Selected option values are completed where bagwiz has a closed set, such as
  `--storage <mcap|sqlite3>`.
- Flag values that name a bag topic of a specific type are completed by opening
  `<input>` and offering only topics of that type:
  - `bagwiz generate video <input> ... --cam-info <topic>` — `sensor_msgs/msg/CameraInfo` topics
  - `bagwiz generate video <input> ... --pcd <topic>` — `sensor_msgs/msg/PointCloud2` topics
  - `bagwiz map slam <input> ... --imu <topic>` — `sensor_msgs/msg/Imu` topics
  - `bagwiz walk <input> <topic> --cam-info <topic>` — `sensor_msgs/msg/CameraInfo` topics
  - `bagwiz pcd concat <input> ... --pcd <topic>...` —
    `sensor_msgs/msg/PointCloud2` topics, offered at every value of the variadic
    run
  - `bagwiz pcd concat <input> ... --stamp-offset <topic>=<val>` — the `<topic>`
    half is completed to the same `sensor_msgs/msg/PointCloud2` topics (as
    `<topic>=`) until the value word contains `=`; the `<val>` duration has
    nothing to suggest
  - `bagwiz cam-info replace <input> <calib_yaml> -t/--topics <topic>...` —
    `sensor_msgs/msg/CameraInfo` topics (the only type `cam-info replace`
    rewrites), offered at every value of the variadic run
  - `bagwiz topic drop <input> -t/--topics <selector>...` / `bagwiz topic keep
<input> -t/--topics <selector>...` — every topic in the bag (no type
    filter — these take selectors, which may be globs), offered at every value
    of the variadic run
  - `bagwiz tf tree <input> [-t/--topics <topic>...]` — restricted to
    `tf2_msgs/msg/TFMessage` topics (the only type `tf tree` can render),
    offered at every value of the variadic run. The flag is optional; omitting
    it merges every TF topic in the bag
- Commands that take a `<topic>` positional argument complete it by opening
  `<input>` as a ROS 2 rosbag and listing topics with names that start with
  the current prefix. The currently-covered positions are:
  - `bagwiz walk <input> <topic>`
  - `bagwiz traj dump <input> <topic> <output>` — restricted to the message
    types `traj dump` can process (`tf2_msgs/msg/TFMessage`,
    `geometry_msgs/msg/PoseStamped`,
    `geometry_msgs/msg/PoseWithCovarianceStamped`, `nav_msgs/msg/Odometry`);
    topics of any other type are omitted
  - `bagwiz traj join <input> <traj_file> <topic>`
  - `bagwiz topic rename <input> <src_topic> <dst_topic>` — every topic in the bag
    at the `<src_topic>` slot only; `<dst_topic>` is a new name with nothing to suggest
  - `bagwiz generate video <input> <image_topic> <output>` — restricted to the image
    types `generate video` operates on (`sensor_msgs/msg/Image`,
    `sensor_msgs/msg/CompressedImage`); topics of any other type are omitted
  - `bagwiz map slam <input> <pcd_topic> <output_root>` — restricted to
    `sensor_msgs/msg/PointCloud2` topics (the only type `map slam` ingests);
    topics of any other type are omitted

  Paths beginning with `~/` are expanded against the current user's home
  directory before opening the bag. Topic completion is suppressed when the
  earlier positional slots have been replaced by flags so flag-value
  completion takes over.

- TF frame-id positions complete to the set of distinct `header.frame_id`
  and `child_frame_id` values observed in the input bag's
  `tf2_msgs/msg/TFMessage` topics. Coverage:
  - `bagwiz traj dump <input> ... --from <FRAME>` / `--to <FRAME>` (all TF topics)
  - `bagwiz traj join <input> ... --from <FRAME>` / `--to <FRAME>` (all TF topics)
  - `bagwiz pcd undistort <input> ... --from <FRAME>` / `--to <FRAME>` (all TF topics)
  - `bagwiz tf walk <input> <FRAME> <FRAME>` (the `<from>` and `<to>`
    positional slots; all TF topics, static + dynamic, merged)
  - `bagwiz tf static calc <input> <FRAME> <FRAME>` (the `<from>` and `<to>`
    positional slots; **only** static `*tf_static` topics, since `tf static calc`
    resolves the static tree)

  The bag is opened lazily and only the first ~5000 TF messages are scanned
  so per-keystroke latency stays bounded on large bags. When the bag opens
  cleanly but carries no matching TF data — no TF at all, or, for
  `tf static calc`, no static `*tf_static` topic — no candidates are emitted
  and the shell's default file completion takes over, exactly as when the bag
  path does not exist or the input slot is itself a flag.

- Path completion is delegated to the shell's default file completion when
  bagwiz does not provide command-specific candidates. The bash script uses
  `complete -o default`; the zsh script falls back to `_files`; the fish
  script registers a `-F` rule gated by an `__bagwiz_no_candidates` condition
  that matches the same behavior.
