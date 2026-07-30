# `bagwiz complete`

Generate a shell completion script for `bagwiz`.

## Usage

```text
bagwiz complete --shell <shell> [--install] [-w|--overwrite]
```

## Options

| Flag                | Description                                                                               |
| ------------------- | ----------------------------------------------------------------------------------------- |
| `--shell <shell>`   | Shell completion format to emit (`bash`, `zsh`, `fish`).                                  |
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
bagwiz complete --shell bash --install
bagwiz complete --shell zsh  --install
bagwiz complete --shell fish --install
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
bagwiz complete --shell bash > ~/.local/share/bash-completion/completions/bagwiz
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
bagwiz complete --shell zsh > ~/.zsh/completions/_bagwiz
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
bagwiz complete --shell fish > ~/.config/fish/completions/bagwiz.fish
```

Open a new fish shell, or source the file in the current session:

```fish
source ~/.config/fish/completions/bagwiz.fish
```

## Behavior

- Top-level commands and known nested subcommands are completed statically.
- Typing `-` and pressing TAB lists the option flags available at the current
  position. Every command and subcommand responds, including ones that take
  only flags — in that case the listing falls back to the
  `-h` / `--help` flags that CLI11 auto-injects. At the bagwiz top level,
  `-<TAB>` also surfaces `--version`. The covered positions are:
  - `bagwiz -<TAB>` → `--help`, `--version`, `-h`
  - `bagwiz <cmd> -<TAB>` for every command (`cam-info`, `check`, `complete`,
    `convert`, `generate`, `ls`, `map`, `pcd`, `tf`, `topic`, `traj`, `trim`, `walk`);
    `walk -<TAB>` also surfaces `--cam-info`, `ls -<TAB>` surfaces `-l`/`--long`,
    and `trim -<TAB>` surfaces `--start`, `--end`, `--duration`, `--both`,
    `--align`, `--stamp`, `--output`/`-o`, `--overwrite`/`-w`
  - `bagwiz <cmd> <subcommand> -<TAB>` for every nested subcommand
    (`cam-info replace`, `cam-info recompute-p`, `cam-info dump`, `check broken`,
    `convert format`, `generate video`, `map slam`, `map viewer`,
    `pcd concat`, `pcd undistort`, `tf static calc`, `tf static cp`,
    `tf tree`, `topic drop`, `topic keep`, `topic rename`,
    `traj dump`, `traj join`);
    `cam-info replace -<TAB>` surfaces `--frame-id`, `--output`/`-o`,
    `--topics`/`-t`, and `-w`/`--overwrite`; `check broken -<TAB>` surfaces
    `--rm` and `--deep`; `topic drop -<TAB>` / `topic keep -<TAB>` surface
    `--output`/`-o`, `--overwrite`/`-w`, and `--topics`/`-t` (`topic rename
-<TAB>` surfaces `--output`/`-o` and `--overwrite`/`-w` but not `--topics`/`-t`,
    since its topics are now `--src` and `--dst`, long-form only);
    `tf tree -<TAB>` surfaces `--input`/`-i` and `--topics`/`-t` (the flag is optional there —
    omitting it merges every TF topic); `tf static calc -<TAB>` also surfaces
    `--json`, `--of`, `--ref`, and `tf static cp -<TAB>` surfaces `--src`, `--dst`
    (long-form only — neither has a short form), `--output`/`-o` and `-w`/`--overwrite`.
    `tf static` is itself a command group, so `tf static <TAB>` completes its
    actions (`calc`, `cp`) and `tf static -<TAB>` lists just the help flags.
    `cam-info`, `check`, `generate`, `map`, `pcd`, and `topic` are likewise
    command groups: `cam-info <TAB>` completes `replace`, `recompute-p`, `dump`, `check <TAB>`
    completes `broken`, `generate <TAB>` completes `video`, `map <TAB>`
    completes `slam`, `viewer`, `pcd <TAB>` completes `concat`, `undistort`,
    and `topic <TAB>` completes `drop`, `keep`, `rename`
- Selected option values are completed where bagwiz has a closed set, such as
  `--storage <mcap|sqlite3>` and `--stamp <header|recv>`.
- Flag values that name a bag topic of a specific type are completed by opening
  `<input>` and offering only topics of that type:
  - `bagwiz generate video -i <input> ... --cam-info <topic>` — `sensor_msgs/msg/CameraInfo` topics
  - `bagwiz generate video -i <input> ... --pcd <topic>` — `sensor_msgs/msg/PointCloud2` topics
  - `bagwiz map slam -i <input> ... --imu <topic>` — `sensor_msgs/msg/Imu` topics
  - `bagwiz map slam -i <input> ... --color <topic>...` — `sensor_msgs/msg/Image` or
    `sensor_msgs/msg/CompressedImage` topics, offered at every value of the
    variadic run
  - `bagwiz map slam -i <input> ... --cam <topic>...` — `sensor_msgs/msg/Image` or
    `sensor_msgs/msg/CompressedImage` topics, offered at every value of the
    variadic run
  - `bagwiz map slam -i <input> ... --cam-info <image>=<info>...` — the
    `<image_topic>` half is completed to the same image topics (as
    `<topic>=`) at every value of the run; once the cursor moves past `=`,
    the `<info_topic>` half has nothing to suggest
  - `bagwiz walk -i <input> -t <topic> --cam-info <topic>` — `sensor_msgs/msg/CameraInfo` topics
  - `bagwiz pcd concat -i <input> ... --pcd <topic>...` —
    `sensor_msgs/msg/PointCloud2` topics, offered at every value of the variadic
    run
  - `bagwiz pcd concat -i <input> ... --stamp-offset <topic>=<val>...` — the
    `<topic>` half is completed to the same `sensor_msgs/msg/PointCloud2`
    topics (as `<topic>=`) at every value of the run; once the cursor moves
    past `=`, the `<val>` duration has nothing to suggest
  - `bagwiz pcd undistort -i <input> ... --pcd <topic>...` —
    `sensor_msgs/msg/PointCloud2` topics, offered at every value of the variadic
    run
  - `bagwiz cam-info replace -i <input> --yaml <yaml> -t/--topics <topic>...` —
    `sensor_msgs/msg/CameraInfo` topics (the only type `cam-info replace`
    rewrites), offered at every value of the variadic run
  - `bagwiz cam-info recompute-p -i <input> ... -t/--topics <topic>...` —
    `sensor_msgs/msg/CameraInfo` topics, offered at every value of the variadic
    run
  - `bagwiz cam-info dump -i <input> -t <topic>` — `sensor_msgs/msg/CameraInfo` topics
  - `bagwiz topic drop -i <input> -t/--topics <selector>...` / `bagwiz topic keep
-i <input> -t/--topics <selector>...` — every topic in the bag (no type
    filter — these take selectors, which may be globs), offered at every value
    of the variadic run
  - `bagwiz tf tree -i <input> [-t/--topics <topic>...]` — restricted to
    `tf2_msgs/msg/TFMessage` topics (the only type `tf tree` can render),
    offered at every value of the variadic run. The flag is optional; omitting
    it merges every TF topic in the bag
  - `bagwiz trim -i <input> --align <topic>...` — every topic in the bag (no type
    filter — these take selectors, which may be globs), offered at the first
    value of the run only
- Commands that take a `<topic>` flag value complete it by opening
  `<input>` as a ROS 2 rosbag and listing topics with names that start with
  the current prefix. The currently-covered positions are:
  - `bagwiz walk -i <input> -t <topic>`
  - `bagwiz traj dump -i <input> -t <topic> -o <output>` — restricted to the message
    types `traj dump` can process (`tf2_msgs/msg/TFMessage`,
    `geometry_msgs/msg/PoseStamped`,
    `geometry_msgs/msg/PoseWithCovarianceStamped`, `nav_msgs/msg/Odometry`);
    topics of any other type are omitted
  - `bagwiz traj join -i <input> --traj <traj_file> -t <topic>`
  - `bagwiz topic rename -i <input> --src <src_topic> --dst <dst_topic>` — every topic in the bag
    at the `<src_topic>` slot only; `<dst_topic>` is a new name with nothing to suggest
  - `bagwiz generate video -i <input> -t <image_topic> -o <output>` — restricted to the image
    types `generate video` operates on (`sensor_msgs/msg/Image`,
    `sensor_msgs/msg/CompressedImage`); topics of any other type are omitted
  - `bagwiz map slam -i <input> --pcd <pcd_topic> -o <output_root>` — restricted to
    `sensor_msgs/msg/PointCloud2` topics (the only type `map slam` ingests);
    topics of any other type are omitted

  Paths beginning with `~/` are expanded against the current user's home
  directory before opening the bag.

- TF frame-id positions complete to the set of distinct `header.frame_id`
  and `child_frame_id` values observed in the input bag's
  `tf2_msgs/msg/TFMessage` topics. Coverage:
  - `bagwiz traj dump -i <input> ... --ref <FRAME>` / `--of <FRAME>` (all TF topics)
  - `bagwiz traj join -i <input> ... --ref <FRAME>` / `--of <FRAME>` (all TF topics)
  - `bagwiz pcd undistort -i <input> ... --ref <FRAME>` / `--of <FRAME>` (all TF topics)
  - `bagwiz tf static calc -i <input> ... --ref <FRAME>` / `--of <FRAME>`
    (**only** static `*tf_static` topics, since `tf static calc` resolves the
    static tree)

  The bag is opened lazily and only the first ~5000 TF messages are scanned
  so per-keystroke latency stays bounded on large bags. When the bag opens
  cleanly but carries no matching TF data — no TF at all, or, for
  `tf static calc`, no static `*tf_static` topic — no candidates are emitted
  and the shell's default file completion takes over, exactly as when the bag
  path does not exist or the input slot is itself a flag. FILE-mode-compressed
  bags (`*.db3.zstd`, or a directory bag with `compression_mode: FILE`) also
  emit no frame ids: the first TF read would decompress the entire shard to a
  temporary `.db3`, which would hang TAB for seconds on a multi-GB bag.

- Path completion is delegated to the shell's default file completion when
  bagwiz does not provide command-specific candidates. The bash script uses
  `complete -o default`; the zsh script falls back to `_files`; the fish
  script registers a `-F` rule gated by an `__bagwiz_no_candidates` condition
  that matches the same behavior.

## Migration

`<shell>` used to be a positional argument. It is now `--shell`:

```bash
bagwiz complete bash          # before — now an error
bagwiz complete --shell bash  # after
```
