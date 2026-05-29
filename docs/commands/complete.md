# `bagwiz complete`

Generate a shell completion script for `bagwiz`.

## Usage

```text
bagwiz complete <shell> [--install] [--force]
```

## Positional arguments

| Name    | Description                                              |
| ------- | -------------------------------------------------------- |
| `shell` | Shell completion format to emit (`bash`, `zsh`, `fish`). |

## Options

| Flag        | Description                                                                               |
| ----------- | ----------------------------------------------------------------------------------------- |
| `--install` | Write the script to the shell's standard completion directory instead of stdout.          |
| `--force`   | Overwrite an existing file when used with `--install`. Has no effect without `--install`. |

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

Default targets (XDG variables are honored when set):

| Shell  | Path                                                                  |
| ------ | --------------------------------------------------------------------- |
| `bash` | `${XDG_DATA_HOME:-~/.local/share}/bash-completion/completions/bagwiz` |
| `zsh`  | `~/.zsh/completions/_bagwiz`                                          |
| `fish` | `${XDG_CONFIG_HOME:-~/.config}/fish/completions/bagwiz.fish`          |

Re-running `--install` against an existing file fails unless `--force` is
passed.

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
  - `bagwiz <cmd> -<TAB>` for every registered command (including
    `complete`, `convert`, `ls`, `tf`, `traj`, `walk`)
  - `bagwiz <cmd> <subcommand> -<TAB>` for every nested subcommand
    (`convert format`, `tf tree`, `traj dump`, `traj join`)
- Selected option values are completed where bagwiz has a closed set, such as
  `--storage <mcap|sqlite3>`.
- Commands that take a `<topic>` positional argument complete it by opening
  `<input>` as a ROS 2 rosbag and listing topics with names that start with
  the current prefix. The currently-covered positions are:
  - `bagwiz walk <input> <topic>`
  - `bagwiz traj dump <input> <topic> <output>`
  - `bagwiz traj join <input> <traj_file> <topic>`

  Paths beginning with `~/` are expanded against the current user's home
  directory before opening the bag. Topic completion is suppressed when the
  earlier positional slots have been replaced by flags so flag-value
  completion takes over.

- TF frame-id positions complete to the set of distinct `header.frame_id`
  and `child_frame_id` values observed in the input bag's
  `tf2_msgs/msg/TFMessage` topics (static + dynamic). Coverage:
  - `bagwiz traj dump <input> ... --from <FRAME>` / `--to <FRAME>`
  - `bagwiz traj join <input> ... --from <FRAME>` / `--to <FRAME>`

  The bag is opened lazily and only the first ~5000 TF messages are scanned
  so per-keystroke latency stays bounded on large bags. When the bag opens
  cleanly but carries no TF data at all, a single
  `NO-TF-FRAMES-FOUND-IN-BAG` sentinel is emitted so the empty result is
  visibly distinct from the shell's silent file-completion fallback. When
  the bag path does not exist or the input slot is itself a flag, no
  candidates are emitted and the shell's default file completion takes
  over.

- Path completion is delegated to the shell's default file completion when
  bagwiz does not provide command-specific candidates. The bash script uses
  `complete -o default`; the zsh script falls back to `_files`; the fish
  script registers a `-F` rule gated by an `__bagwiz_no_candidates` condition
  that matches the same behavior.
