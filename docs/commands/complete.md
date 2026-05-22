# `bagwiz complete`

Generate a shell completion script for `bagwiz`.

## Usage

```text
bagwiz complete <shell>
```

## Positional arguments

| Name    | Description                                                |
| ------- | ---------------------------------------------------------- |
| `shell` | Shell completion format to emit. Only `bash` is supported. |

## Supported shells

| Shell  | Status    |
| ------ | --------- |
| `bash` | Supported |

The completion candidate engine is shell-independent. Adding a shell such as
`zsh` or `fish` only requires adding a shell definition and a script renderer
that calls the existing hidden `bagwiz __complete` protocol.

## Installing bash completion

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

## Behavior

- Top-level commands and known nested subcommands are completed statically.
- Selected option values are completed where bagwiz has a closed set, such as
  `--storage <mcap|sqlite3>` and `tf walk --rot <quat|euler|euler_rad|euler_deg>`.
- `bagwiz walk <input> <topic>` completes `<topic>` by opening `<input>` as a
  ROS 2 rosbag and listing topics with names that start with the current
  prefix. Paths beginning with `~/` are expanded against the current user's
  home directory before opening the bag.
- Path completion is delegated to bash's default file completion when bagwiz
  does not provide command-specific candidates.
