# `bagwiz walk`

Walk the messages of a single topic in a ROS 2 rosbag one at a time and
render each payload as YAML, mirroring `ros2 topic echo`. Designed for
interactive inspection: the view is a pager with vim-style scroll keys,
backed by the reusable TUI SDK (`bagwiz::core::tui`). The header and
footer are pinned in place — only the body region scrolls — and any
line that does not fit the terminal width is wrapped onto continuation
lines. Wrapped continuation lines inherit the original line's leading
whitespace so YAML nesting stays visually intact. The view also redraws
cleanly on terminal resize. ROS 1 `*.bag` inputs are not supported —
convert them first with
[`bagwiz convert 1to2`](convert.md#bagwiz-convert-1to2).

## Usage

```text
bagwiz walk <input> <topic>
```

## Positional arguments

| Name    | Description                                               |
| ------- | --------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`). |
| `topic` | Topic name to inspect. Must exist in the bag.             |

## Behavior

- Decoding goes through bagwiz's unified decoder factory. For MCAP shards
  with a non-empty `ros2msg` schema embedded for the topic's type the
  schema-driven backend is used directly. Otherwise (legacy MCAP / SQLite3
  inputs) bagwiz falls back to the rosidl introspection typesupport, which
  **requires the message package to be installed and on
  `AMENT_PREFIX_PATH` at runtime**. If the typesupport library is missing,
  bagwiz reports the package name to install and exits non-zero.
- Messages are loaded lazily and cached as you advance, so `prev` is
  always `O(1)` for anything you have already seen. Only `G` (jump to
  last) can trigger a full-remaining scan.
- Pressing `→` / `Space` past the last message wraps back to the first
  with a `(wrapped to first)` status hint.
- Pressing `s` saves the **currently displayed** message body (the same
  YAML string shown in the pager, not including the header lines) to a
  file. The command prompts for an output path; press Enter with an empty
  line to write under the process current working directory using the name
  `<topic>_<index>.yaml`, where `<topic>` is the ROS topic with each `/`
  replaced by `__`, and `<index>` is the same **zero-based** message index
  as the first number in the header line `[<index> / <last>[+]]` (see the
  Header section for `<last>`).
- By default, primitive arrays with more than 32 elements (e.g. the byte
  buffer behind `sensor_msgs/Image.data` or `sensor_msgs/PointCloud2.data`)
  are summarized as `[<N items>]` to keep the pager view scannable. Pressing
  `a` toggles **full array expansion** for the rest of the walk session.
  When expanded, long arrays render as a YAML block sequence (one element
  per line under a `-` marker) so the output stays within the terminal
  width and remains valid YAML. Short arrays (≤ 32 elements) keep their
  inline `[a, b, c]` form either way. The toggle affects both on-screen
  rendering and the YAML written by `s`, so saving while expanded produces
  a full-fidelity dump of every element. Press `a` again to return to the
  summarized view.

## Layout

The visible viewport is split into three pinned regions:

```text
┌─────────────────────────────────────────────────┐
│ timestamp: ...                                  │ ← header (≥ 3 rows)
│ size:      N bytes                              │
│                                                 │
│ <decoded YAML body — scrolls>                   │ ← body
│ ...                                             │
│                                                 │
│   [i / n+]  /topic  Type    lines X-Y of M      │ ← footer (≥ 4 rows)
│   [keys legend ...]                             │
│   <status hint or blank>                        │
└─────────────────────────────────────────────────┘
```

Header and footer rows are sized to the wrapped content, so on narrow
terminals the key legend or other long lines occupy multiple rows and
the body region shrinks accordingly. The status row is always reserved
(blank when there is no message) so the body never grows or shrinks
underfoot when transient messages like `(saved /tmp/x.yaml)` or
`(wrapped to first)` appear.

## Header

Each redraw shows a two-line header (plus a blank separator before the
body):

```text
timestamp: YYYY-MM-DD HH:MM:SS.nnnnnnnnn UTC (<seconds>.<nanos>)
size:      <bytes> bytes
```

## Footer

The footer carries the message index, the topic, the type, the scroll
hint, the key legend, and a status row:

```text
  [<index> / <last>[+]]  <topic>  <type>    lines <X>-<Y> of <M>
  [→/Space] next   [←/b] prev   ...   [q] quit
  <status hint or blank>
```

`<last>` is the index of the last message currently loaded in the cache
(equivalently, the count of loaded messages minus one). The trailing `+`
after `<last>` means the bag has more messages after that index that have
not been read into the cache yet (they get pulled in on demand).

## Keys

| Key            | Action                                                             |
| -------------- | ------------------------------------------------------------------ |
| `→` / `Space`  | Next message (wraps from last back to first).                      |
| `←` / `b`      | Previous message.                                                  |
| `↑` / `k`      | Scroll body up one line.                                           |
| `↓` / `j`      | Scroll body down one line.                                         |
| `Home` / `H`   | Jump body scroll to the head.                                      |
| `End` / `T`    | Jump body scroll to the tail.                                      |
| `g`            | Jump to the first message.                                         |
| `G`            | Jump to the last message (forces a full-remaining scan).           |
| `s`            | Save as yaml - writes the current message body (prompts for path). |
| `a`            | Toggle full expansion of long primitive arrays (default off).      |
| `q` / `Ctrl-C` | Quit.                                                              |

When the body is taller than the visible window, a `lines X-Y of N`
indicator is shown above the key legend.

## Requirements

- `walk` is interactive — both stdin and stdout must be a TTY. Piping the
  output (`bagwiz walk … | less`) exits with an error.

## Example

```bash
bagwiz walk capture.mcap /sensing/imu/data
```

## Exit status

| Code | Meaning                                                                                                           |
| ---- | ----------------------------------------------------------------------------------------------------------------- |
| `0`  | Quit cleanly via `q` / `Ctrl-C`, or the topic had no messages.                                                    |
| `1`  | Bag could not be opened, the topic is absent, the decoder could not be initialized, or stdin/stdout is not a TTY. |
