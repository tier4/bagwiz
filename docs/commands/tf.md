# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merged static∪dynamic forest; edge tags `[S]`/`[D]`/`[B]` and optional TTY colors.
- [`walk`](#bagwiz-tf-walk) — interactive chain between `<from>` and `<to>` at each dynamic `/tf` update.
- [`inject-static`](#bagwiz-tf-inject-static) — copy a destination bag with `/tf_static` injected from a source bag at the destination's start time.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Replays **every** `tf2_msgs/msg/TFMessage` topic in the bag: topics whose name
ends with `tf_static` are static; all other TF topics are dynamic (`/tf`-style).

### Default mode

Collects every distinct parent→child pair from static topics (`*tf_static`) and
from dynamic topics (everything else), then draws **one** merged forest: the
union of those edges. Each branch line marks the child frame with a short
tag (`[S]`, `[D]`, `[B]`) so the edge kind is identifiable without relying on
color alone. On a color-capable TTY, the child name also uses bright blue,
yellow, or magenta for static-only, dynamic-only, or both (respectively)—hues
chosen to stay distinguishable under common color-vision deficiency, via
[rang](https://github.com/agauniyal/rang).

`├──` / `└──` / `│` box drawing is the default. Set `BAGWIZ_TF_TREE_ASCII=1` to
use `|--` / `` `-- `` / `|` instead (see [Environment](#environment)). Block
headers use double-line rules (`═`); see [Stdout layout](#stdout-layout) for
the exact order.

The static edge set, the dynamic edge set, and the **merged** set are each
validated as a union of one or more trees without contradictions. The merged
check can fail even when static and dynamic are valid on their own (for example
if the same child would need two different parents when both topic classes are
combined). In that case the command exits with a `Combined TF union: …` error.
Otherwise, if any of the following holds within the set under test, the command
exits with an error:

- The same child frame lists two different parents.
- Both `A → B` and `B → A` appear (opposite edges).
- A directed cycle exists among frames.
- A self edge `F → F` appears.

### Stdout layout

<!-- AUTO-GENERATED: bagwiz tf tree print order (sync with `run_tree` in `src/commands/tf.cpp`) -->

`tf tree` writes to stdout in this order (no leading `#` summary line):

1. **Dynamic TF topics** — `═` rule line, then an indented comma-separated list
   of dynamic TF topic names (or `(none)`).
2. **Static TF topics** — same for `*tf_static`-style topics.
3. **Legend** — `═` rule line, then one line: `static-only [S] · dynamic-only [D] · both [B]`
   (with TTY colors on the three keywords when applicable).
4. **TF tree (static ∪ dynamic edges)** — `═` rule line, then the merged forest
   (or `(none)` if there are no edges).

### Environment

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- `NO_COLOR`: if set to any value, disables ANSI colors on `tf tree`. Tags `[S]`, `[D]`, and `[B]` after each child frame name are still printed.
- `BAGWIZ_TF_TREE_ASCII`: if set to any value, uses ASCII branch glyphs instead of Unicode box drawing (see `make_tree_glyphs` in `src/commands/tf.cpp`).

Colors are also omitted when stdout is not a TTY (same effect as `NO_COLOR` for styling).

### Usage

```text
bagwiz tf tree <input>
```

### Positional arguments

| Name    | Description                                               |
| ------- | --------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`). |

### Behavior

- One pass over all TF topics (same loading path as `tf walk`).
- Topic names are grouped under two sections (`Dynamic TF topics` and
  `Static TF topics`), each with the same `═`-style rule line as the Legend
  and tree blocks; the list body is comma-separated, indented, and dim on TTY.
- Tags `[S]` / `[D]` / `[B]` are always printed after each branch child
  frame name so edge kind is readable when color is absent (`NO_COLOR`), piped
  to a file, or hard to distinguish by hue.
- When stdout is a TTY and `NO_COLOR` is unset, child names use the Legend hues
  (blue / yellow / magenta); branch glyphs stay dim gray for readability.
- The Legend block is a `═` section (not a `#` comment): one rule line, then
  the single legend line of keywords and tags. See [Stdout layout](#stdout-layout)
  and [Environment](#environment).
- Tree glyphs default to Unicode; `BAGWIZ_TF_TREE_ASCII=1` forces ASCII branch
  characters (see [Environment](#environment)).

### Examples

```bash
bagwiz tf tree capture.mcap
```

### Exit status

| Code | Meaning                                                                                                                        |
| ---- | ------------------------------------------------------------------------------------------------------------------------------ |
| `0`  | Tree written to stdout.                                                                                                        |
| `1`  | Bag could not be opened, no TFMessage topic, no decoded transforms, TF union validation failed, decoder failure, or I/O error. |

---

## `bagwiz tf walk`

Advance one-at-a-time through the TF chain between `<from>` and `<to>` at
every dynamic `/tf` update in the bag. Each timeline index renders the lookup
result at that exact stamp, so you can scrub a recorded TF tree the same way
you would scrub a YAML message stream with `bagwiz walk`. The view uses the
same TUI pager as `bagwiz walk`: the header (timestamp and TF arrow) and the
footer (index row, key legend, status row) are pinned, the body region
scrolls, and any line that does not fit the terminal width is wrapped onto
continuation lines that inherit the original line's leading whitespace.

### Usage

```text
bagwiz tf walk <input> <from> <to>
```

### Positional arguments

| Name    | Description                                                                        |
| ------- | ---------------------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                          |
| `from`  | Reference (fixed) frame — the output expresses `<to>` in this frame's coordinates. |
| `to`    | Tracked (moving) frame to sample.                                                  |

### Options

`tf walk` takes no user-defined flags. The rotation format is switched
interactively from inside the viewer — see [Keys](#keys) and
[Rotation formats](#rotation-formats).

### Behavior

- All `tf2_msgs/msg/TFMessage` topics are read in a single pass:
  - Topics whose name ends in `tf_static` are inserted as static
    transforms; everything else as dynamic transforms.
  - The set of distinct timestamps emitted by dynamic `/tf` messages
    (i.e. excluding `tf_static`) becomes the walk's timeline. Each one
    is a moment at which the TF tree observably changed.
- Before entering the interactive view, bagwiz probes
  `lookupTransform(<from>, <to>, timeline.front())`:
  - If the chain is structurally broken (frame absent, no connecting
    edges) the command exits non-zero.
  - If the chain simply hasn't been published yet at the bag's first
    dynamic stamp (typical when sensor `/tf` precedes the localizer),
    the timeline is cropped forward to the earliest stamp at which the
    chain is queryable.
  - If the chain is never queryable for any timeline stamp, the command
    exits non-zero.
- Bags with **no** dynamic `/tf` updates (only static TF topics, or no dynamic
  timestamps) still run a walk: bagwiz inserts a single timeline slot at `t=0`.
  The UI shows `[0 / 0]` plus `[STATIC TF] (no dynamic /tf in bag)` instead of
  dynamic timestamps (same `[index / last]` convention as `bagwiz walk`).
  If `<from>`→`<to>` requires a dynamic segment that never appears in the bag,
  lookup fails with an error (same idea as a broken chain).

### Layout

The viewport is split into three regions, identical in spirit to
`bagwiz walk`:

```text
┌─────────────────────────────────────────────────┐
│ timestamp: YYYY-MM-DD HH:MM:SS.nnnnnnnnn UTC …  │ ← header (≥ 3 rows)
│ TF: <from>  ->  <to>                            │
│                                                 │
│ chain: <from> -> ... -> <to>                    │ ← body (scrolls)
│                                                 │
│ translation:                                    │
│   x: …                                          │
│   …                                             │
│   [<index> / <last>]  <from> -> <to>    …       │ ← footer (≥ 4 rows)
│   [keys legend …]                               │
│   <status hint or blank>                        │
└─────────────────────────────────────────────────┘
```

Header / footer row counts grow when content wraps. The bracket line
matches `bagwiz walk`: `<index>` is zero-based and `<last>` is the
highest timeline index (count of stamps minus one). For static-only
bags the header's first row becomes `[STATIC TF]  (no dynamic /tf in
bag)` instead of a timestamp.

The body shows the lookup result at the current index. If a mid-bag gap
or a chain that ceases to publish before the bag ends causes a lookup to
fail, `tf2`'s error text is shown inline (`⚠  Lookup failed at this
index: …`) instead of crashing the walk.

### Rotation formats

The viewer starts in `quat` and the interactive `r` key cycles between the
three formats:

| Format      | Output                         |
| ----------- | ------------------------------ |
| `quat`      | Quaternion `(x, y, z, w)`.     |
| `euler_rad` | Roll / pitch / yaw in radians. |
| `euler_deg` | Roll / pitch / yaw in degrees. |

Pressing `r` advances `quat` → `euler_rad` → `euler_deg` → `quat`. The body
header line (`rotation (quaternion):` / `rotation (euler, rad):` /
`rotation (euler, deg):`) always reflects the active format.

### Keys

| Key            | Action                                                       |
| -------------- | ------------------------------------------------------------ |
| `→` / `Space`  | Next timeline index (wraps from last back to first).         |
| `←` / `b`      | Previous timeline index.                                     |
| `↑` / `k`      | Scroll body up one line.                                     |
| `↓` / `j`      | Scroll body down one line.                                   |
| `Home` / `H`   | Jump body scroll to the head.                                |
| `End` / `T`    | Jump body scroll to the tail.                                |
| `g`            | Jump to the first timeline index.                            |
| `G`            | Jump to the last timeline index.                             |
| `r`            | Cycle rotation format (quat → euler rad → euler deg → quat). |
| `q` / `Ctrl-C` | Quit.                                                        |

When the body is taller than the visible window (typically because the
`chain:` line wrapped, or the terminal is short), a `lines X-Y of N`
indicator is shown after the index row in the footer.

### Requirements

- `tf walk` is interactive — both stdin and stdout must be a TTY.
- The bag must contain at least one `tf2_msgs/msg/TFMessage` topic.

### Examples

```bash
# Track base_link in the map frame. Starts in quaternion mode; press `r`
# in the viewer to cycle to euler (rad), then again for euler (deg).
bagwiz tf walk capture.mcap map base_link
```

### Exit status

| Code | Meaning                                                                                                                                                                                               |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Quit cleanly via `q` / `Ctrl-C`.                                                                                                                                                                      |
| `1`  | Bag could not be opened; no TFMessage topic; TF load or init-time lookup failure (broken chain, missing transforms, or empty timeline after cropping); decoder failure; or stdin/stdout is not a TTY. |

---

## `bagwiz tf inject-static`

Copy `<dst>` to `<output>` with `/tf_static` from `<src>` injected as a single
message at `<dst>`'s start time. Source `header.stamp` values are rewritten to
that same time so the injected static transforms appear consistent with the
destination's timeline.

### Usage

```text
bagwiz tf inject-static <src> <dst> -o <output> [--force] [--overwrite]
```

### Positional arguments

| Name  | Description                                              |
| ----- | -------------------------------------------------------- |
| `src` | Source bag (provides `/tf_static`).                      |
| `dst` | Destination bag (copied unchanged except for static TF). |

### Options

| Flag                   | Default      | Description                                                                                           |
| ---------------------- | ------------ | ----------------------------------------------------------------------------------------------------- |
| `-o`, `--output <OUT>` | _(required)_ | Output bag path (file or directory).                                                                  |
| `--force`              | `false`      | Overwrite per-topic when `<dst>` already has messages on a `*tf_static` topic.                        |
| `--overwrite`          | `false`      | Replace `-o/--output` if it already exists. Without this flag, an existing output path stops the run. |

### Behavior

- Scans `<src>` for `tf2_msgs/msg/TFMessage` topics whose name ends in
  `tf_static` and collects a deduplicated `TransformStamped[]` per topic.
- Inherits `<output>`'s storage format from `<dst>` when `-o`'s extension
  does not disambiguate a single-file format (e.g. `.mcap`, `.db3`).
- For each `*tf_static` topic from `<src>`:
  - **Not in `<dst>`**: declare the topic with `<src>`'s schema, then emit
    the merged static payload.
  - **Present in `<dst>` with no messages**: keep `<dst>`'s schema, emit the
    merged static payload.
  - **Present in `<dst>` with messages, `--force` not set**: abort with a
    conflict error listing the affected topic and existing message count.
  - **Present in `<dst>` with messages, `--force` set**: drop the existing
    messages from the output stream and emit the merged static payload
    instead.
- Stream-copies every other message from `<dst>` to `<output>` unchanged.
- Emits exactly one merged `TFMessage` per static topic at `<dst>`'s
  `start_ns`, with every `header.stamp` rewritten to that timestamp.

### Exit status

| Code | Meaning                                                                                                                                                                                                                                            |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Output bag written successfully.                                                                                                                                                                                                                   |
| `1`  | Failure to open `<src>` or `<dst>`; output path already exists without `--overwrite`; non-positive `<dst>` start time; existing `*tf_static` payload conflict without `--force`; topic-type mismatch; decoder/serializer error; or writer failure. |
