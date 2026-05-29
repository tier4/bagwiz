# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merged static∪dynamic forest; edge tags `[S]`/`[D]`/`[B]` and optional TTY colors.
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

- One pass over all TF topics.
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
