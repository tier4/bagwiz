# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merged static∪dynamic forest; edge tags `[S]`/`[D]` and optional TTY colors.
- [`static`](#bagwiz-tf-static) — resolve the rigid transform from `<from>` to `<to>` using only the bag's static TF tree; print translation/quaternion/RPY or JSON.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Replays **every** `tf2_msgs/msg/TFMessage` topic in the bag: topics whose name
ends with `tf_static` are static; all other TF topics are dynamic (`/tf`-style).

### Default mode

Collects every distinct parent→child pair from static topics (`*tf_static`) and
from dynamic topics (everything else), then draws **one** merged forest: the
union of those edges. Each branch line marks the child frame with a short
tag (`[S]`, `[D]`) so the edge kind is identifiable without relying on
color alone. On a color-capable TTY, the child name also uses bright blue or
yellow for static or dynamic (respectively)—hues chosen to stay
distinguishable under common color-vision deficiency, via
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

The static and dynamic edge sets must also be **disjoint**: bagwiz does not
allow the same `parent → child` transform to be published on both a static
(`*tf_static`) and a dynamic topic. If an edge appears in both classes the
command exits with a `TF union: edge '…' -> '…' is published on both static and
dynamic topics …` error. Consequently every branch is exactly one of static
`[S]` or dynamic `[D]` — there is no "both" classification.

### Stdout layout

<!-- AUTO-GENERATED: bagwiz tf tree print order (sync with `run_tree` in `src/commands/tf.cpp`) -->

`tf tree` writes to stdout in this order (no leading `#` summary line):

1. **Dynamic TF topics** — `═` rule line, then an indented comma-separated list
   of dynamic TF topic names (or `(none)`).
2. **Static TF topics** — same for `*tf_static`-style topics.
3. **Legend** — `═` rule line, then one line: `static [S] · dynamic [D]`
   (with TTY colors on the two keywords when applicable).
4. **TF tree (static ∪ dynamic edges)** — `═` rule line, then the merged forest
   (or `(none)` if there are no edges).

### Environment

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- `NO_COLOR`: if set to any value, disables ANSI colors on `tf tree`. Tags `[S]` and `[D]` after each child frame name are still printed.
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
- Tags `[S]` / `[D]` are always printed after each branch child
  frame name so edge kind is readable when color is absent (`NO_COLOR`), piped
  to a file, or hard to distinguish by hue.
- When stdout is a TTY and `NO_COLOR` is unset, child names use the Legend hues
  (blue / yellow); branch glyphs stay dim gray for readability.
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

## `bagwiz tf static`

Resolves the rigid-body transform from `<from>` to `<to>` using **only** the
bag's static TF (topics whose name ends with `tf_static`). Dynamic `/tf` topics
are intentionally ignored. The transform is composed across the whole static
chain, so `<from>` and `<to>` need not be directly adjacent — any two frames
connected through the static tree work.

### Direction convention

The printed transform is `lookupTransform(target=<to>, source=<from>)`, i.e. the
same result as:

```bash
ros2 run tf2_ros tf2_echo <from> <to>
```

The translation is `<from>`'s origin expressed in the `<to>` frame, and the
rotation re-expresses a `<from>`-frame orientation in `<to>`. Swapping the two
arguments yields the inverse transform.

### Usage

```text
bagwiz tf static <input> <from> <to> [--json]
```

### Positional arguments

| Name    | Description                                               |
| ------- | --------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`). |
| `from`  | Source frame id.                                          |
| `to`    | Target frame id.                                          |

### Options

| Flag     | Description                                       |
| -------- | ------------------------------------------------- |
| `--json` | Emit the transform as JSON instead of human text. |

### Output

Human form (monochrome, like `tf2_echo`):

```text
Transform: base_link -> lidar  (static)
  Translation (x, y, z):          [-0.000000, 1.000000, -0.500000]
  Rotation quaternion (x,y,z,w):  [0.000000, 0.000000, -0.707107, 0.707107]
  Rotation RPY (rad):             [0.000000, 0.000000, -1.570796]
  Rotation RPY (deg):             [0.000000, 0.000000, -90.000000]
```

JSON form (`--json`, pretty-printed; full-precision doubles):

```json
{
  "from": "base_link",
  "to": "lidar",
  "translation": { "x": 0.0, "y": 1.0, "z": -0.5 },
  "rotation": { "x": 0.0, "y": 0.0, "z": -0.7071067811865475, "w": 0.7071067811865476 },
  "rpy_rad": { "roll": 0.0, "pitch": 0.0, "yaw": -1.5707963267948963 },
  "rpy_deg": { "roll": 0.0, "pitch": 0.0, "yaw": -89.99999999999999 }
}
```

### Examples

```bash
bagwiz tf static capture.mcap base_link lidar
bagwiz tf static capture.mcap base_link lidar --json
```

### Exit status

| Code | Meaning                                                                                                                                                                                                     |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Transform written to stdout.                                                                                                                                                                                |
| `1`  | Bag could not be opened, no static TF topic, decode failure, the frames are not connected through the static tree, or I/O error. When a frame is unknown, the available static frames are listed on stderr. |
