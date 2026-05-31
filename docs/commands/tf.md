# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merge one or more `tf2_msgs/msg/TFMessage` topics into one TF frame tree, colored per topic.
- [`static`](#bagwiz-tf-static) — resolve the rigid transform from `<from>` to `<to>` using only the bag's static TF tree; print translation/quaternion/RPY or JSON.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Merges one or more `tf2_msgs/msg/TFMessage` topics (`<topics>...`) into a single
TF frame tree built from the union of their distinct parent→child edges. Any TF
topic works — a dynamic `/tf`-style topic or a static `*tf_static` one; bagwiz
no longer classifies them as static vs dynamic. With two or more topics each
topic's edges are drawn in a distinct color and tagged so their source is
identifiable; with a single topic the tree is drawn plain.

Each `<topic>` supports TAB completion: only `tf2_msgs/msg/TFMessage` topics in
the input bag are offered as candidates, at every topic position (see
[`bagwiz complete`](complete.md)). A topic repeated on the command line is
treated once.

### Validation

The command exits with an error (and prints nothing) when the topics cannot
form one consistent tree. Specifically:

- **Per topic** — each topic's own edges must form a valid forest. It is an
  error if, within a single topic, the same child lists two different parents,
  both `A → B` and `B → A` appear, a directed cycle exists, or a self edge
  `F → F` appears.
- **Across topics** — no edge may be defined on more than one topic. If two
  topics publish the same `parent → child` transform the command reports which
  two topics share it.
- **Merged** — the union of all topics' edges must also form a valid forest.
  This catches conflicts that appear only after merging, e.g. two topics each
  giving the same child a different parent.

### Stdout layout

<!-- AUTO-GENERATED: bagwiz tf tree print order (sync with `run_tree` in `src/commands/tf.cpp`) -->

With a **single** topic, `tf tree` writes:

1. A `═` rule line naming the topic: `═══ TF tree (<topic>) ═══`.
2. The forest: one root frame per tree (each `●`-prefixed, bold on a TTY),
   followed by its descendants on `├──` / `└──` branch lines (plain names).

With **two or more** topics it writes:

1. A `═══ Topics ═══` rule, then one `[N] <topic>` line per topic (in the
   order given on the command line), each colored with that topic's color on a
   TTY.
2. A `═══ TF tree ═══` rule, then the merged forest. Each child frame carries a
   `[N]` tag identifying the topic that defined its parent→child edge, and on a
   TTY the child name is drawn in that topic's color.

### Terminal styling

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- On a color-capable TTY (and when `NO_COLOR` is unset) section headers and root
  lines are bold and branch glyphs are dim gray. For multiple topics each
  topic is assigned a color from a fixed palette (bright blue, yellow, magenta,
  cyan, green, red, then wrapping) and its edges' child names use that color;
  for a single topic child names use the terminal's default color.
- The `[N]` tags always print for multiple topics, so the source topic stays
  identifiable under `NO_COLOR`, when piped to a file, or when colors repeat
  past the palette size.
- `├──` / `└──` / `│` box drawing is the default. Set `BAGWIZ_TF_TREE_ASCII=1`
  to use `|--` / `` `-- `` / `|` instead (see [Environment](#environment)).

### Environment

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- `NO_COLOR`: if set to any value, disables ANSI colors on `tf tree`. The `[N]` topic tags are still printed.
- `BAGWIZ_TF_TREE_ASCII`: if set to any value, uses ASCII branch glyphs instead of Unicode box drawing (see `make_tree_glyphs` in `src/commands/tf.cpp`).

Colors are also omitted when stdout is not a TTY (same effect as `NO_COLOR` for styling).

### Usage

```text
bagwiz tf tree <input> <topic> [<topic>...]
```

### Positional arguments

| Name     | Description                                                                              |
| -------- | ---------------------------------------------------------------------------------------- |
| `input`  | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                                |
| `topics` | One or more `tf2_msgs/msg/TFMessage` topics to merge and render (e.g. `/tf /tf_static`). |

### Behavior

- One pass over the requested topics; their distinct parent→child edges are
  merged into one tree.
- Every `<topic>` must name a `tf2_msgs/msg/TFMessage` topic that exists in the
  bag. If any is missing or has another message type, the command exits with an
  error that lists the offending names and the bag's available TF topics on
  stderr.
- Tree glyphs default to Unicode; `BAGWIZ_TF_TREE_ASCII=1` forces ASCII branch
  characters (see [Environment](#environment)).

### Examples

```bash
bagwiz tf tree capture.mcap /tf
bagwiz tf tree capture.mcap /tf /tf_static
```

Single-topic output (plain):

```text
═══ TF tree (/tf) ═══
● map
└── odom
    └── base_link
        ├── camera
        └── lidar
```

Merged output for `tf tree capture.mcap /tf /tf_static` (the `[N]` tags map to
the `Topics` legend; on a TTY each topic's edges are also colored):

```text
═══ Topics ═══
  [1] /tf
  [2] /tf_static

═══ TF tree ═══
● map
└── odom [1]
    └── base_link [1]
        ├── camera [1]
        ├── imu [2]
        └── lidar [1]
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                            |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Tree written to stdout.                                                                                                                                                                                                                            |
| `1`  | Bag could not be opened, has no TFMessage topic, a `<topic>` is missing or not a TFMessage topic, no transforms were decoded, a topic shares an edge with another, TF tree validation failed (per-topic or merged), decoder failure, or I/O error. |

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
