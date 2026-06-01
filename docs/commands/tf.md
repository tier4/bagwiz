# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merge one or more `tf2_msgs/msg/TFMessage` topics into one TF frame tree, colored per topic.
- [`static`](#bagwiz-tf-static) — resolve the rigid transform from `<from>` to `<to>` using only the bag's static TF tree; print translation/quaternion/RPY or JSON.
- [`walk`](#bagwiz-tf-walk) — merge every TF topic into one buffer and step interactively through the times the merged TF changed, showing the `<from>` → `<to>` transform at each.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Merges one or more `tf2_msgs/msg/TFMessage` topics (`<topics>...`) into a single
TF frame tree built from the union of their distinct parent→child edges. Any TF
topic works — a dynamic `/tf`-style topic or a static `*tf_static` one; bagwiz
no longer classifies them as static vs dynamic. With two or more topics each
topic's edges are drawn in a distinct color and tagged so their source is
identifiable; with a single topic the tree is drawn plain.

When `<topics>` is omitted, bagwiz defaults to **every** `tf2_msgs/msg/TFMessage`
topic in the bag, sorted by name. The result is identical to listing all of them
explicitly: two or more TF topics produce the colored, `[N]`-tagged merged view,
and a bag with a single TF topic produces the plain tree.

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
bagwiz tf tree <input> [<topic>...]
```

### Positional arguments

| Name     | Description                                                                                                                                                |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input`  | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`).                                                                                                  |
| `topics` | Zero or more `tf2_msgs/msg/TFMessage` topics to merge and render (e.g. `/tf /tf_static`). When omitted, all TF topics in the bag are used, sorted by name. |

### Behavior

- One pass over the requested topics; their distinct parent→child edges are
  merged into one tree.
- When no `<topic>` is given, every `tf2_msgs/msg/TFMessage` topic in the bag is
  merged (sorted by name) — equivalent to listing them all explicitly, so the
  same per-topic / merged validation applies.
- Every `<topic>` must name a `tf2_msgs/msg/TFMessage` topic that exists in the
  bag. If any is missing or has another message type, the command exits with an
  error that lists the offending names and the bag's available TF topics on
  stderr.
- Tree glyphs default to Unicode; `BAGWIZ_TF_TREE_ASCII=1` forces ASCII branch
  characters (see [Environment](#environment)).

### Examples

```bash
bagwiz tf tree capture.mcap              # merge every TF topic in the bag
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

| Code | Meaning                                                                                                                                                                                                                                                  |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Tree written to stdout.                                                                                                                                                                                                                                  |
| `1`  | Bag could not be opened, has no TFMessage topic, a given `<topic>` is missing or not a TFMessage topic, no transforms were decoded, a topic shares an edge with another, TF tree validation failed (per-topic or merged), decoder failure, or I/O error. |

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

`<from>` and `<to>` support TAB completion. Because `tf static` resolves only
the static tree, the candidates are restricted to frame ids found in the bag's
static `*tf_static` topics (see [`bagwiz complete`](complete.md)).

### Options

| Flag     | Description                                       |
| -------- | ------------------------------------------------- |
| `--json` | Emit the transform as JSON instead of human text. |

### Output

Human form (monochrome, like `tf2_echo`):

```text
Transform: base_link -> lidar  (static)
  t:
    x: -0.000000
    y: 1.000000
    z: -0.500000
  r:
    quat:
      x: 0.000000
      y: 0.000000
      z: -0.707107
      w: 0.707107
    rpy_rad:
      roll: 0.000000
      pitch: 0.000000
      yaw: -1.570796
    rpy_deg:
      roll: 0.000000
      pitch: 0.000000
      yaw: -90.000000
```

JSON form (`--json`, pretty-printed; full-precision doubles). Translation is
under `t`; rotation is under `r` as a quaternion (`quat`) plus RPY in radians
(`rpy_rad`) and degrees (`rpy_deg`). Object keys are emitted in alphabetical
order (nlohmann's default), so consumers should not rely on key ordering:

```json
{
  "from": "base_link",
  "to": "lidar",
  "t": { "x": 0.0, "y": 1.0, "z": -0.5 },
  "r": {
    "quat": { "x": 0.0, "y": 0.0, "z": -0.7071067811865475, "w": 0.7071067811865476 },
    "rpy_rad": { "roll": 0.0, "pitch": 0.0, "yaw": -1.5707963267948963 },
    "rpy_deg": { "roll": 0.0, "pitch": 0.0, "yaw": -89.99999999999999 }
  }
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

---

## `bagwiz tf walk`

Merges **every** `tf2_msgs/msg/TFMessage` topic in the bag (`/tf`, `*tf_static`,
and any other TF topic) into one TF buffer, then steps through the distinct
times at which the merged TF changed — one step per timestamp — resolving the
`<from>` → `<to>` transform at each. Unlike [`tf static`](#bagwiz-tf-static),
`tf walk` does **not** classify transforms as static vs dynamic: static topics
are merged in alongside dynamic ones so a chain that crosses both (e.g. a
dynamic `map → base_link` plus a static `base_link → lidar`) resolves at every
step. The view is the same interactive pager as [`bagwiz walk`](walk.md).

### Direction convention

Each step shows `lookupTransform(target=<to>, source=<from>)`, identical to
`tf static` and to `ros2 run tf2_ros tf2_echo <from> <to>`: the translation is
`<from>`'s origin expressed in the `<to>` frame.

### Usage

```text
bagwiz tf walk <input> <from> <to>
```

`tf walk` requires an interactive terminal (stdin and stdout must be a TTY).

### Positional arguments

| Name    | Description                                               |
| ------- | --------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`). |
| `from`  | Source frame id.                                          |
| `to`    | Target frame id.                                          |

Both `<from>` and `<to>` support TAB completion from the bag's TF frame ids
across **all** TF topics (static + dynamic, since `tf walk` merges them; see
[`bagwiz complete`](complete.md)). This is broader than `tf static`, which
restricts the same slots to static `*tf_static` frames.

### Keys

| Key                       | Action                                    |
| ------------------------- | ----------------------------------------- |
| `→` / `Space`             | next timestamp (wraps from last to first) |
| `←` / `b`                 | previous timestamp                        |
| `↑` / `k`, `↓` / `j`      | scroll the transform body up / down       |
| `Home` / `H`, `End` / `T` | jump the body scroll to head / tail       |
| `g` / `G`                 | jump to the first / last timestamp        |
| `q` / `Ctrl-C`            | quit                                      |

### Output

Per step, the header shows the timestamp and the body shows the resolved
transform (monochrome, like `tf2_echo`; no `(static)` tag since the walk does
not classify transforms):

```text
timestamp: 2026-01-01 12:00:00.000000000 UTC (1767268800.000000000)

Transform: base_link -> lidar
  t:
    x: -0.000000
    y: 1.000000
    z: -0.500000
  r:
    quat:
      x: 0.000000
      y: 0.000000
      z: -0.707107
      w: 0.707107
    rpy_rad:
      roll: 0.000000
      pitch: 0.000000
      yaw: -1.570796
    rpy_deg:
      roll: 0.000000
      pitch: 0.000000
      yaw: -90.000000
```

When the merged TF cannot connect `<from>` and `<to>` at a given timestamp
(e.g. the chain is not yet complete), that step shows a `⚠` warning line with
the tf2 reason instead of a transform, and navigation continues.

### Examples

```bash
bagwiz tf walk capture.mcap base_link lidar
bagwiz tf walk capture.mcap map base_link
```

### Exit status

| Code | Meaning                                                                                                                           |
| ---- | --------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The pager ran and the user quit.                                                                                                  |
| `1`  | Not a TTY, the bag could not be opened, it has no `tf2_msgs/msg/TFMessage` topic, or the TF topics carry no decodable transforms. |
