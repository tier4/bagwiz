# `bagwiz tf`

TF inspection on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merge one or more `tf2_msgs/msg/TFMessage` topics into one TF frame tree, colored by static vs dynamic (`static` / `dynamic` selectors supported).
- [`static calc`](#bagwiz-tf-static-calc) — resolve the rigid transform from `<from>` to `<to>` using only the bag's static TF tree; print translation/quaternion/RPY or JSON. (`static` is a command group; `calc` is its action.)
- [`static cp`](#bagwiz-tf-static-cp) — copy every static TF topic from `<src>` into `<dst>` (in place, or to a new bag via `-o`), preserving topic names and stamping each at `<dst>`'s start time.
- [`walk`](#bagwiz-tf-walk) — merge every TF topic into one buffer and step interactively through the times the merged TF changed, showing the `<from>` → `<to>` transform at each.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Merges one or more `tf2_msgs/msg/TFMessage` topics (`<topics>...`) into a single
TF frame tree built from the union of their distinct parent→child edges. In the
merged tree each edge is colored by whether it came from a **static**
(`*tf_static`) or a **dynamic** topic. When the tree contains both kinds, a
legend is printed and each child frame is colored and tagged `[S]` (static) or
`[D]` (dynamic); when only one kind is present the tree is drawn plain and the
header names the category, e.g. `═══ TF tree (static) ═══`.

`<topics>` accepts two reserved selectors in addition to literal topic names:

- `static` — expands to **every** `*tf_static` topic in the bag.
- `dynamic` — expands to every non-static TF topic in the bag.

They compose with each other and with literal names. For example
`tf tree bag dynamic /extra_tf` merges all dynamic TF topics plus `/extra_tf`,
and `tf tree bag static` shows the merged static tree alone. (ROS topic names
start with `/`, so the bare words `static` / `dynamic` never collide with a real
topic.) When `<topics>` is omitted, bagwiz defaults to **every**
`tf2_msgs/msg/TFMessage` topic in the bag.

Literal `<topic>` names support TAB completion: only `tf2_msgs/msg/TFMessage`
topics in the input bag are offered as candidates (see
[`bagwiz complete`](complete.md)). A topic repeated on the command line is
treated once.

### Validation

The command exits with an error (and prints nothing) when the selected topics
cannot form one consistent tree. Specifically:

- **Merge conflict** — the merge is rejected when the same `child_frame_id` is
  given a different parent by two different topics, or when a frame is declared
  by both a static and a dynamic topic. This is consistent with the merge-and-
  detect-conflicts behavior of [`bagwiz tf static calc`](#bagwiz-tf-static-calc) and
  [`bagwiz traj dump`](traj.md). Two topics declaring the **same** edge (same
  parent, same class) are fine.
- **Forest** — the union of all selected edges must form a valid forest: no
  frame may have two parents, both `A → B` and `B → A` cannot appear, no
  directed cycle, and no self edge `F → F`.

### Stdout layout

<!-- AUTO-GENERATED: bagwiz tf tree print order (sync with `run_tree` in `src/commands/tf.cpp`) -->

When the merged tree is a **single category** (only static, or only dynamic),
`tf tree` writes:

1. A `═` rule naming the category: `═══ TF tree (static) ═══` or
   `═══ TF tree (dynamic) ═══`.
2. The forest: one root frame per tree (each `●`-prefixed, bold on a TTY),
   followed by its descendants on `├──` / `└──` branch lines (plain names).

When it contains **both** static and dynamic edges it writes:

1. A `═══ Legend ═══` rule, then a `[D] dynamic` line and a `[S] static` line,
   each colored with that category's color on a TTY.
2. A `═══ TF tree ═══` rule, then the merged forest. Each child frame carries a
   `[S]` / `[D]` tag for its edge's category, and on a TTY the child name is
   drawn in that category's color.

### Terminal styling

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- On a color-capable TTY (and when `NO_COLOR` is unset) section headers and root
  lines are bold and branch glyphs are dim gray. In a mixed tree, dynamic edges
  are bright cyan and static edges bright yellow; a single-category tree uses
  the terminal's default color.
- The `[S]` / `[D]` tags always print in a mixed tree, so the category stays
  identifiable under `NO_COLOR` or when piped to a file.
- `├──` / `└──` / `│` box drawing is the default. Set `BAGWIZ_TF_TREE_ASCII=1`
  to use `|--` / `` `-- `` / `|` instead (see [Environment](#environment)).

### Environment

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `src/commands/tf.cpp`) -->

- `NO_COLOR`: if set to any value, disables ANSI colors on `tf tree`. The `[S]` / `[D]` category tags are still printed.
- `BAGWIZ_TF_TREE_ASCII`: if set to any value, uses ASCII branch glyphs instead of Unicode box drawing (see `make_tree_glyphs` in `src/commands/tf.cpp`).

Colors are also omitted when stdout is not a TTY (same effect as `NO_COLOR` for styling).

### Usage

```text
bagwiz tf tree <input> [<topic-or-selector>...]
```

### Positional arguments

| Name     | Description                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `input`  | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                                                                                                                           |
| `topics` | Zero or more `tf2_msgs/msg/TFMessage` topics and/or the selectors `static` / `dynamic` (e.g. `/tf /tf_static`, `static`, `dynamic /extra_tf`). When omitted, all TF topics in the bag are merged. |

### Behavior

- One pass over the selected topics; their distinct parent→child edges are
  merged into one tree, each tagged static or dynamic by its source.
- `static` / `dynamic` expand to all static / dynamic TF topics; any other token
  must name a `tf2_msgs/msg/TFMessage` topic that exists in the bag. An unknown
  literal exits with an error listing the offending names and the bag's
  available TF topics on stderr.
- When no `<topic>` is given, every `tf2_msgs/msg/TFMessage` topic in the bag is
  merged.
- Tree glyphs default to Unicode; `BAGWIZ_TF_TREE_ASCII=1` forces ASCII branch
  characters (see [Environment](#environment)).

### Examples

```bash
bagwiz tf tree capture.mcap                  # merge every TF topic in the bag
bagwiz tf tree capture.mcap static           # only the static (*tf_static) tree
bagwiz tf tree capture.mcap dynamic          # only the dynamic tree
bagwiz tf tree capture.mcap dynamic /extra_tf  # all dynamic topics + /extra_tf
bagwiz tf tree capture.mcap /tf /tf_static   # explicit merge
```

Single-category output, e.g. `tf tree capture.mcap dynamic` (plain):

```text
═══ TF tree (dynamic) ═══
● map
└── odom
    └── base_link
```

Mixed output for `tf tree capture.mcap /tf /tf_static`, where `map → base_link`
is dynamic and the sensor mounts are static (on a TTY the names are also colored
cyan/yellow):

```text
═══ Legend ═══
  [D] dynamic
  [S] static

═══ TF tree ═══
● map
└── base_link [D]
    ├── camera [S]
    ├── imu [S]
    └── lidar [S]
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                         |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Tree written to stdout.                                                                                                                                                                                                                         |
| `1`  | Bag could not be opened, has no TFMessage topic, a token is neither a TFMessage topic nor a selector, the selectors matched nothing, no transforms were decoded, a TF merge conflict, TF tree validation failed, decoder failure, or I/O error. |

---

## `bagwiz tf static calc`

`static` is a command group for working with the bag's static TF tree. Its
actions are `calc` (resolve a transform, below) and [`cp`](#bagwiz-tf-static-cp)
(copy static TF between bags), so the full invocation is
`bagwiz tf static calc ...`. Running `bagwiz tf static` without an action prints
an error and the group's help.

Resolves the rigid-body transform from `<from>` to `<to>` using **only** the
bag's static TF (topics whose name ends with `tf_static`). Dynamic `/tf` topics
are intentionally ignored. The transform is composed across the whole static
chain, so `<from>` and `<to>` need not be directly adjacent — any two frames
connected through the static tree work. The printed `transform:` line lists
that full chain (every intermediate frame joined with `->`), not just the two
endpoints.

When the bag has **several** static topics (e.g. `/tf_static` and
`/sensing/tf_static`), they are all merged into one static tree. The merge is
rejected if the topics disagree: the command exits with an error when the same
`child_frame_id` is given a different parent by two different topics. Two topics
declaring the **same** edge (same parent) are fine. This matches the merge-and-
detect-conflicts behavior of [`bagwiz traj dump`](traj.md).

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
bagwiz tf static calc <input> <from> <to> [--json]
```

### Positional arguments

| Name    | Description                                                             |
| ------- | ----------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`). |
| `from`  | Source frame id.                                                        |
| `to`    | Target frame id.                                                        |

`<from>` and `<to>` support TAB completion. Because `tf static calc` resolves
only the static tree, the candidates are restricted to frame ids found in the
bag's static `*tf_static` topics (see [`bagwiz complete`](complete.md)).

### Options

| Flag     | Description                                       |
| -------- | ------------------------------------------------- |
| `--json` | Emit the transform as JSON instead of human text. |

### Output

Human form (monochrome, like `tf2_echo`). The `transform:` line lists the full
resolved frame chain from `<from>` to `<to>`, not just the endpoints (here
`base_link` reaches `lidar` through `sensor_kit_base_link`):

```text
transform: base_link -> sensor_kit_base_link -> lidar  (static)
  translation:
    x: -0.000000
    y: 1.000000
    z: -0.500000
  rotation:
    quaternion:
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
under `translation`; rotation is under `rotation` as a quaternion
(`quaternion`) plus RPY in radians (`rpy_rad`) and degrees (`rpy_deg`). The
JSON carries only the `from` / `to` endpoints, not the intermediate chain.
Object keys are emitted in alphabetical order (nlohmann's default), so
consumers should not rely on key ordering:

```json
{
  "from": "base_link",
  "to": "lidar",
  "translation": { "x": 0.0, "y": 1.0, "z": -0.5 },
  "rotation": {
    "quaternion": { "x": 0.0, "y": 0.0, "z": -0.7071067811865475, "w": 0.7071067811865476 },
    "rpy_rad": { "roll": 0.0, "pitch": 0.0, "yaw": -1.5707963267948963 },
    "rpy_deg": { "roll": 0.0, "pitch": 0.0, "yaw": -89.99999999999999 }
  }
}
```

### Examples

```bash
bagwiz tf static calc capture.mcap base_link lidar
bagwiz tf static calc capture.mcap base_link lidar --json
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Transform written to stdout.                                                                                                                                                                                                                                                           |
| `1`  | Bag could not be opened, no static TF topic, decode failure, a TF merge conflict between static topics (same child, different parents), the frames are not connected through the static tree, or I/O error. When a frame is unknown, the available static frames are listed on stderr. |

---

## `bagwiz tf static cp`

Copies every static TF topic (a `tf2_msgs/msg/TFMessage` topic whose name ends
with `tf_static`) from `<src>` into `<dst>`, preserving each topic's original
name. Dynamic `/tf` topics in `<src>` are ignored. Each copied topic is written
as a single `TFMessage`: a static topic that was re-published several times in
`<src>` collapses to one latched message carrying the final transform per
`child_frame_id`.

### Timestamp

Every copied message is stamped at `<dst>`'s start time — both the message's
receive time and the `header.stamp` of every transform it carries are set to the
earliest message timestamp in `<dst>`. The source timestamps are not preserved;
this places the latched static TF at the very start of the destination's
timeline, where a static transform is expected to already hold.

### Output modes

- Default (no `-o`): `<dst>` is rewritten in place via an atomic tmp-swap that
  preserves its storage format and layout. If the pass fails, `<dst>` is left
  untouched.
- `-o <output>`: `<dst>` is left untouched and the result (`<dst>`'s messages
  plus the copied static TF) is written to `<output>`. The storage format and
  layout follow `<output>`'s extension (`.mcap` / `.db3` / a directory).

### `-w`, `--overwrite`

A single flag that permits clobbering either conflict:

- The `-o <output>` path already exists — it is replaced.
- `<dst>` already contains a static topic whose name collides with one being
  copied — its existing messages are dropped and replaced by `<src>`'s.

Without `-w`/`--overwrite`, either conflict aborts the run with an explanatory
error and leaves `<dst>` (and any existing output) untouched. A collision with a
destination topic of a **different** message type is always an error, regardless
of the flag.

### Usage

```text
bagwiz tf static cp <src> <dst> [-o <output>] [-w|--overwrite]
```

`<src>` and `<dst>` are read; `<dst>` (or `<output>`) is the write target —
source-before-destination, like `cp src dst`.

### Positional arguments

| Name  | Description                                                                       |
| ----- | --------------------------------------------------------------------------------- |
| `src` | Source rosbag to copy static TF from (rosbag2 directory, `*.mcap`, `*.db3`, ...). |
| `dst` | Destination rosbag to copy static TF into (rewritten in place unless `-o`).       |

### Options

| Flag                | Description                                                                                             |
| ------------------- | ------------------------------------------------------------------------------------------------------- |
| `-o`, `--output`    | Write the result to this new bag instead of rewriting `<dst>` in place.                                 |
| `-w`, `--overwrite` | Replace an existing `-o`/`--output` path, and replace any colliding static topic's messages in `<dst>`. |

### Examples

```bash
bagwiz tf static cp donor.mcap target.mcap              # rewrite target.mcap in place
bagwiz tf static cp donor.mcap target.mcap -o merged.mcap  # write a new bag
bagwiz tf static cp donor.mcap target.mcap -w  # replace a colliding /tf_static
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                 |
| ---- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Static TF copied; `<dst>` rewritten or `<output>` written.                                                                                                                                                                              |
| `1`  | A bag could not be opened, `<src>` has no static TF topic carrying transforms, a decode/serialize failure, an unresolved conflict (existing output or colliding topic without `-w`/`--overwrite`, or a type mismatch), or an I/O error. |

---

## `bagwiz tf walk`

Merges **every** `tf2_msgs/msg/TFMessage` topic in the bag (`/tf`, `*tf_static`,
and any other TF topic) into one TF buffer, then steps through the distinct
times at which the merged TF changed — one step per timestamp — resolving the
`<from>` → `<to>` transform at each. Unlike [`tf static calc`](#bagwiz-tf-static-calc),
`tf walk` does **not** classify transforms as static vs dynamic: static topics
are merged in alongside dynamic ones so a chain that crosses both (e.g. a
dynamic `map → base_link` plus a static `base_link → lidar`) resolves at every
step. The view is the same interactive pager as [`bagwiz walk`](walk.md).

### Direction convention

Each step shows `lookupTransform(target=<to>, source=<from>)`, identical to
`tf static calc` and to `ros2 run tf2_ros tf2_echo <from> <to>`: the translation
is `<from>`'s origin expressed in the `<to>` frame.

### Usage

```text
bagwiz tf walk <input> <from> <to>
```

`tf walk` requires an interactive terminal (stdin and stdout must be a TTY).

### Positional arguments

| Name    | Description                                                             |
| ------- | ----------------------------------------------------------------------- |
| `input` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`). |
| `from`  | Source frame id.                                                        |
| `to`    | Target frame id.                                                        |

Both `<from>` and `<to>` support TAB completion from the bag's TF frame ids
across **all** TF topics (static + dynamic, since `tf walk` merges them; see
[`bagwiz complete`](complete.md)). This is broader than `tf static calc`, which
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
not classify transforms). The `transform:` line lists the full frame chain
resolved at that step, not just the endpoints:

```text
timestamp: 2026-01-01 12:00:00.000000000 UTC (1767268800.000000000)

transform: base_link -> sensor_kit_base_link -> lidar
  translation:
    x: -0.000000
    y: 1.000000
    z: -0.500000
  rotation:
    quaternion:
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
