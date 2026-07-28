# `bagwiz tf`

TF inspection and static-TF editing on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merge one or more `tf2_msgs/msg/TFMessage` topics into one TF frame tree, colored by static vs dynamic (`static` / `dynamic` selectors supported).
- [`static calc`](#bagwiz-tf-static-calc) — resolve the pose of `--of` expressed in `--ref` using only the bag's static TF tree; print translation/quaternion/RPY or JSON. (`static` is a command group; `calc` is its action.)
- [`static cp`](#bagwiz-tf-static-cp) — copy every static TF topic from `<src>` into `<dst>` (in place, or to a new bag via `-o`), preserving topic names and stamping each at `<dst>`'s start time.
- [`walk`](#bagwiz-tf-walk) — merge every TF topic into one buffer and step interactively through the times the merged TF changed, showing `--of`'s pose in `--ref` at each.

ROS 1 `*.bag` inputs are not supported.

---

## `bagwiz tf tree`

Merges one or more `tf2_msgs/msg/TFMessage` topics (`-t`/`--topics`) into a
single TF frame tree built from the union of their distinct parent→child
edges. In the merged tree each edge is colored by whether it came from a
**static** (`*tf_static`) or a **dynamic** topic. When the tree contains both
kinds, a legend is printed and each child frame is colored and tagged `[S]`
(static) or `[D]` (dynamic); when only one kind is present the tree is drawn
plain and the header names the category, e.g. `═══ TF tree (static) ═══`.

`-t`/`--topics` accepts two reserved selectors in addition to literal topic
names:

- `static` — expands to **every** `*tf_static` topic in the bag.
- `dynamic` — expands to every non-static TF topic in the bag.

They compose with each other and with literal names. For example
`tf tree -i bag -t dynamic /extra_tf` merges all dynamic TF topics plus
`/extra_tf`, and `tf tree -i bag -t static` shows the merged static tree alone.
(ROS topic names start with `/`, so the bare words `static` / `dynamic` never
collide with a real topic.) When `-t`/`--topics` is omitted, bagwiz defaults to **every**
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

<!-- AUTO-GENERATED: bagwiz tf tree print order (sync with `run_tree` in `bagwiz/src/commands/tf.cpp`) -->

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

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `bagwiz/src/commands/tf.cpp`) -->

- On a color-capable TTY (and when `NO_COLOR` is unset) section headers and root
  lines are bold and branch glyphs are dim gray. In a mixed tree, dynamic edges
  are bright cyan and static edges bright yellow; a single-category tree uses
  the terminal's default color.
- The `[S]` / `[D]` tags always print in a mixed tree, so the category stays
  identifiable under `NO_COLOR` or when piped to a file.
- `├──` / `└──` / `│` box drawing is the default. Set `BAGWIZ_TF_TREE_ASCII=1`
  to use `|--` / `` `-- `` / `|` instead, and to drop the `●` root prefix so each
  root line prints as the bare frame name (see [Environment](#environment)).

### Environment

<!-- AUTO-GENERATED: `tf tree` / terminal styling (sync with `stdout_use_color`, `make_tree_glyphs` in `bagwiz/src/commands/tf.cpp`) -->

- `NO_COLOR`: if set to any value, disables ANSI colors on `tf tree`. The `[S]` / `[D]` category tags are still printed.
- `BAGWIZ_TF_TREE_ASCII`: if set to any value, uses ASCII branch glyphs instead of Unicode box drawing (see `make_tree_glyphs` in `bagwiz/src/commands/tf.cpp`).

Colors are also omitted when stdout is not a TTY (same effect as `NO_COLOR` for styling).

### Usage

```text
bagwiz tf tree -i <input> [-t|--topics <topic-or-selector>...]
```

### Options

| Flag                    | Description                                                                                                                                                                                       |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                                                                                                                           |
| `-t`, `--topics <t>...` | Zero or more `tf2_msgs/msg/TFMessage` topics and/or the selectors `static` / `dynamic` (e.g. `/tf /tf_static`, `static`, `dynamic /extra_tf`). When omitted, all TF topics in the bag are merged. |

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
bagwiz tf tree -i capture.mcap                       # merge every TF topic in the bag
bagwiz tf tree -i capture.mcap -t static             # only the static (*tf_static) tree
bagwiz tf tree -i capture.mcap -t dynamic            # only the dynamic tree
bagwiz tf tree -i capture.mcap -t dynamic /extra_tf  # all dynamic topics + /extra_tf
bagwiz tf tree -i capture.mcap -t /tf /tf_static     # explicit merge
```

Single-category output, e.g. `tf tree -i capture.mcap -t dynamic` (plain):

```text
═══ TF tree (dynamic) ═══
● map
└── odom
    └── base_link
```

Mixed output for `tf tree -i capture.mcap -t /tf /tf_static`, where `map → base_link`
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

Resolves the pose of `--of` expressed in the `--ref` frame using **only** the
bag's static TF (topics whose name ends with `tf_static`). Dynamic `/tf` topics
are intentionally ignored. The transform is composed across the whole static
chain, so `--of` and `--ref` need not be directly adjacent — any two frames
connected through the static tree work. The printed `transform:` line names the
two endpoints; the `chain:` line below it lists the full resolved chain (every
intermediate frame joined with `->`), not just the two endpoints.

When the bag has **several** static topics (e.g. `/tf_static` and
`/sensing/tf_static`), they are all merged into one static tree. The merge is
rejected if the topics disagree: the command exits with an error when the same
`child_frame_id` is given a different parent by two different topics. Two topics
declaring the **same** edge (same parent) are fine. This matches the merge-and-
detect-conflicts behavior of [`bagwiz traj dump`](traj.md).

### Direction convention

The printed transform is the **pose of `--of` expressed in the `--ref` frame** —
`lookupTransform(target=<ref>, source=<of>)`, whose translation is `<of>`'s
origin in `<ref>`. Swapping the two flags yields the inverse transform.

This is equivalent to:

```bash
ros2 run tf2_ros tf2_echo <ref> <of>
```

Note the operand order: `tf2_echo` takes the **reference frame first**, so its
arguments are the reverse of the `--of` / `--ref` reading order.

### Usage

```text
bagwiz tf static calc -i <input> --of <frame> --ref <frame> [--json]
```

### Options

| Flag                    | Description                                                             |
| ----------------------- | ----------------------------------------------------------------------- |
| `-i`, `--input <input>` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`). |
| `--of <frame>`          | Frame whose pose is resolved (`<of>`).                                  |
| `--ref <frame>`         | Reference frame the pose is expressed in (`<ref>`).                     |
| `--json`                | Emit the transform as JSON instead of human text.                       |

`--of` and `--ref` support TAB completion. Because `tf static calc` resolves
only the static tree, the candidates are restricted to frame ids found in the
bag's static `*tf_static` topics (see [`bagwiz complete`](complete.md)).

### Output

Human form (monochrome, like `tf2_echo`). The `transform:` line names the two
endpoints as `of=<of>  ref=<ref>`; the `chain:` line below it lists the full
resolved frame chain from `<of>` to `<ref>` (here `base_link` reaches `lidar`
through `sensor_kit_base_link`):

```text
transform: of=base_link  ref=lidar  (static)
  chain: base_link -> sensor_kit_base_link -> lidar
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
JSON carries only the `of` / `ref` endpoints, not the intermediate chain.
Object keys are emitted in alphabetical order (nlohmann's default), so
consumers should not rely on key ordering:

```json
{
  "of": "base_link",
  "ref": "lidar",
  "rotation": {
    "quaternion": {
      "w": 0.7071067811865476,
      "x": 0.0,
      "y": 0.0,
      "z": -0.7071067811865475
    },
    "rpy_deg": {
      "pitch": 0.0,
      "roll": 0.0,
      "yaw": -89.99999999999999
    },
    "rpy_rad": {
      "pitch": 0.0,
      "roll": 0.0,
      "yaw": -1.5707963267948963
    }
  },
  "translation": {
    "x": 0.0,
    "y": 1.0,
    "z": -0.5
  }
}
```

### Examples

```bash
bagwiz tf static calc -i capture.mcap --of base_link --ref lidar
bagwiz tf static calc -i capture.mcap --of base_link --ref lidar --json
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
  layout follow `<output>`: a `.mcap` or `.db3` extension picks that
  single-file backend, and any other path produces a **directory-layout MCAP**
  bag — a directory output does not inherit `<dst>`'s storage backend.

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
bagwiz tf static cp --src <src> --dst <dst> [-o <output>] [-w|--overwrite]
```

`<src>` is read; `<dst>` (or `<output>`) is the write target.

### Options

| Flag                | Description                                                                                             |
| ------------------- | ------------------------------------------------------------------------------------------------------- |
| `--src <src>`       | Source rosbag to copy static TF from (rosbag2 directory, `*.mcap`, `*.db3`, ...).                       |
| `--dst <dst>`       | Destination rosbag to copy static TF into (rewritten in place unless `-o`).                             |
| `-o`, `--output`    | Write the result to this new bag instead of rewriting `<dst>` in place.                                 |
| `-w`, `--overwrite` | Replace an existing `-o`/`--output` path, and replace any colliding static topic's messages in `<dst>`. |

### Examples

```bash
bagwiz tf static cp --src donor.mcap --dst target.mcap              # rewrite target.mcap in place
bagwiz tf static cp --src donor.mcap --dst target.mcap -o merged.mcap  # write a new bag
bagwiz tf static cp --src donor.mcap --dst target.mcap -w  # replace a colliding /tf_static
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
times at which the merged TF changed — one step per timestamp — resolving
`--of`'s pose in `--ref` at each. Unlike [`tf static calc`](#bagwiz-tf-static-calc),
`tf walk` does **not** classify transforms as static vs dynamic: static topics
are merged in alongside dynamic ones so a chain that crosses both (e.g. a
dynamic `map → base_link` plus a static `base_link → lidar`) resolves at every
step. The view is the same interactive pager as [`bagwiz walk`](walk.md).

### Direction convention

Each step shows the **pose of `--of` expressed in the `--ref` frame** —
`lookupTransform(target=<ref>, source=<of>)`, identical to `tf static calc` and
to `ros2 run tf2_ros tf2_echo <ref> <of>`: the translation is `<of>`'s origin
expressed in the `<ref>` frame.

### Usage

```text
bagwiz tf walk -i <input> --of <frame> --ref <frame>
```

`tf walk` requires an interactive terminal (stdin and stdout must be a TTY).

### Options

| Flag                    | Description                                                             |
| ----------------------- | ----------------------------------------------------------------------- |
| `-i`, `--input <input>` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`). |
| `--of <frame>`          | Frame whose pose is resolved (`<of>`).                                  |
| `--ref <frame>`         | Reference frame the pose is expressed in (`<ref>`).                     |

Both `--of` and `--ref` support TAB completion from the bag's TF frame ids
across **all** TF topics (static + dynamic, since `tf walk` merges them; see
[`bagwiz complete`](complete.md)). This is broader than `tf static calc`, which
restricts the same slots to static `*tf_static` frames.

### Keys

| Key                                     | Action                                    |
| --------------------------------------- | ----------------------------------------- |
| `→` / `Space`                           | next timestamp (wraps from last to first) |
| `←` / `b`                               | previous timestamp                        |
| `↑` / `k`, `↓` / `j`                    | scroll the transform body up / down       |
| `Home` / `H`, `End` / `T`               | jump the body scroll to head / tail       |
| `g` / `G`                               | jump to the first / last timestamp        |
| `q` / `Q` / `Esc` / `Ctrl-C` / `Ctrl-D` | quit                                      |

### Output

Per step, the header shows the timestamp and the body shows the resolved
transform (monochrome, like `tf2_echo`; no `(static)` tag since the walk does
not classify transforms). The `transform:` line names the two endpoints as
`of=<of>  ref=<ref>`; the `chain:` line below it lists the full frame chain
resolved at that step, not just the endpoints:

```text
timestamp: 2026-01-01 12:00:00.000000000 UTC (1767268800.000000000)

transform: of=base_link  ref=lidar
  chain: base_link -> sensor_kit_base_link -> lidar
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

When the merged TF cannot connect `--of` and `--ref` at a given timestamp
(e.g. the chain is not yet complete), that step shows a `⚠` warning line with
the tf2 reason instead of a transform, and navigation continues.

### Examples

```bash
bagwiz tf walk -i capture.mcap --of base_link --ref lidar
bagwiz tf walk -i capture.mcap --of map --ref base_link
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                             |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The pager ran and the user quit.                                                                                                                                                                                                                                                    |
| `1`  | Not a TTY, the bag could not be opened, it has no `tf2_msgs/msg/TFMessage` topic, the TF topics carry no decodable transforms, a `--of`/`--ref` frame absent from the bag's merged TF tree (available frames are listed on stderr), an empty frame id, or a TF load/decode failure. |

## Migration

The bag operand is now `-i` / `--input` on every `tf` subcommand.
`tf static cp` operands are `--src` and `--dst`, long-form only: the `-s` / `-d`
short forms they briefly carried have been removed.
The frame operands on `tf static calc` and `tf walk` have long been `--of` /
`--ref`; the only change is that the bag is no longer positional:

```bash
bagwiz tf tree capture.mcap                           # before — now an error
bagwiz tf tree -i capture.mcap                      # after

bagwiz tf static calc capture.mcap --of base_link --ref lidar   # before — now an error
bagwiz tf static calc -i capture.mcap --of base_link --ref lidar # after

bagwiz tf static cp donor.mcap target.mcap                # before — now an error
bagwiz tf static cp -s donor.mcap -d target.mcap          # before — now an error
bagwiz tf static cp --src donor.mcap --dst target.mcap    # after

bagwiz tf walk capture.mcap --of base_link --ref lidar  # before — now an error
bagwiz tf walk -i capture.mcap --of base_link --ref lidar # after
```
