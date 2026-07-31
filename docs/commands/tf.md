# `bagwiz tf`

TF inspection and static-TF editing on a ROS 2 rosbag.

- [`tree`](#bagwiz-tf-tree) — merge one or more `tf2_msgs/msg/TFMessage` topics into one TF frame tree, colored by static vs dynamic (`static` / `dynamic` selectors supported).
- [`static calc`](#bagwiz-tf-static-calc) — resolve the pose of `--of` expressed in `--ref` using only the bag's static TF tree; print translation/quaternion/RPY or JSON. (`static` is a command group; `calc` is its action.)
- [`static cp`](#bagwiz-tf-static-cp) — copy every static TF topic from `<src>` into `<dst>` (in place, or to a new bag via `-o`), preserving topic names and stamping each at `<dst>`'s start time.
- [`static dump`](#bagwiz-tf-static-dump) — write the bag's static TF tree as nested `parent: child: {x, y, z, roll, pitch, yaw}` YAML (RPY in radians) to `-o`, or to stdout.
- [`static join`](#bagwiz-tf-static-join) — the inverse of `static dump`: embed such a YAML into the bag as one latched `/tf_static` message stamped at the bag's start time.

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

### `--force` vs `-w`, `--overwrite`

Two separate permissions, as on [`static join`](#bagwiz-tf-static-join) and
[`traj join`](traj.md#bagwiz-traj-join):

- `--force` — `<dst>` already contains a static topic whose name collides with one
  being copied. Its existing messages are dropped and replaced by `<src>`'s.
- `-w`, `--overwrite` — the `-o <output>` path already exists; it is replaced. No
  effect in in-place mode, where `<dst>` is the target by definition.

Without the matching flag, either conflict aborts the run with an explanatory error
and leaves `<dst>` (and any existing output) untouched. Neither flag stands in for
the other: clearing an output path does not also authorise replacing a bag's real
static TF. A collision with a destination topic of a **different** message type is
always an error, regardless of `--force`.

### Usage

```text
bagwiz tf static cp --src <src> --dst <dst> [-o <output>] [--force] [-w|--overwrite]
```

`<src>` is read; `<dst>` (or `<output>`) is the write target.

### Options

| Flag                | Description                                                                       |
| ------------------- | --------------------------------------------------------------------------------- |
| `--src <src>`       | Source rosbag to copy static TF from (rosbag2 directory, `*.mcap`, `*.db3`, ...). |
| `--dst <dst>`       | Destination rosbag to copy static TF into (rewritten in place unless `-o`).       |
| `-o`, `--output`    | Write the result to this new bag instead of rewriting `<dst>` in place.           |
| `--force`           | Replace the messages of a colliding static topic in `<dst>`.                      |
| `-w`, `--overwrite` | Replace an existing `-o`/`--output` path. No effect in in-place mode.             |

### Examples

```bash
bagwiz tf static cp --src donor.mcap --dst target.mcap              # rewrite target.mcap in place
bagwiz tf static cp --src donor.mcap --dst target.mcap -o merged.mcap  # write a new bag
bagwiz tf static cp --src donor.mcap --dst target.mcap --force  # replace a colliding /tf_static
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                      |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `0`  | Static TF copied; `<dst>` rewritten or `<output>` written.                                                                                                                                                                                                   |
| `1`  | A bag could not be opened, `<src>` has no static TF topic carrying transforms, a decode/serialize failure, an unresolved conflict (a colliding topic without `--force`, an existing output without `-w`/`--overwrite`, or a type mismatch), or an I/O error. |

---

## `bagwiz tf static dump`

Writes the bag's static TF tree (every `tf2_msgs/msg/TFMessage` topic whose name
ends with `tf_static`) as the nested `parent: child: {x, y, z, roll, pitch, yaw}`
YAML that static-transform publisher configs use — the inverse of reading such a
config and broadcasting it. Dynamic `/tf` topics are ignored.

The output is a config you can hand back to a static-transform publisher, which
makes this the way to recover a recorded rig's calibration from a bag, or to diff
a bag against the config it was supposedly recorded with.

```yaml
base_link:
  drs_base_link:
    x: 0.796
    y: 0.0
    z: 1.826
    roll: 0.0
    pitch: 0.0
    yaw: 0.0

drs_base_link:
  lidar_left:
    x: -0.002254
    y: 0.508026
    z: 0.013543
    roll: 0.005816
    pitch: 0.018911
    yaw: 1.574117
```

### Rotation convention

Rotations are **roll/pitch/yaw in radians**, in tf2's fixed-axis convention —
what `tf2::Matrix3x3::getRPY` produces and `tf2::Quaternion::setRPY(roll, pitch,
yaw)` consumes. Feeding a dumped value back through `setRPY` reproduces the
quaternion the bag carried.

### Precision

Numbers carry 14 significant digits and always show a decimal point (`0.0`, never
`0`, so a consumer that demands a float does not trip over an integer). This is
deliberately not a bit-exact copy: converting a quaternion back to RPY costs a
few ULP, which a full-precision rendering would expose as
`roll: -0.0027009999999999795` where the calibration said `-0.002701`. 14 digits
folds that away, at a cost of ~1e-14 relative error — far below what any
calibration resolves. Use [`tf static calc --json`](#bagwiz-tf-static-calc) for
the full-precision view of a single transform.

Angles are additionally snapped to `0.0` below 1e-12 rad. Relative precision
cannot clean up a component whose true value is zero, and recovering RPY from a
quaternion cannot hold an exact zero beside a right angle — the
`camera_link → camera_optical_link` rotation comes back with
`pitch: -5.55e-17`. The floor sits three orders above that noise and six below the
microradian any real calibration resolves, so it only ever erases noise. It is not
applied to translations, which never pass through this conversion.

### Which messages are read

Only the **first message** of each static topic. Static TF is latched: a
broadcaster sends its whole set in one message and republishes that same set (so
that each split file of a long recording carries it), so the first message is the
complete tree and the rest of the bag is skipped. This keeps the command fast on
large bags.

The consequence is that an edge introduced only by a _later_ message is not
dumped, which happens when several broadcasters publish disjoint subsets to one
topic. Compare against [`tf tree -t static`](#bagwiz-tf-tree), which reads the
whole topic, if you suspect that.

### Merging and dropped data

- **All static topics merge into one tree.** The schema has no topic dimension,
  so `/tf_static` and `/sensing/tf_static` fuse. Two topics naming the same
  parent for a child is fine and collapses to one entry; two topics giving one
  child **different** parents is a contradiction one tree cannot hold, and the
  run aborts with an error naming both topics and both parents. This matches the
  merge-and-detect-conflicts behavior of [`tf tree`](#bagwiz-tf-tree) and
  [`tf static calc`](#bagwiz-tf-static-calc).
- **`header.stamp` is dropped.** The schema has nowhere to put it, and a static
  transform's stamp carries no information a config needs.
- Everything else in the bag's static TF is written, including
  `camera_link → camera_optical_link` edges that a publisher may be configured to
  regenerate itself. A dump does not silently discard bag content.

### Ordering

Parent groups are ordered breadth-first from the tree's roots (a parent frame
that is never a child), children in first-seen order within a parent, so the base
frame heads the file and it reads top-down. A parent unreachable from any root —
only possible for a cyclic input, which a valid TF tree never is — is written
after the reachable ones, so no transform is ever lost.

### Frame ids

Frame ids come from the bag, so they are not assumed safe. A name outside a
conservative plain-scalar set, or one a YAML reader would resolve as a bool or
null rather than a string (`no`, `y`, `true`, `null`), is emitted as an escaped
double-quoted scalar. Ordinary ROS frame ids (`base_link`,
`camera0/camera_link`) stay unquoted.

### Usage

```text
bagwiz tf static dump -i <input> [-o <output>] [-w|--overwrite]
```

### Options

| Flag                    | Description                                                                            |
| ----------------------- | -------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                |
| `-o`, `--output <path>` | Write the YAML to this file. When omitted, it goes to stdout.                          |
| `-w`, `--overwrite`     | Replace an existing `-o`/`--output` path. Without it, an existing path aborts the run. |

Without `-o` the YAML is written to stdout and every diagnostic to stderr, so
`bagwiz tf static dump -i <bag> > tf_static.yaml` is pipe-clean. The output path
is claimed only after the read succeeds, so a bag with no static TF cannot
destroy an existing `-o` file under `-w`/`--overwrite`.

### Examples

```bash
bagwiz tf static dump -i capture.mcap                          # print to stdout
bagwiz tf static dump -i capture.mcap -o tf_static.yaml        # write a file
bagwiz tf static dump -i capture.mcap -o tf_static.yaml -w     # replace it
bagwiz tf static dump -i capture.mcap > tf_static.yaml         # equivalent to -o
```

### Exit status

| Code | Meaning                                                                                                                                                                                                         |
| ---- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Static TF tree written to `<output>` or stdout.                                                                                                                                                                 |
| `1`  | Bag could not be opened, has no static TF topic carrying transforms, a decode failure, two static topics giving one child different parents, an existing `-o` path without `-w`/`--overwrite`, or an I/O error. |

---

## `bagwiz tf static join`

The inverse of [`static dump`](#bagwiz-tf-static-dump): reads a static-transform
publisher config — the nested `parent: child: {x, y, z, roll, pitch, yaw}` YAML,
rotations as RPY in radians — and embeds it into the bag as a single latched
`tf2_msgs/msg/TFMessage`.

Together the two close the loop: `dump` recovers a config from a recorded rig, and
`join` puts a config into a bag that is missing its static TF, or replaces one that
is wrong. A bag trimmed to start after `/tf_static` was last published, for
instance, has no static tree at all until you join one back in.

### Rotation convention

The YAML's `roll`/`pitch`/`yaw` are radians in tf2's fixed-axis convention and are
converted to a quaternion with `tf2::Quaternion::setRPY` — exactly what a
static-transform publisher does with the same file, and the inverse of the
`getRPY` that [`static dump`](#bagwiz-tf-static-dump) applies.

So `dump` → `join` reproduces the bag it came from: **translations exactly**, and
rotations to within the precision `dump` writes (see its
[Precision](#precision) section). Measured over a 21-edge vehicle rig, the worst
rotation deviation was 2.5e-15 rad — a picometre over a 100 m lever arm. And
`dump` → `join` → `dump` is byte-identical, so a config survives any number of
trips through a bag unchanged.

### Nesting

A mapping that carries the six transform keys is an edge from the key enclosing
it; one that does not is a further level. Nesting may therefore go **arbitrarily
deep**, matching `multi_transform_publisher`, so any config that works with the
publisher works here.

Depth beyond two is **not a chain** — it is a grouping heading, which is how a
large rig config gets split into sections:

```yaml
sensors: # a heading: parents nothing
  base_link:
    drs_base_link:
      x: 0.796
      # ... => base_link -> drs_base_link
  drs_base_link:
    lidar_front:
      # ... => drs_base_link -> lidar_front
```

Only the level immediately above a transform names its parent. Because an author
could instead have meant `a: {b: {c: {...}}}` as the chain `a → b → c` (it is
`b → c`, with `a` a heading), `join` prints a warning naming every key that turned
out to parent nothing. The two-level form [`static dump`](#bagwiz-tf-static-dump)
writes has no headings and warns about nothing.

### Accepted input

Otherwise strict, because this is a hand-edited file and a silently-ignored key
becomes a silently-wrong sensor pose. A transform must carry **exactly** the six
keys `x`, `y`, `z`, `roll`, `pitch`, `yaw` with numeric values. Rejected, with the
offending frame or key named:

- A missing key. There is no default: a pose missing `pitch` is underspecified,
  and filling in `0` would invent a transform the author did not write.
- Any other key. A key name misspelled by a letter would otherwise leave that axis
  silently at `0`.
  This is also what catches a child nested _beside_ the six keys — the publisher
  reads the six and drops that child's transform without a word, so here `join` is
  deliberately stricter than the publisher.
- A non-numeric value, an empty frame id, a value that is neither a transform nor
  child frames, an empty mapping, or a frame that is its own parent.
- **A transform at the document root**: there is no enclosing key to be its
  parent, i.e. the parent frame was forgotten. (`multi_transform_publisher`
  broadcasts this with an empty parent frame id.)
- An empty document — there would be nothing to write.
- Nesting deeper than 32 levels, a guard against a pathological document; no
  hand-written config comes close.

Finally the parsed transforms are checked to be a **buildable tf tree**, since a
file can parse cleanly and still be unusable. This is the same
`core::validate_tf_tree` any bagwiz code writing transforms can call, and it
rejects:

- **A child claimed by two parents, both `A → B` and `B → A`, or a cycle** — the
  same forest validation [`tf tree`](#bagwiz-tf-tree) applies to a bag's merged
  tree.
- **A non-finite value.** `.nan` and `.inf` are valid YAML floats, so they parse
  happily, but `tf2::BufferCore` _drops_ such a transform (logging
  `TF_NAN_INPUT`). Without this check `join` would write a perfectly well-formed
  `/tf_static` whose tree is empty the moment anything used it — and `tf tree`
  would still draw it, since that reads the raw edges.
- **A rotation that is not unit length.** tf2 does _not_ reject this one: it keeps
  the quaternion, and `tf2::Matrix3x3` builds its matrix from the raw components
  without normalising, so the transform comes out skewed. Silently wrong geometry
  is worse than a missing frame. The tolerance (1e-6 on the squared length) passes
  a quaternion that was stored as float32 and widened back.
- Anything else tf2 itself refuses, and any frame that does not resolve against
  its own tree root once loaded.

Several roots — a forest rather than one connected tree — is **accepted**, as it is
by [`tf tree`](#bagwiz-tf-tree) and by ROS itself. A partial config can be
completed by TF the bag already carries, so a frame is only ever required to
resolve within its own tree.

Note also that unlike `multi_transform_publisher`, `join` does **not** synthesize
`camera_link → camera_optical_link` edges. It writes exactly the transforms the
file declares; if you want those edges in the bag, put them in the file (which is
what `static dump` produces, since it reads them from the bag).

### Timestamp

The message is stamped at `<input>`'s earliest message time — both the message's
receive time and the `header.stamp` of every transform it carries. That places the
latched static TF at the very start of the timeline, where a static transform is
expected to already hold. It is also written ahead of the copied messages, so its
storage position agrees with its timestamp: a consumer that reads a `.db3` in row
order rather than by timestamp (Foxglove's readers issue their message query
without an `ORDER BY`) still receives it first.

### Output modes

- Default (no `-o`): `<input>` is rewritten in place via an atomic tmp-swap that
  preserves its storage format and layout. If the pass fails, `<input>` is left
  untouched.
- `-o <output>`: `<input>` is left untouched and the result (its messages plus the
  embedded static TF) is written to `<output>`. The storage format and layout
  follow `<output>`: a `.mcap` or `.db3` extension picks that single-file backend,
  and any other path produces a **directory-layout MCAP** bag.

### `--force` vs `-w`, `--overwrite`

Two separate permissions, matching [`bagwiz traj join`](traj.md#bagwiz-traj-join)
rather than [`static cp`](#bagwiz-tf-static-cp)'s combined flag:

- `--force` — `<topic>` already carries messages in `<input>`. Its existing
  messages are dropped and replaced by the config's. Without it, this aborts:
  silently replacing a bag's real static TF with a config would be
  unrecoverable. A collision with a topic of a **different** message type is
  always an error, `--force` or not.
- `-w`, `--overwrite` — the `-o <output>` path already exists; it is replaced. No
  effect in in-place mode, where `<input>` is the target by definition.

### Topic

`-t`/`--topic` defaults to `/tf_static`, the name a static transform broadcaster
publishes under. The YAML carries no topic name, so a default is needed; pass
`-t` to write e.g. `/sensing/tf_static` instead. A name that does not end with
`tf_static` is accepted but warns, because every bagwiz static-TF reader
(`tf static dump`, `tf static calc`, `tf tree -t static`, `tf static cp`) selects
topics by that suffix and would treat the topic as dynamic.

### Usage

```text
bagwiz tf static join -i <input> --yaml <file> [-t <topic>] [-o <output>] [--force] [-w|--overwrite]
```

### Options

| Flag                    | Default      | Description                                                                                 |
| ----------------------- | ------------ | ------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | _(required)_ | ROS 2 rosbag path (rosbag2 directory, `*.mcap`, `*.db3`, `*.db3.zstd`).                     |
| `--yaml <file>`         | _(required)_ | Static TF YAML to embed, in the schema `tf static dump` writes.                             |
| `-t`, `--topic <topic>` | `/tf_static` | Topic to embed the transforms under.                                                        |
| `-o`, `--output <OUT>`  | _(unset)_    | Write the result to this new bag instead of rewriting `<input>` in place.                   |
| `--force`               | `false`      | Replace `<topic>`'s existing messages in `<input>`; otherwise a populated `<topic>` aborts. |
| `-w`, `--overwrite`     | `false`      | Replace an existing `-o`/`--output` path. No effect in in-place mode.                       |

### Examples

```bash
# Rewrite capture.mcap in place, embedding the config on /tf_static.
bagwiz tf static join -i capture.mcap --yaml multi_tf_static.yaml

# Write a new bag instead of touching the input.
bagwiz tf static join -i capture.mcap --yaml multi_tf_static.yaml -o with_tf.mcap

# Replace a /tf_static the bag already carries.
bagwiz tf static join -i capture.mcap --yaml multi_tf_static.yaml --force

# Embed under a different static topic.
bagwiz tf static join -i capture.mcap --yaml sensing.yaml -t /sensing/tf_static

# Round trip: recover a rig's config from one bag, put it into another.
bagwiz tf static dump -i donor.mcap -o rig.yaml
bagwiz tf static join -i target.mcap --yaml rig.yaml
```

### Exit status

| Code | Meaning                                                                                                                                                                                                                                                                |
| ---- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Static TF embedded; `<input>` rewritten or `<output>` written.                                                                                                                                                                                                         |
| `1`  | The YAML could not be read or was rejected (see [Accepted input](#accepted-input)), the bag could not be opened, `<topic>` is populated without `--force` or has another type, an existing `-o` path without `-w`/`--overwrite`, a serialize failure, or an I/O error. |

## Migration

**`tf static cp`: replacing a colliding topic is now `--force`, not `-w`.** The
single `-w`/`--overwrite` that used to permit both conflicts has been split, so
`cp` matches [`static join`](#bagwiz-tf-static-join) and
[`traj join`](traj.md#bagwiz-traj-join): `--force` covers a colliding static topic
in `<dst>`, and `-w`/`--overwrite` now covers only an existing `-o` path. Neither
stands in for the other, so clearing an output path no longer also authorises
replacing a bag's real static TF.

```bash
# before: -w permitted both
bagwiz tf static cp --src donor.mcap --dst target.mcap -w

# after: name the conflict being permitted
bagwiz tf static cp --src donor.mcap --dst target.mcap --force            # colliding topic
bagwiz tf static cp --src donor.mcap --dst target.mcap -o out.mcap -w     # existing output
bagwiz tf static cp --src donor.mcap --dst target.mcap -o out.mcap --force -w  # both
```

`tf walk` was removed. `tf static calc` covers the static-tree query; there is
no in-tree replacement for stepping through a dynamic TF timeline.

The bag operand is now `-i` / `--input` on every `tf` subcommand.
`tf static cp` operands are `--src` and `--dst`, long-form only: the `-s` / `-d`
short forms they briefly carried have been removed.
The frame operands on `tf static calc` have long been `--of` / `--ref`; the only
change is that the bag is no longer positional:

```bash
bagwiz tf tree capture.mcap                           # before — now an error
bagwiz tf tree -i capture.mcap                      # after

bagwiz tf static calc capture.mcap --of base_link --ref lidar   # before — now an error
bagwiz tf static calc -i capture.mcap --of base_link --ref lidar # after

bagwiz tf static cp donor.mcap target.mcap                # before — now an error
bagwiz tf static cp -s donor.mcap -d target.mcap          # before — now an error
bagwiz tf static cp --src donor.mcap --dst target.mcap    # after
```
