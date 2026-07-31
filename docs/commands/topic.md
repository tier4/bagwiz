# `bagwiz topic`

Topic-level bag surgery. Subcommands:

| Subcommand                       | What it does                                          |
| -------------------------------- | ----------------------------------------------------- |
| [`drop`](#bagwiz-topic-drop)     | Remove selected topics, copying every other verbatim. |
| [`keep`](#bagwiz-topic-keep)     | Keep only the selected topics, dropping every other.  |
| [`rename`](#bagwiz-topic-rename) | Rename one topic, copying every other verbatim.       |

---

## `bagwiz topic drop`

Remove one or more topics from a rosbag. Every removed topic disappears
entirely from the output — both its messages and its topic declaration /
metadata. All other topics are copied verbatim; no deserialization or type
conversion is performed.

### Usage

```text
bagwiz topic drop -i <input> -t <topics>... [OPTIONS]
```

### Examples

```bash
# Drop a single topic, rewriting the bag in place.
bagwiz topic drop -i drive.mcap -t /sensing/lidar

# Drop an entire subtree to a new bag, leaving the input untouched.
bagwiz topic drop -i drive.mcap -t '/sensing/*' -o drive_trimmed.mcap

# Drop several topics at once (mix literals and globs).
bagwiz topic drop -i drive_dir/ -t /tf_static '*/image_raw' -o trimmed_dir/
```

Quote globs (e.g. `'/sensing/*'`) so the shell does not expand them as
filename patterns before bagwiz sees them.

### Options

| Flag                    | Description                                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                              |
| `-t`, `--topics <t>...` | **Required.** One or more topic selectors to remove (see [Selectors](#selectors)). At least one.      |
| `-o`, `--output <p>`    | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### Selectors

Each `<topic>` is either a literal topic name or a glob whose only wildcard is
`*`:

- `*` matches any run of characters, including `/` and the empty string.
- Every other character matches literally, so a selector without `*` is an
  exact topic-name match.

Examples:

| Selector         | Matches                                                      |
| ---------------- | ------------------------------------------------------------ |
| `/sensing/lidar` | exactly the topic `/sensing/lidar`                           |
| `/sensing/*`     | every topic under `/sensing/` (e.g. `/sensing/camera/image`) |
| `*/image_raw`    | any topic ending in `/image_raw`                             |
| `*`              | every topic in the bag                                       |

Because `*` spans `/`, `/sensing/*` removes the entire `/sensing` subtree.

### Selector resolution

- Selectors are resolved against the bag's topic list before anything is
  written. A selector that matches no topic stops the run with a clear error
  and leaves the input untouched — this catches typo'd names instead of
  silently rewriting a bag with nothing removed.
- When the selectors together match every topic, the run still succeeds and
  produces a bag with no topics, after logging a warning.

### In-place vs `-o`

- Without `-o`, `<input>` is rewritten via an atomic
  tmp-swap that preserves its storage format and layout (the input is both
  source and destination). With `-o`, `<input>` is left untouched and the
  result is written to that path; the output's storage follows the output
  extension (`.mcap` / `.db3` pick a single-file backend) or, for a directory
  output, inherits the input bag's storage backend.
- In-place rewriting requires an uncompressed input. A directory bag whose
  `metadata.yaml` declares `compression_mode: file` — including a
  chunk-compressed directory bag produced by an earlier `-o` run of
  `topic`/`trim` — is rejected with `could not detect storage format of input
bag`; pass an explicit `-o` output for those.

### Chunk pass-through

- When both the input and the output are MCAP, chunks the edit does not touch
  are copied byte-for-byte, preserving the input's chunk compression; only
  chunks that still carry both removed and surviving topics are re-encoded
  (with the chunk's own codec); a chunk holding nothing but removed topics is
  dropped whole. A few chunks are also re-encoded for layout reasons unrelated
  to the edit (a missing or untrustworthy chunk message index). When this fast
  path cannot apply — non-MCAP storage, multi-shard inputs, and a few other
  layouts — the bag is re-encoded and the output MCAP is written with
  `compression=none`; re-compress afterwards with `ros2 bag convert` if
  needed.
- Embedded message schemas are preserved for the surviving topics so MCAP
  outputs stay self-describing.
- MCAP attachment and metadata records are not carried into the output. On
  the chunk pass-through path the run logs a warning naming the counts; on
  the decoded fallback path they are dropped silently.

---

## `bagwiz topic keep`

The inverse of [`drop`](#bagwiz-topic-drop): keep **only** the selected topics
and drop every other one. Each dropped topic disappears entirely from the output
— both its messages and its topic declaration / metadata. The kept topics are
copied verbatim; no deserialization or type conversion is performed.

The interface is identical to `drop` — same input flag, same options,
same selector syntax, same in-place / `-o` behavior. Only the sense of the
selection is flipped: `drop` removes what matches; `keep` removes what does not.

### Usage

```text
bagwiz topic keep -i <input> -t <topics>... [OPTIONS]
```

### Examples

```bash
# Keep a single topic, rewriting the bag in place (everything else is dropped).
bagwiz topic keep -i drive.mcap -t /sensing/lidar

# Keep an entire subtree to a new bag, leaving the input untouched.
bagwiz topic keep -i drive.mcap -t '/sensing/*' -o drive_sensing_only.mcap

# Keep several topics at once (mix literals and globs).
bagwiz topic keep -i drive_dir/ -t /tf_static '*/image_raw' -o trimmed_dir/
```

Quote globs (e.g. `'/sensing/*'`) so the shell does not expand them as
filename patterns before bagwiz sees them.

### Options

| Flag                    | Description                                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                              |
| `-t`, `--topics <t>...` | **Required.** One or more topic selectors to keep (see [Selectors](#selectors)). At least one.        |
| `-o`, `--output <p>`    | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### Selectors

Identical to `drop` — see [Selectors](#selectors) above. A selector matches the
topics to **keep**; everything else is dropped. For example, `/sensing/*` keeps
the entire `/sensing` subtree and drops all other topics.

### Selector resolution

- Selectors are resolved against the bag's topic list before anything is
  written. A selector that matches no topic stops the run with a clear error
  and leaves the input untouched — this catches typo'd names instead of
  silently producing a near-empty bag.
- When the selectors together match every topic, the run still succeeds and
  keeps every topic (nothing is removed), after logging a warning.

### In-place vs `-o`

In-place vs `-o`, embedded-schema preservation, and output compression all
behave exactly as documented for [`drop`](#bagwiz-topic-drop) above.

---

## `bagwiz topic rename`

Rename a single topic. The topic named `<src_topic>` is re-declared as
`<dst_topic>` and all of its messages move with it; every other topic is copied
verbatim. Only the name string changes — the topic's message type, QoS, and
embedded schema are preserved, and no deserialization or type conversion is
performed.

Unlike `drop` / `keep`, rename is a strict 1:1 operation: `<src_topic>` and
`<dst_topic>` are **literal topic names, not globs**.

### Usage

```text
bagwiz topic rename -i <input> --src <src_topic> --dst <dst_topic> [OPTIONS]
```

### Examples

```bash
# Rename a topic, rewriting the bag in place.
bagwiz topic rename -i drive.mcap --src /sensing/lidar --dst /sensing/laser

# Rename to a new bag, leaving the input untouched.
bagwiz topic rename -i drive.mcap --src /old/name --dst /new/name -o drive_renamed.mcap

# Rename a topic in a directory bag.
bagwiz topic rename -i drive_dir/ --src /camera/image_raw --dst /camera/front/image_raw
```

### Options

| Flag                    | Description                                                                                           |
| ----------------------- | ----------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>` | **Required.** Input ROS 2 rosbag (directory or single-file). Must exist.                              |
| `--src <src_topic>`     | **Required.** Existing topic to rename. A literal name; must match a topic exactly. Long-form only.   |
| `--dst <dst_topic>`     | **Required.** New name for the topic. A literal name; must not already exist. Long-form only.         |
| `-o`, `--output <p>`    | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `-w`, `--overwrite`     | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### Name validation

The names are resolved against the bag's topic list before anything is
written. The run stops with a clear error and leaves the input untouched when:

- `<src_topic>` matches no topic in the bag (catches a typo'd source);
- `<dst_topic>` already names a topic in the bag — renaming onto it would
  collide two distinct declarations onto one name (and a topic may have only
  one type);
- either `<src_topic>` or `<dst_topic>` is empty (both must be non-empty); or
- `<src_topic>` and `<dst_topic>` are identical (a no-op).

### In-place vs `-o`

In-place vs `-o`, embedded-schema preservation, and output compression all
behave exactly as documented for [`drop`](#bagwiz-topic-drop) above, with the
re-encode trigger read as the _renamed_ topic: chunks carrying `<src_topic>`
are re-encoded (with the same codec) and every other chunk is copied
byte-for-byte.

---

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
