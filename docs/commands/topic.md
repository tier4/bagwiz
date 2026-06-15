# `bagwiz topic`

Topic-level bag surgery. Subcommands:

| Subcommand                   | What it does                                          |
| ---------------------------- | ----------------------------------------------------- |
| [`drop`](#bagwiz-topic-drop) | Remove selected topics, copying every other verbatim. |
| [`keep`](#bagwiz-topic-keep) | Keep only the selected topics, dropping every other.  |

---

## `bagwiz topic drop`

Remove one or more topics from a rosbag. Every removed topic disappears
entirely from the output — both its messages and its topic declaration /
metadata. All other topics are copied verbatim; no deserialization or type
conversion is performed.

### Usage

```text
bagwiz topic drop [OPTIONS] <input> <topics>...
```

### Positional arguments

| Name     | Description                                                          |
| -------- | -------------------------------------------------------------------- |
| `input`  | Input ROS 2 rosbag (directory or single-file). Must exist.           |
| `topics` | One or more topic selectors to remove (see Selectors). At least one. |

### Options

| Flag                 | Description                                                                                           |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| `-o`, `--output <p>` | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `--overwrite`        | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

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

### Behavior

- Selectors are resolved against the bag's topic list before anything is
  written. A selector that matches no topic stops the run with a clear error
  and leaves the input untouched — this catches typo'd names instead of
  silently rewriting a bag with nothing removed.
- When the selectors together match every topic, the run still succeeds and
  produces a bag with no topics, after logging a warning.
- In-place vs `-o`. Without `-o`, `<input>` is rewritten via an atomic
  tmp-swap that preserves its storage format and layout (the input is both
  source and destination). With `-o`, `<input>` is left untouched and the
  result is written to that path; the output's storage follows the output
  extension (`.mcap` / `.db3` pick a single-file backend) or, for a directory
  output, inherits the input bag's storage backend.
- Embedded message schemas are preserved for the surviving topics so MCAP
  outputs stay self-describing.
- The output MCAP is written with `compression=none`. Re-compress afterwards
  with `ros2 bag convert` if needed.

### Example

```bash
# Drop a single topic, rewriting the bag in place.
bagwiz topic drop drive.mcap /sensing/lidar

# Drop an entire subtree to a new bag, leaving the input untouched.
bagwiz topic drop drive.mcap '/sensing/*' -o drive_trimmed.mcap

# Drop several topics at once (mix literals and globs).
bagwiz topic drop drive_dir/ /tf_static '*/image_raw' -o trimmed_dir/
```

Quote globs (e.g. `'/sensing/*'`) so the shell does not expand them as
filename patterns before bagwiz sees them.

---

## `bagwiz topic keep`

The inverse of [`drop`](#bagwiz-topic-drop): keep **only** the selected topics
and drop every other one. Each dropped topic disappears entirely from the output
— both its messages and its topic declaration / metadata. The kept topics are
copied verbatim; no deserialization or type conversion is performed.

The interface is identical to `drop` — same positional arguments, same options,
same selector syntax, same in-place / `-o` behavior. Only the sense of the
selection is flipped: `drop` removes what matches; `keep` removes what does not.

### Usage

```text
bagwiz topic keep [OPTIONS] <input> <topics>...
```

### Positional arguments

| Name     | Description                                                        |
| -------- | ------------------------------------------------------------------ |
| `input`  | Input ROS 2 rosbag (directory or single-file). Must exist.         |
| `topics` | One or more topic selectors to keep (see Selectors). At least one. |

### Options

| Flag                 | Description                                                                                           |
| -------------------- | ----------------------------------------------------------------------------------------------------- |
| `-o`, `--output <p>` | Write the result to a new bag instead of rewriting `<input>` in place.                                |
| `--overwrite`        | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place. |

### Selectors

Identical to `drop` — see [Selectors](#selectors) above. A selector matches the
topics to **keep**; everything else is dropped. For example, `/sensing/*` keeps
the entire `/sensing` subtree and drops all other topics.

### Behavior

- Selectors are resolved against the bag's topic list before anything is
  written. A selector that matches no topic stops the run with a clear error
  and leaves the input untouched — this catches typo'd names instead of
  silently producing a near-empty bag.
- When the selectors together match every topic, the run still succeeds and
  keeps every topic (nothing is removed), after logging a warning.
- In-place vs `-o`, embedded-schema preservation, and `compression=none` output
  all behave exactly as documented for `drop` above.

### Example

```bash
# Keep a single topic, rewriting the bag in place (everything else is dropped).
bagwiz topic keep drive.mcap /sensing/lidar

# Keep an entire subtree to a new bag, leaving the input untouched.
bagwiz topic keep drive.mcap '/sensing/*' -o drive_sensing_only.mcap

# Keep several topics at once (mix literals and globs).
bagwiz topic keep drive_dir/ /tf_static '*/image_raw' -o trimmed_dir/
```

Quote globs (e.g. `'/sensing/*'`) so the shell does not expand them as
filename patterns before bagwiz sees them.

## Exit status

| Code | Meaning                                                                                                                                                                                                                        |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `0`  | The bag was rewritten successfully, including the all-topics-matched edge case (an empty bag for `drop`, an unchanged topic set for `keep`), which logs a warning.                                                             |
| `1`  | The input could not be opened, a selector matched no topic, the `-o` output path collided without `--overwrite`, the input storage format could not be detected for an in-place rewrite, or a read/write/close error occurred. |
