# `bagwiz convert`

Cross-format bag conversion. Three subcommands:

| Subcommand                           | Direction                                      |
| ------------------------------------ | ---------------------------------------------- |
| [`1to2`](#bagwiz-convert-1to2)       | ROS 1 `*.bag` → ROS 2 rosbag2 (MCAP / SQLite3) |
| [`2to1`](#bagwiz-convert-2to1)       | ROS 2 rosbag2 → ROS 1 `*.bag`                  |
| [`storage`](#bagwiz-convert-storage) | ROS 2 rosbag2 repack between MCAP and SQLite3  |

## Common notes

- For `1to2` / `2to1`, the message type to convert no longer has to come
  from a fixed list — see
  [Type and schema resolution](#type-and-schema-resolution-for-1to2--2to1)
  below. Topics whose schema cannot be resolved, whose ROS 2 schema
  cannot be canonicalised to ROS 1 form, or (for `1to2`) whose
  bag-recorded md5sum disagrees with the synthesised ROS 2 schema are
  skipped with a warning, and the run exits with status `2` — see
  [Exit status](#exit-status).
- For `1to2` and `storage`, when `--storage` is omitted the storage
  backend is inferred from the output path's extension (`.mcap` →
  MCAP, `.db3` → SQLite3). Output paths that do not carry one of those
  extensions (e.g. a directory) require an explicit `--storage`.
- Any pre-existing entry at `<output>` (file or directory) stops the
  run with a clear log line. Pass `--overwrite` to replace it instead.
  The flag is supported by every `bagwiz` subcommand that writes a
  file or directory output (`convert 1to2`, `convert 2to1`,
  `convert storage`, `traj dump`, `traj join -o`, `tf inject-static -o`).
- `mcap` outputs are written without chunk compression. Re-compress
  afterwards with `ros2 bag convert` if needed.
- Per-message conversion / write failures are reported as warnings
  (rate-limited to the first 3 per topic) and counted in the per-topic
  summary. A bad message never aborts the bag.
- For `1to2` and `2to1`, `time` and `duration` fields whose 32-bit
  value would change signedness across the version boundary
  (post-2038 timestamps, negative durations, etc.) are transcribed
  verbatim and surfaced as a rate-limited per-topic warning. The wire
  bytes are never modified.

---

## `bagwiz convert 1to2`

Convert a ROS 1 `*.bag` to a ROS 2 rosbag2.

### Usage

```text
bagwiz convert 1to2 [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                   |
| -------- | ------------------------------------------------------------- |
| `input`  | ROS 1 `*.bag` file (must exist).                              |
| `output` | Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). |

### Options

| Flag                   | Description                                                                                                                                                                                            |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-s`, `--storage <S>`  | Output storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension.                                                                                                         |
| `--strict`             | Abort on the first topic-level error (md5 mismatch, schema unresolvable, refused canonicalisation, writer reject) instead of skipping the topic and continuing. Mutually exclusive with the next flag. |
| `--allow-md5-mismatch` | Treat md5 mismatch between the bag's ROS 1 connection record and the synthesised ROS 2 schema as a warning rather than a topic skip. Mutually exclusive with `--strict`.                               |
| `--overwrite`          | Replace `<output>` if it already exists. Without this flag, an existing output path stops the run.                                                                                                     |

### Behavior

- The ROS 1 type name (`pkg/Type`) is auto-mapped to the ROS 2
  equivalent (`pkg/msg/Type`); a small override table covers historical
  renames such as `tf/tfMessage` → `tf2_msgs/msg/TFMessage`.
- The ROS 2 schema for the destination type is then resolved via the
  hierarchy described in
  [Type and schema resolution](#type-and-schema-resolution-for-1to2--2to1).
- For each connection, bagwiz synthesises the canonical ROS 1 form of
  the resolved schema and computes its MD5. The result is compared
  against the bag's `md5sum` field for that connection. Three outcomes:
  - **Match.** The topic is converted normally; the resolved ROS 2
    schema is embedded in the output bag.
  - **Mismatch (default).** The topic is skipped with a warning
    detailing both md5s and the source the schema came from. The run
    exits with status `2` so callers can detect partial conversion.
  - **Mismatch with `--allow-md5-mismatch`.** A warning is emitted
    and the topic is admitted. Useful for known wire-equivalent
    renames where bagwiz cannot tell the new md5 from a real wire
    incompatibility — for example, `sensor_msgs/CameraInfo`'s
    `D/K/R/P → d/k/r/p` field rename across the ROS 1 / ROS 2
    boundary produces a different md5 even though the wire layout is
    identical.
- `--strict` upgrades any topic-level skip cause (md5 mismatch,
  schema unresolvable, refused canonicalisation, writer reject) into
  an immediate run abort.
- The same ROS 1 topic appearing under multiple connections (one
  publisher per chunk, etc.) is declared once on the writer side.
- The MCAP writer is configured with `compression=none`.

### Example

```bash
# Default policy: skip topics whose md5 disagrees, exit 2 if any did.
bagwiz convert 1to2 drive.bag drive.mcap

# Force SQLite3 output via --storage when the path has no recognised
# extension.
bagwiz convert 1to2 drive.bag drive_dir/ --storage sqlite3

# Admit known wire-equivalent renames (e.g. sensor_msgs/CameraInfo)
# despite the md5 mismatch.
bagwiz convert 1to2 --allow-md5-mismatch drive.bag drive.mcap

# Refuse to produce a partial bag — abort on the first divergence.
bagwiz convert 1to2 --strict drive.bag drive.mcap
```

---

## `bagwiz convert 2to1`

Convert a ROS 2 rosbag2 to a ROS 1 `*.bag` file.

### Usage

```text
bagwiz convert 2to1 [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                  |
| -------- | ------------------------------------------------------------ |
| `input`  | ROS 2 rosbag2 (directory, `*.mcap`, or `*.db3`; must exist). |
| `output` | Output ROS 1 `*.bag` file.                                   |

### Options

| Flag          | Description                                                                                                                                       |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--strict`    | Abort on the first topic-level error (schema unresolvable, refused canonicalisation, writer reject) instead of skipping the topic and continuing. |
| `--overwrite` | Replace `<output>` if it already exists. Without this flag, an existing output path stops the run.                                                |

### Behavior

- The output is a non-compressed ROS 1 bag v2.0.
- rosbag2-layer compression on the input
  (`compression_mode: FILE` / `MESSAGE` in `metadata.yaml`) is not
  supported and is rejected with a clear error. Decompress the input
  first with `ros2 bag convert`.
  - Note: MCAP chunk-level compression on a single-file MCAP input is
    transparent to bagwiz (libmcap handles it), and is therefore
    accepted.
- The ROS 2 type name (`pkg/msg/Type`) is auto-mapped to the ROS 1
  equivalent (`pkg/Type`).
- The ROS 2 schema is resolved via the hierarchy described in
  [Type and schema resolution](#type-and-schema-resolution-for-1to2--2to1);
  the bag-embedded record is preferred when present.
- bagwiz synthesises the canonical ROS 1 `(md5sum,
message_definition)` pair from the resolved schema and writes it
  into each connection record in the output bag.
- Topics whose schema cannot be resolved, or whose ROS 2 schema
  cannot be canonicalised (currently only `wstring` triggers this —
  it has no wire-equivalent ROS 1 representation), are skipped with a
  warning. The run exits with status `2`. `--strict` promotes such
  errors to an immediate abort.

### Example

```bash
bagwiz convert 2to1 drive.mcap drive.bag
bagwiz convert 2to1 rosbag2_2025_01_01/ drive.bag

# Refuse to produce a partial bag.
bagwiz convert 2to1 --strict drive.mcap drive.bag
```

---

## `bagwiz convert storage`

Repack a ROS 2 rosbag2 between MCAP and SQLite3 storage backends. Messages
are copied verbatim — no deserialization or type conversion.

### Usage

```text
bagwiz convert storage [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                   |
| -------- | ------------------------------------------------------------- |
| `input`  | Input ROS 2 rosbag2 (directory or single-file). Must exist.   |
| `output` | Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). |

### Options

| Flag                  | Description                                                                                        |
| --------------------- | -------------------------------------------------------------------------------------------------- |
| `-s`, `--storage <S>` | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension.     |
| `--overwrite`         | Replace `<output>` if it already exists. Without this flag, an existing output path stops the run. |

### Behavior

- Same-storage repacks (e.g. MCAP → MCAP) are rejected; a plain `cp` is
  what you actually want. Format detection uses magic bytes
  (single-file inputs) or `metadata.yaml` (directory layouts), never
  the file extension, so a renamed input is still classified
  correctly.
- Inputs that use rosbag2-layer compression are rejected the same way
  as `2to1`.
- For multi-shard MCAP inputs, schemas are loaded eagerly before
  declaring topics so the output preserves self-description.
- The MCAP writer is configured with `compression=none`.

### Example

```bash
# MCAP -> SQLite3 (extension picks the backend).
bagwiz convert storage drive.mcap drive.db3

# SQLite3 -> directory-layout MCAP.
bagwiz convert storage drive_dir/ drive_mcap_dir/ --storage mcap
```

---

## Type and schema resolution (for `1to2` / `2to1`)

bagwiz no longer ships a fixed list of supported message types. Any
type whose ROS 2 schema can be reached from one of the sources below
is convertible. The resolver tries them in priority order and uses
the first source that succeeds:

1. **Bag-embedded schema text.** When the input is a self-describing
   ROS 2 rosbag2 (rosbag2 from Iron onwards, or any MCAP whose
   `Schema.encoding` is `ros2msg`), the schema text inside the bag is
   used. This is the only source guaranteed to match what the
   producer actually serialised, so it wins outright when present.
   Only meaningful for `2to1`; ROS 1 bags carry no ROS 2 schema.
2. **AMENT install (`$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg`).**
   Used when the matching ROS 2 distribution is sourced. This is the
   common case after `source /opt/ros/<distro>/setup.bash` plus any
   workspace overlays.
3. **Introspection typesupport library**
   (`lib<pkg>__rosidl_typesupport_introspection_cpp.so`). Walked when
   no `.msg` file is on disk for the type — for example,
   binary-installed packages whose source `.msg` is not deployed.
   The introspection metadata cannot recover constants or default
   values, so the synthesised md5 may diverge from the AMENT result
   for constants-bearing types; the resolver records every source
   attempt so this divergence is observable.

If none of the three sources produces a schema, the topic is skipped
with `schema unresolvable`. This is the diagnostic surface for
"sourced the wrong distro" / "package isn't installed" mistakes.

`wstring` fields are the one construct the synthesizer refuses
outright: ROS 1 has no wire-equivalent counterpart. Such topics are
skipped with `schema cannot be canonicalised`.

## Exit status

| Code | Meaning                                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Conversion finished with every topic emitted. Per-topic failure tallies (per-message conversion errors, sign-flip warnings) are still logged on stderr.                                                             |
| `1`  | Argument resolution failed (bad `--storage`, ambiguous output path), the input could not be opened or used a rejected compression mode, the output could not be opened, or a fatal read/write/close error occurred. |
| `2`  | Conversion produced a partial bag: at least one topic was dropped due to a topic-level error (md5 mismatch, schema unresolvable, refused canonicalisation, writer reject). Also returned by `--strict` aborts.      |
