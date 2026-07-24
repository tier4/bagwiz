# `bagwiz convert`

Cross-format bag conversion. Subcommands:

| Subcommand                           | Direction                                                                           |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| [`format`](#bagwiz-convert-format)   | ROS 2 rosbag2 repack between MCAP and SQLite3 storage and/or file/directory layout  |
| [`msg geo`](#bagwiz-convert-msg-geo) | Convert a geographic source (`NavSatFix`) into a pose type, projected to ENU or UTM |

`msg` is a command group for message-type conversions; each family of
conversions lives under its own leaf (today: `geo` for position-related types).

## Common notes

- Target storage backend resolution order (first match wins):
  1. `-s/--storage` if given.
  2. Output path's extension (`.mcap` → MCAP, `.db3` → SQLite3).
  3. Input bag's detected storage backend. Directory-layout outputs
     without `--storage` therefore inherit the input's backend — handy
     for a pure file ↔ directory layout change.
- Any pre-existing entry at `<output>` (file or directory) stops the
  run with a clear log line. Pass `-w`/`--overwrite` to replace it instead.
  The flag is supported by every `bagwiz` subcommand that writes a
  file or directory output.
- `mcap` outputs are written without chunk compression. Re-compress
  afterwards with `ros2 bag convert` if needed.

---

## `bagwiz convert format`

Repack a ROS 2 rosbag2. The subcommand handles two independent
conversions in one pass — choose the target storage backend (MCAP ↔
SQLite3) via `-s/--storage`, and choose the on-disk layout
(single-file ↔ directory) via the shape of `<output>`. Messages are
copied verbatim; no deserialization or type conversion is performed.

### Usage

```text
bagwiz convert format [OPTIONS] <input> <output>
```

### Positional arguments

| Name     | Description                                                   |
| -------- | ------------------------------------------------------------- |
| `input`  | Input ROS 2 rosbag2 (directory or single-file). Must exist.   |
| `output` | Output rosbag2 directory or single-file (`*.mcap` / `*.db3`). |

### Options

| Flag                  | Description                                                                                                                                      |
| --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-s`, `--storage <S>` | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension; otherwise inherited from the input bag's storage. |
| `-w`, `--overwrite`   | Replace `<output>` if it already exists. Without this flag, an existing output path stops the run.                                               |

### Behavior

- Same-storage + same-layout repacks (e.g. MCAP file → MCAP file) are
  rejected; a plain `cp` is what you actually want. Format detection
  uses magic bytes (single-file inputs) or `metadata.yaml` (directory
  layouts), never the file extension, so a renamed input is still
  classified correctly.
- Layout transitions (file ↔ directory) are supported on either
  storage backend. The output layout is derived from `<output>`: a
  path ending in `.mcap` or `.db3` produces a single-file bag, any
  other path produces a directory-layout bag with the canonical
  `metadata.yaml`.
- rosbag2-layer compression handling on directory inputs (driven by
  `compression_mode` / `compression_format` in `metadata.yaml`):
  - `MESSAGE` + `zstd`: payloads are transparently decompressed on
    read; an `[INFO]` line announces the path. The output bag is
    always written uncompressed.
  - `MESSAGE` + non-zstd: rejected with a clear error (only `zstd`
    is implemented today).
  - `FILE` on MCAP: this is rosbag2's label for storage-internal
    chunk compression, which libmcap decompresses transparently.
    Accepted; no extra work needed.
  - `FILE` + `zstd` on SQLite3: a whole-database `.db3.zstd` envelope.
    Each shard is stream-decompressed to a temporary `.db3` on read (an
    `[INFO]` line announces the path) and removed when the reader
    closes; reading needs free temp space roughly the size of the
    decompressed database. A bare single-file `.db3.zstd` is accepted
    the same way. The output bag is always written uncompressed.
  - `FILE` + non-zstd on SQLite3: rejected with a clear error (only
    `zstd` is implemented today).
  - `NONE` / empty / single-file MCAP inputs: nothing to do; MCAP
    chunk compression on a single-file MCAP is already transparent
    via libmcap.
- For multi-shard MCAP inputs, schemas are loaded eagerly before
  declaring topics so the output preserves self-description.
- SQLite3 inputs from Humble and earlier carry no embedded message
  definitions. bagwiz resolves each missing definition from
  `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg` before declaring
  the topic so the resulting MCAP keeps self-description for strict
  downstream readers.
- The MCAP writer is configured with `compression=none`.

### Example

```bash
# MCAP file -> SQLite3 file (extension picks the backend).
bagwiz convert format drive.mcap drive.db3

# SQLite3 directory -> directory-layout MCAP.
bagwiz convert format drive_dir/ drive_mcap_dir/ --storage mcap

# Layout change without storage change: single-file MCAP -> directory MCAP.
# --storage is optional here — the directory output inherits MCAP from the input.
bagwiz convert format drive.mcap drive_dir/
```

---

## `bagwiz convert msg geo`

Convert the message type of selected topics from a geographic source
(`sensor_msgs/msg/NavSatFix`) into a `geometry_msgs` pose type, projecting
WGS84 latitude/longitude/altitude into a Cartesian frame (ENU local tangent
plane or UTM) via [GeographicLib](https://geographiclib.sourceforge.io/). Only
the selected topics are re-typed; every other topic is copied verbatim.

The conversion is **one-directional** (`NavSatFix` → pose). The reverse needs the
original datum/zone (absent from a pose bag) and would synthesise `NavSatFix`
status fields, so it is intentionally not offered.

Whitelisted routes (`--src` → `--dst`):

| `--src`       | `--dst`                                        |
| ------------- | ---------------------------------------------- |
| `nav_sat_fix` | `pose_with_covariance_stamped`, `pose_stamped` |

### Usage

```text
bagwiz convert msg geo [OPTIONS] <input> --dst <type>
```

### Positional arguments

| Name    | Description                                                |
| ------- | ---------------------------------------------------------- |
| `input` | Input ROS 2 rosbag (directory or single-file). Must exist. |

### Options

| Flag                     | Description                                                                                                           |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------- |
| `--src <type>`           | Source message type (snake_case). Required unless `--topic` is given; **ignored** when it is.                         |
| `--dst <type>`           | Target message type (snake_case). Required.                                                                           |
| `--topic <t>...`         | Convert exactly these topic(s) instead of every topic matching `--src`. All named topics must share one message type. |
| `--crs <enu\|utm>`       | Target Cartesian coordinate system. Optional; defaults to `enu`.                                                      |
| `--origin <lat,lon,alt>` | WGS84 datum. Required for ENU unless it can be derived from the first `NavSatFix`; an optional offset for UTM.        |
| `--frame-id <name>`      | `frame_id` written onto the converted messages. Defaults to `map` (enu) or `utm` (utm).                               |
| `-o`, `--output <p>`     | Write the result to a new bag instead of rewriting `<input>` in place.                                                |
| `-w`, `--overwrite`      | Replace an existing `-o` path. Without it, an existing output path stops the run.                                     |

### Behavior

- **Topic selection.** With `--topic`, exactly those topics are converted and
  `--dst` is required (`--src` is ignored); all named topics must share one
  message type. Without `--topic`, `--src` and `--dst` are required and every
  topic whose type matches `--src` is converted.
- **Coordinate systems.** `enu` projects to a local East-North-Up tangent plane
  (metres) around the origin (`position.x` = East, `y` = North, `z` = Up).
  `utm` projects to UTM easting/northing (the zone/hemisphere come from the
  longitude); an `--origin` shifts the result so coordinates stay small.
- **Origin.** `--origin` wins when given. For ENU without `--origin`, the first
  decodable `NavSatFix` among the selected topics is used and logged so the value
  can be reused later. UTM tolerates an absent origin (standard UTM).
- **frame_id.** The source sensor `frame_id` is replaced (it names the antenna
  frame, not the world frame); the default is the REP-105 `map` for ENU or the
  `robot_localization` convention `utm` for UTM.
- **Fields.** The source header timestamp is preserved. `orientation` is set to
  identity `(0,0,0,1)`. For `pose_with_covariance_stamped`, the `NavSatFix`
  `position_covariance` (3×3) maps into the upper-left block of the pose 6×6
  covariance; the rotation block stays zero. `pose_stamped` has no covariance,
  so it is dropped.
- **In-place vs `-o`.** Without `-o`, `<input>` is rewritten via an atomic
  tmp-swap that preserves its storage format and layout. With `-o`, `<input>`
  is left untouched and the result is written to that path.
- The output MCAP is written with `compression=none`.

### Example

```bash
# Convert every NavSatFix topic to PoseWithCovarianceStamped in a local ENU
# frame (the default CRS), deriving the origin from the first fix; new bag.
bagwiz convert msg geo drive.mcap \
  --src nav_sat_fix --dst pose_with_covariance_stamped -o drive_pose.mcap

# Convert one specific topic to PoseStamped in UTM, in place.
bagwiz convert msg geo drive.mcap \
  --topic /sensing/gnss/fix --dst pose_stamped --crs utm --origin 35.68,139.76,40.0
```

## Exit status

| Code | Meaning                                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Repack finished successfully. Per-message failure tallies are still logged on stderr.                                                                                                                               |
| `1`  | Argument resolution failed (bad `--storage`, ambiguous output path), the input could not be opened or used a rejected compression mode, the output could not be opened, or a fatal read/write/close error occurred. |
