# `bagwiz convert`

Cross-format bag conversion. One subcommand:

| Subcommand                         | Direction                                                                          |
| ---------------------------------- | ---------------------------------------------------------------------------------- |
| [`format`](#bagwiz-convert-format) | ROS 2 rosbag2 repack between MCAP and SQLite3 storage and/or file/directory layout |

## Common notes

- Target storage backend resolution order (first match wins):
  1. `-s/--storage` if given.
  2. Output path's extension (`.mcap` → MCAP, `.db3` → SQLite3).
  3. Input bag's detected storage backend. Directory-layout outputs
     without `--storage` therefore inherit the input's backend — handy
     for a pure file ↔ directory layout change.
- Any pre-existing entry at `<output>` (file or directory) stops the
  run with a clear log line. Pass `--overwrite` to replace it instead.
  The flag is supported by every `bagwiz` subcommand that writes a
  file or directory output (`convert format`, `traj dump`, `traj join -o`,
  `tf inject-static -o`).
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
| `--overwrite`         | Replace `<output>` if it already exists. Without this flag, an existing output path stops the run.                                               |

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
- Inputs that use rosbag2-layer compression
  (`compression_mode: FILE` / `MESSAGE` in `metadata.yaml`) are
  rejected with a clear error. Decompress the input first with
  `ros2 bag convert`.
  - Note: MCAP chunk-level compression on a single-file MCAP input is
    transparent to bagwiz (libmcap handles it), and is therefore
    accepted.
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

## Exit status

| Code | Meaning                                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Repack finished successfully. Per-message failure tallies are still logged on stderr.                                                                                                                               |
| `1`  | Argument resolution failed (bad `--storage`, ambiguous output path), the input could not be opened or used a rejected compression mode, the output could not be opened, or a fatal read/write/close error occurred. |
