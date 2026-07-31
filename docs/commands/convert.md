# `bagwiz convert`

Cross-format bag conversion. Subcommands:

| Subcommand                         | What it does                                                                       |
| ---------------------------------- | ---------------------------------------------------------------------------------- |
| [`format`](#bagwiz-convert-format) | ROS 2 rosbag2 repack between MCAP and SQLite3 storage and/or file/directory layout |

---

## `bagwiz convert format`

Repack a ROS 2 rosbag2. The subcommand handles two independent
conversions in one pass — choose the target storage backend (MCAP ↔
SQLite3) via `--storage`, and choose the on-disk layout
(single-file ↔ directory) via the shape of `<output>`. Messages are
copied verbatim; no deserialization or type conversion is performed.

### Usage

```text
bagwiz convert format -i <input> -o <output> [OPTIONS]
```

### Examples

```bash
# MCAP file -> SQLite3 file (extension picks the backend).
bagwiz convert format -i drive.mcap -o drive.db3

# SQLite3 directory -> directory-layout MCAP.
bagwiz convert format -i drive_dir/ -o drive_mcap_dir/ --storage mcap

# Layout change without storage change: single-file MCAP -> directory MCAP.
# --storage is optional here — the directory output inherits MCAP from the input.
bagwiz convert format -i drive.mcap -o drive_dir/
```

### Options

| Flag                      | Description                                                                                                                                                                                                                              |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `-i`, `--input <input>`   | **Required.** Input ROS 2 rosbag2 (directory or single-file). Must exist.                                                                                                                                                                |
| `-o`, `--output <output>` | **Required.** Output rosbag2 directory or single-file (`*.mcap` / `*.db3`).                                                                                                                                                              |
| `--storage <S>`           | Target storage backend. One of `mcap`, `sqlite3`. Default: inferred from the output extension; otherwise inherited from the input bag's storage (see [Storage backend resolution](#storage-backend-resolution)). Long-form only.         |
| `-w`, `--overwrite`       | Replace `<output>` if it already exists. Without this flag, any pre-existing entry at `<output>` (file or directory) stops the run with a clear log line. Supported by every `bagwiz` subcommand that writes a file or directory output. |

### Storage backend resolution

The target storage backend is resolved in this order (first match wins):

1. `--storage` if given.
2. Output path's extension (`.mcap` → MCAP, `.db3` → SQLite3).
3. Input bag's detected storage backend. Directory-layout outputs
   without `--storage` therefore inherit the input's backend — handy
   for a pure file ↔ directory layout change.

### Format detection

- Same-storage + same-layout repacks (e.g. MCAP file → MCAP file) are
  rejected; a plain `cp` is what you actually want. Format detection
  uses magic bytes (single-file inputs) or `metadata.yaml` (directory
  layouts), so a renamed `.mcap` / `.db3` input is still classified
  correctly. The one exception: for a single-file zstd envelope
  (`.mcap.zstd` / `.db3.zstd`) the magic sniff cannot see past the
  compression, so the inner storage is resolved from the extension.

### Layout conversion

- Layout transitions (file ↔ directory) are supported on either
  storage backend. The output layout is derived from `<output>`: a
  path ending in `.mcap` or `.db3` produces a single-file bag, any
  other path produces a directory-layout bag with the canonical
  `metadata.yaml`.

### Compression handling

rosbag2-layer compression handling on directory inputs (driven by
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

The MCAP writer is configured with `compression=none`: `mcap` outputs
are written without chunk compression. Re-compress afterwards with
`ros2 bag convert` if needed.

### Self-description preservation

- For multi-shard MCAP inputs, schemas are loaded eagerly before
  declaring topics so the output preserves self-description.
- SQLite3 inputs from Humble and earlier carry no embedded message
  definitions. bagwiz resolves each missing definition from
  `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg` before declaring
  the topic so the resulting MCAP keeps self-description for strict
  downstream readers.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
