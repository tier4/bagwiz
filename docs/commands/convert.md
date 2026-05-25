# `bagwiz convert`

Cross-format bag conversion. One subcommand:

| Subcommand                           | Direction                                     |
| ------------------------------------ | --------------------------------------------- |
| [`storage`](#bagwiz-convert-storage) | ROS 2 rosbag2 repack between MCAP and SQLite3 |

## Common notes

- When `--storage` is omitted the storage backend is inferred from the
  output path's extension (`.mcap` → MCAP, `.db3` → SQLite3). Output
  paths that do not carry one of those extensions (e.g. a directory)
  require an explicit `--storage`.
- Any pre-existing entry at `<output>` (file or directory) stops the
  run with a clear log line. Pass `--overwrite` to replace it instead.
  The flag is supported by every `bagwiz` subcommand that writes a
  file or directory output (`convert storage`, `traj dump`, `traj join -o`,
  `tf inject-static -o`).
- `mcap` outputs are written without chunk compression. Re-compress
  afterwards with `ros2 bag convert` if needed.

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
# MCAP -> SQLite3 (extension picks the backend).
bagwiz convert storage drive.mcap drive.db3

# SQLite3 -> directory-layout MCAP.
bagwiz convert storage drive_dir/ drive_mcap_dir/ --storage mcap
```

## Exit status

| Code | Meaning                                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | Repack finished successfully. Per-message failure tallies are still logged on stderr.                                                                                                                               |
| `1`  | Argument resolution failed (bad `--storage`, ambiguous output path), the input could not be opened or used a rejected compression mode, the output could not be opened, or a fatal read/write/close error occurred. |
