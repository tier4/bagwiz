# `bagwiz check`

Integrity checks for ROS 2 rosbags. `check` is a command group; its action is
`broken`.

| Subcommand                       | What it does                                                            |
| -------------------------------- | ----------------------------------------------------------------------- |
| [`broken`](#bagwiz-check-broken) | Scan rosbags for storage-level corruption and, optionally, delete them. |

---

## `bagwiz check broken`

Scan one or more rosbags for storage-level corruption and, optionally, delete
the broken ones.

A bag is reported as broken only when its storage container cannot be read as a
rosbag — a truncated or corrupt MCAP header / footer / index, a damaged SQLite
database, and similar format-level damage. A mere mismatch between the
statistics recorded in `metadata.yaml` and the actual records is not treated as
broken.

### Usage

```text
bagwiz check broken -i <input> [--rm] [--deep]
```

### Examples

```bash
# Report broken bags under a tree (does not delete anything when piped).
bagwiz check broken -i ~/data/rosbags/

# Check a single bag.
bagwiz check broken -i capture.mcap

# Delete every broken bag found, no prompt.
bagwiz check broken -i ~/data/rosbags/ --rm

# Thorough scan that reads every message.
bagwiz check broken -i ~/data/rosbags/ --deep

# Feed the broken-bag list into another tool.
bagwiz check broken -i ~/data/rosbags/ > broken.txt
```

### Options

| Flag                    | Description                                                                                                                                                                                |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `-i`, `--input <input>` | **Required.** A single rosbag (`*.mcap` / `*.db3` / `*.db3.zstd`, or a rosbag2 directory) or a directory to scan. A directory is walked recursively and every rosbag within it is checked. |
| `--rm`                  | Delete every broken bag without prompting. Without this flag, the broken bags are listed and you are asked once before anything is deleted.                                                |
| `--deep`                | Thorough mode: stream every message to end-of-file (without decoding) to catch payload corruption a structural check cannot see. This reads the whole bag, so it is much slower.           |

### Bag discovery

When `input` is a directory, every directory that contains a `metadata.yaml`
is treated as one rosbag (its shards are not checked individually), and every
loose `*.mcap` / `*.db3` / `*.db3.zstd` file is treated as one rosbag.

A single file passed directly as `input` is also accepted when its storage
format is detected from its magic bytes, so a bag file that has been renamed
or stripped of its extension still works. The `*.mcap` / `*.db3` / `*.db3.zstd`
extension rule is what governs files discovered during the recursive directory
walk.

### Check depth

By default the check is structural: each bag is opened and its topic list and
summary statistics are read, without decoding any message payload. For a
directory bag whose `metadata.yaml` already carries a complete message summary,
that summary is trusted and the shards are not re-opened; use `--deep` to force
every shard and chunk to be read.

### Output

- The list of broken bags is written to `stdout`, one path per line, so it
  pipes cleanly into other tools.
- Progress, per-bag failure reasons, and the deletion prompt are written to
  `stderr`.

### Deleting broken bags

- Without `--rm`, deletion happens only after you confirm at the interactive
  prompt. When `stdin` is not a terminal (for example in a pipeline), nothing
  is deleted and you are advised to re-run with `--rm`.
- A single-file bag is deleted as one file; a directory bag is deleted
  recursively.

## Exit status

| Code | Meaning                              |
| ---- | ------------------------------------ |
| `0`  | Success.                             |
| `1`  | Failed — check stderr for the cause. |
