# `bagwiz trim`

Trim a rosbag to a time window. Only the messages inside the window are copied
to the output; every topic stays declared, and no deserialization or type
conversion is performed.

## Usage

```text
bagwiz trim [OPTIONS] <input> [--start <offset>] [--end <offset> | --duration <length> | --both <offset> | --align <topics>...]
```

## Positional arguments

| Name    | Description                                                |
| ------- | ---------------------------------------------------------- |
| `input` | Input ROS 2 rosbag (directory or single-file). Must exist. |

## Options

| Flag                  | Description                                                                                                                                                                                                                 |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--start <bound>`     | Window start: an offset from the bag start (e.g. `5s`, `500ms`) or a message count (`100msg` skips the first 100 messages). Default: bag start.                                                                             |
| `--end <bound>`       | Window end, exclusive: an offset from the bag start (e.g. `90s`) or a message count (`500msg` keeps the first 500 messages). Default: bag end.                                                                              |
| `--duration <len>`    | Window length measured from the window start (e.g. `30s`). Time only — no `msg`. Mutually exclusive with `--end`.                                                                                                           |
| `--both <bound>`      | Trim this much from both the bag start and the bag end: a time offset (`5s`) or a message count (`50msg`). Mutually exclusive with `--start`, `--end`, and `--duration`.                                                    |
| `--align <topics>...` | Trim to the common time span of these topics — from their latest first message to their earliest last message, both included. Literal names or `*` globs (as in `topic drop -t`). Mutually exclusive with the offset flags. |
| `--stamp <clock>`     | Reference clock for the window: `header` (default — `header.stamp`, with per-message fallback to receive time) or `recv` (record time). See Reference clock below.                                                          |
| `-o`, `--output <p>`  | Write the result to a new bag instead of rewriting `<input>` in place.                                                                                                                                                      |
| `-w`, `--overwrite`   | Replace an existing `-o` path. Without it, an existing output path stops the run. No effect in-place.                                                                                                                       |

At least one of `--start`, `--end`, `--duration`, `--both`, or `--align` must be
given — a windowless trim would be a plain copy, which is `cp -r`'s job.

## Time window semantics

- Offsets are **relative to the bag's start time** (the earliest message
  timestamp), not absolute epoch times: `--start 5s` means "5 seconds into the
  bag".
- Every offset needs an explicit unit: `ns`, `us` (or `µs`), `ms`, or `s`.
  Fractional values are fine (`1.5s`, `0.05s`). A bare number such as
  `--start 5` is rejected — it will not be silently read as milliseconds.
- The window is **half-open** `[start, end)`: a message stamped exactly at the
  resolved end is excluded. Consecutive windows (`--start 0s --end 30s`,
  `--start 30s --end 60s`) therefore partition a bag with no duplicated
  message, and `--duration` composes exactly as `end = start + duration`.
- An end past the bag's last message is allowed (the output simply ends at the
  bag end, after a warning) — handy for "everything from 5s on" scripting. A
  **start** past the bag end stops the run with an error instead, since it can
  only produce an empty bag and is almost certainly a typo.
- `--both X` is shorthand for `--start X --end (bag_duration − X)`: the kept
  window shrinks symmetrically from both ends, with the same half-open
  semantics. It is rejected when it would trim away the entire bag
  (`2·X ≥ bag duration`) — and `--both 0s` is rejected too, since trimming
  nothing is the windowless case above.
- `--start`, `--end`, and `--both` also accept a **message count** with the
  `msg` unit: `--start 100msg` skips the 100 earliest messages, `--end 500msg`
  keeps (at most) the 500 earliest, `--both 50msg` trims 50 from each end.
  Counts rank every message on the reference clock (see `--stamp`) and resolve
  to that message's clock value, so when several messages share the boundary
  clock the whole tie group lands on one side and the effective count shifts
  accordingly. An `--end` count at or past the bag's message total behaves like
  an end past the bag end (warn, keep to the end); a `--start` count at or past
  it is an error. Counts are whole numbers ≥ 0; `--duration` takes only time
  lengths.
- `--align` takes topic selectors instead of offsets: the window runs from the
  **latest first message** to the **earliest last message** across the selected
  topics — the span where every selected topic has data — and, unlike `--end`,
  **both boundary messages are included** (the option's contract is that the
  selected topics' first and last messages are part of the output). The run
  stops with an error when a selector matches no topic, a selected topic has
  no messages, or the selected topics do not overlap in time.

## Reference clock

By default every time comparison — the offset anchor, `--align`'s first/last
resolution, and the per-message keep decision — uses the message's
**`header.stamp`** (the sensor/acquisition time), not the time the message was
recorded into the bag. This keeps windows consistent for downstream tools that
pair data by stamp: a camera frame whose stamp is inside the window is kept
even when pipeline latency pushed its record time outside it.

- A message falls back to its **receive time** when its type has no leading
  `std_msgs/Header` field, its stamp was left at 0, or its type's definition
  cannot be found (no embedded schema and no `.msg` on `$AMENT_PREFIX_PATH` —
  a warning lists such types once).
- Message timestamps in the output bag are never rewritten; the clock only
  decides which messages are kept.
- `header` mode cannot use the storage's time index (stamps live inside the
  payloads), so the whole bag is scanned regardless of window size. Pass
  `--stamp recv` to window on record time and get the indexed fast path —
  right for bags without headers, or when the record order is what matters.

## Behavior

- The window is resolved against the bag's time extent before anything is
  written; bad offsets or an out-of-range start fail the run and leave the
  input untouched.
- Trim removes messages, never topics: every topic declaration is copied
  verbatim, so a topic whose messages all fall outside the window is still
  declared (with zero messages) in the output. A window that contains no
  messages at all still produces a valid, empty bag, after a warning.
- The time range is pushed down into the storage layer (MCAP chunk index /
  SQLite `WHERE`), so out-of-range data is skipped, not read and discarded.
- Bags whose storage carries no time information (an empty bag, or an MCAP
  without a summary index) cannot anchor relative offsets; the run stops with
  an error.
- In-place vs `-o`. Without `-o`, `<input>` is rewritten via an atomic
  tmp-swap that preserves its storage format and layout. With `-o`, `<input>`
  is left untouched and the result is written to that path; the output's
  storage follows the output extension (`.mcap` / `.db3` pick a single-file
  backend) or, for a directory output, inherits the input bag's storage
  backend.
- Embedded message schemas are preserved so MCAP outputs stay self-describing.
- The output MCAP is written with `compression=none`. Re-compress afterwards
  with `ros2 bag convert` if needed.

## Example

```bash
# Keep seconds 5..90 of the bag (message at exactly 90s excluded).
bagwiz trim drive.mcap --start 5s --end 90s -o drive_cut.mcap

# The same window, expressed as a length.
bagwiz trim drive.mcap --start 5s --duration 85s -o drive_cut.mcap

# Drop the first 10 seconds, rewriting the bag in place.
bagwiz trim drive_dir/ --start 10s

# Keep only the first half-second.
bagwiz trim drive.mcap --end 500ms -o head.mcap

# Cut 5 seconds off each end of the bag.
bagwiz trim drive.mcap --both 5s -o drive_inner.mcap

# Keep messages 101..1000 (skip the first 100, end after the 1000th).
bagwiz trim drive.mcap --start 100msg --end 1000msg -o drive_head.mcap

# Keep only the span where the lidar topic has data (its first to its last
# message, both included).
bagwiz trim drive.db3 --align /sensing/lidar/concatenated/pointcloud -o aligned.db3
```

## Exit status

| Code | Meaning                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0`  | The bag was trimmed successfully, including a window that contains no messages (a valid empty bag is written, after a warning).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| `1`  | No window flag was given; `--end` and `--duration` were combined; `--both` or `--align` was combined with another window flag; `--both` was zero or would trim away the entire bag; an `--align` selector matched no topic, a selected topic had no messages, or the selected topics did not overlap in time; `--stamp` was not `header`/`recv`; an offset was negative, missing its unit, or unparseable; a message count was fractional, negative, or malformed; `--duration` was given a `msg` count; the window was empty (`start >= end`); the start (offset or message count) was past the bag end; the bag had no time extent; the input could not be opened; the `-o` output path collided without `-w`/`--overwrite`; the input storage format could not be detected for an in-place rewrite; or a read/write/close error occurred. |
