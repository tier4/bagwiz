# Environment variables

bagwiz reads a small set of **optional** environment variables to override its
defaults. None are required for normal use — every one has a sensible default,
and leaving them all unset gives the standard behavior. They fall into five
groups:

- [Runtime behavior](#runtime-behavior) — knobs read by the `bagwiz` binary
- [Color output](#color-output) — ANSI color control
- [Message package resolution](#message-package-resolution) — overlays for
  non-standard message types
- [Launcher and install](#launcher-and-install) — the `bagwiz` wrapper,
  installer, and dependency-build scripts (not the binary)
- [Shell completion install paths](#shell-completion-install-paths)

Diagnostic log lines and progress bars go to **stderr**; command data goes to
**stdout**, so `bagwiz … | tool` stays clean regardless of these settings.

## Runtime behavior

Read by the `bagwiz` executable itself.

| Variable                 | Accepted values (default)                                                                                                                        | Effect                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Scope                                                                                                               | Source                                            |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| `BAGWIZ_LOG_LEVEL`       | `debug`, `info`, `warn`, `error`, `fatal` — case-insensitive (default `info`)                                                                    | Minimum severity for diagnostic log lines on stderr. Set `debug` to surface the lower-level diagnostics that are otherwise suppressed — e.g. the per-topic decoder backend selection (`backend=schema` / `backend=introspection`). An unrecognized value is ignored with a warning.                                                                                                                                                                                                                        | all commands                                                                                                        | `bagwiz_base/src/core/base/logging.cpp`           |
| `BAGWIZ_DECODER`         | `introspection` forces introspection; any other value or unset selects the schema-first auto-policy                                              | Message decoder backend override. The auto-policy uses the schema-driven decoder and falls back to runtime introspection only when the schema backend cannot decode a topic.                                                                                                                                                                                                                                                                                                                               | commands that decode message contents (`walk`, `tf`, `traj`, `convert msg`, `map slam`, `cam-info`)                 | `bagwiz_msg/src/core/decoder/decoder_factory.cpp` |
| `BAGWIZ_BACKEND`         | `sequential`/`seq` or `pipelined`/`pipeline`/`pipe` — case-insensitive; unset or unrecognized keeps the command's own default                    | Overrides which rewrite backend a repack runs on. Mainly a benchmarking lever and an escape hatch back to the sequential oracle; not needed in normal use. Governs only the decoded rewrite pipeline — it has no effect on a rewrite served by the chunk pass-through (see `BAGWIZ_PASSTHROUGH`).                                                                                                                                                                                                          | rosbag-rewrite commands (`convert`, `topic`, `trim`, `traj join`, `tf static cp`, `cam-info replace`/`recompute-p`) | `bagwiz_bag/src/core/pipeline/backend_select.cpp` |
| `BAGWIZ_PASSTHROUGH`     | `off`/`0`/`false`/`no` disables — case-insensitive; unset or any other value keeps the fast path enabled                                         | Kill switch for the mcap chunk pass-through that `topic drop`/`keep`/`rename` and `trim --stamp recv` try before the decoded rewrite pipeline. Disabling it forces every rewrite through the decoded pipeline — the escape hatch for differential testing and debugging.                                                                                                                                                                                                                                   | pure-copy rewrite commands (`topic drop`/`keep`/`rename`, `trim`)                                                   | `bagwiz_bag/src/core/bag/bag_passthrough.cpp`     |
| `BAGWIZ_PROFILE`         | truthy enables; unset or `0`/`false`/`no`/`off` (case-insensitive) disables                                                                      | When enabled, prints a one-shot per-stage (read / process / write) bottleneck report to stderr after a rewrite. Ships dormant with zero hot-path cost when off. The chunk pass-through bypasses the staged pipeline, so no report is printed for a rewrite it serves.                                                                                                                                                                                                                                      | rosbag-rewrite commands (`convert`, `topic`, `trim`, `traj join`, `tf static cp`, `cam-info replace`/`recompute-p`) | `bagwiz_bag/src/core/pipeline/stage_profiler.cpp` |
| `BAGWIZ_READ_THREADS`    | integer 0–16 (default 8, capped at the host's core count); unparsable values fall back to the default with a warning                             | Worker count for the parallel read paths: per-chunk decompression on mcap, per-slice scanning on db3. `0` or `1` falls back to the single-threaded read on both — the debugging escape hatch. Emitted messages are identical either way.                                                                                                                                                                                                                                                                   | commands that read mcap or db3 bags                                                                                 | `bagwiz_io/src/io/env_tuning.hpp`                 |
| `BAGWIZ_DB3_SLICE_BYTES` | integer 1–1073741824 (default 33554432, i.e. 32 MiB); unparsable values fall back to the default with a warning                                  | Target payload bytes per timestamp slice on the parallel db3 read path. The slice count is the file size divided by this, capped at 4096; a bag smaller than one slice stays on the single-threaded scan. Peak prefetch memory scales with `(threads + 2) x` this value, so lower it (or lower `BAGWIZ_READ_THREADS`) on memory-constrained hosts.                                                                                                                                                         | commands that read db3 bags                                                                                         | `bagwiz_io/src/io/sqlite3_slice_prefetch.cpp`     |
| `BAGWIZ_DB3_PAGE_SIZE`   | a power of two in 512–65536 (default 32768, i.e. 32 KiB); any other value, including an unparsable one, falls back to the default with a warning | SQLite page size for **newly written** db3 bags. Rosbag payloads are large BLOBs, and the 4 KiB SQLite default spills a 2 MB point cloud across ~500 chained overflow pages versus ~64 at 32 KiB — worth 9–13 % on a rewrite. Existing bags are unaffected; readers negotiate the page size from the file header, so rosbag2 compatibility is unchanged. Rejected rather than clamped because SQLite silently ignores an invalid page size and would leave the 4 KiB default in place.                     | commands that write db3 bags                                                                                        | `bagwiz_io/src/io/env_tuning.hpp`                 |
| `BAGWIZ_WRITE_THREADS`   | integer 0–16 (default 8, capped at the host's core count); unparsable values fall back to the default with a warning                             | Worker count for the parallel mcap write path (per-chunk compression) on zstd/lz4 output. `0` or `1` selects the serial libmcap writer. Uncompressed output has no chunk encode to parallelize and always uses the serial writer — the `topic`/`trim`/`convert` rewrite commands are unaffected: their decoded path forces uncompressed output, and a rewrite served by the chunk pass-through re-encodes chunks itself without going through this writer. Output bytes do not depend on the worker count. | commands that write compressed mcap bags (e.g. `pcd undistort`)                                                     | `bagwiz_io/src/io/mcap_parallel_chunk_writer.cpp` |
| `BAGWIZ_TF_TREE_ASCII`   | set to any value uses ASCII; unset uses Unicode                                                                                                  | Renders `tf tree` branch glyphs as plain ASCII (hyphen, backtick, and vertical-bar characters) instead of Unicode box drawing (`├──`, `└──`, `│`). Useful for terminals or fonts without box-drawing glyphs.                                                                                                                                                                                                                                                                                               | `tf tree`                                                                                                           | `bagwiz/src/commands/tf.cpp`                      |

```bash
# Surface the normally-suppressed decoder-backend DEBUG lines
BAGWIZ_LOG_LEVEL=debug bagwiz walk capture.mcap /sensing/imu/data

# Force the runtime introspection decoder
BAGWIZ_DECODER=introspection bagwiz walk capture.mcap /topic

# Print a per-stage timing report for a repack
BAGWIZ_PROFILE=1 bagwiz convert format capture.mcap out.mcap
```

## Color output

| Variable                   | Accepted values (default)                                                      | Effect                                                                                                                                                                                           | Scope            | Source                                                                   |
| -------------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------- | ------------------------------------------------------------------------ |
| `NO_COLOR`                 | set to any value disables color (unset enables)                                | Honors the no-color.org convention: suppresses ANSI colors in `tf tree` and in the `map slam` progress output. Semantic tags (e.g. `tf tree`'s `[S]` / `[D]`) are still printed.                 | `tf`, `map slam` | `bagwiz/src/commands/tf.cpp`, `bagwiz/src/commands/map_slam_mapping.cpp` |
| `RCUTILS_COLORIZED_OUTPUT` | `1` forces color on, `0` forces it off; unset colors only when stderr is a TTY | Controls whether diagnostic **log lines** are wrapped in per-severity ANSI color, mirroring rcutils' own policy. Independent of `NO_COLOR`, which governs command styling rather than log lines. | all log output   | `bagwiz_base/src/core/base/logging.cpp`                                  |

Color is also omitted automatically when the relevant stream is not a terminal
(e.g. piped to a file), the same visual effect as `NO_COLOR`.

## Message package resolution

Bags whose topics use message types beyond the standard ROS 2 stack need the
matching message packages available at run time. These are standard ROS 2
variables, normally set for you by sourcing an overlay's `install/setup.bash`
(see [Using your own message packages](../README.md#using-your-own-message-packages-overlays)).

| Variable            | Effect                                                                                                                                      | Source        |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `AMENT_PREFIX_PATH` | Colon-separated install prefixes where bagwiz locates `msg/*.msg` definitions and introspection typesupport for non-standard message types. | ROS 2 (ament) |
| `LD_LIBRARY_PATH`   | Shared-library search path used to `dlopen()` the introspection typesupport libraries at run time.                                          | OS / ROS 2    |

```bash
source /path/to/my_msgs_ws/install/setup.bash   # sets both variables
bagwiz walk my.mcap /topic
```

## Launcher and install

Read by the wrapper, installer, and build **shell scripts**, not by the `bagwiz`
binary. They control how the `bagwiz` launcher on your `PATH` is installed,
which build it runs, and which CUDA toolkit the GLIM dependency build uses.

| Variable                | Accepted values (default)                                                                    | Effect                                                                                                                                                                      | Source                       |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------- |
| `BAGWIZ_DEFAULT_DISTRO` | a built distro name (default `jazzy`)                                                        | Which built distro the installed `bagwiz` launcher runs. Baked in by `pixi run -e <distro> install`; export it to switch the active build at run time without reinstalling. | `scripts/bagwiz-run.sh`      |
| `BAGWIZ_INSTALL_DIR`    | a directory (default `~/.local/bin`)                                                         | Where `pixi run install` places the `bagwiz` launcher.                                                                                                                      | `scripts/bagwiz-install.sh`  |
| `BAGWIZ_CUDA_HOME`      | a CUDA toolkit root (default: the active pixi env's conda CUDA, else `/usr/local/cuda-12.8`) | Overrides the CUDA toolkit used by the `--cuda` GLIM dependency build, letting a system toolkit replace the pixi-managed one.                                               | `scripts/build-glim-deps.sh` |

## Shell completion install paths

Read by `bagwiz complete --install` to decide where to write completion
scripts. These are standard OS variables; you rarely need to set them.

| Variable          | Effect                                                                                                      | Source                               |
| ----------------- | ----------------------------------------------------------------------------------------------------------- | ------------------------------------ |
| `HOME`            | Base for the default completion install location and for expanding a leading `~` in paths.                  | `bagwiz/src/commands/completion.cpp` |
| `XDG_DATA_HOME`   | Overrides the base data directory (default `~/.local/share`) used for some shells' completion destinations. | `bagwiz/src/commands/completion.cpp` |
| `XDG_CONFIG_HOME` | Overrides the base config directory (default `~/.config`) used for some shells' completion destinations.    | `bagwiz/src/commands/completion.cpp` |
