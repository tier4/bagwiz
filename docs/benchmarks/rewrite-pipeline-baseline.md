# Rewrite Pipeline — Single-Core Baseline & Speedup Plan

This is the measured baseline for bagwiz's `read → process → write` rewrite loop
on the current `main`, plus the per-command acceleration plan it justifies. It is
the reference the pipeline acceleration backends (PRD Milestones #2–#3) are
benchmarked against.

Reproduce with [`scripts/bench-rewrite.sh`](../../scripts/bench-rewrite.sh). The
per-stage read/write split comes from the env-gated `BAGWIZ_PROFILE` instrumentation
(`BAGWIZ_PROFILE=1 bagwiz topic drop …`); CPU% and RSS from `/usr/bin/time -v`.

## Setup

|                   |                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------ |
| Commit            | `16cbc79` (profiler re-applied onto current `main`)                                  |
| Host              | 24 cores / 62 GB                                                                     |
| Build             | `pixi run -e default build` (Release, Humble)                                        |
| Canonical fixture | `~/data/rosbags/raw/8Q6aVzvu_4V2JAmyk_2025-11-30T17-24-59+0900_27.mcap`              |
| Fixture size      | 2.5 GB (2,677,294,482 bytes), zstd-compressed chunks; 31,678 msgs; 2,835 MiB payload |
| Cache             | warm (OS page cache primed before timed runs)                                        |

The 2.5 GB bag is the canonical fixture: it is the smallest of the `raw/` set
(2.5–4.1 GB) and anchors the prior baseline, so it is the fastest representative
workload to iterate against. Larger bags scale roughly linearly.

## Results (warm)

| Scenario       | Command                    | In-proc wall         | Read             | Process   | Write            | CPU  | Peak RSS |
| -------------- | -------------------------- | -------------------- | ---------------- | --------- | ---------------- | ---- | -------- |
| Full rewrite   | `topic drop /tf_static -o` | 2.469 s              | 0.787 s (31.9 %) | 0 s (0 %) | 1.682 s (68.1 %) | 99 % | 51 MB    |
| Sparse extract | `topic keep /tf_static -o` | 0.777 s              | 0.777 s (100 %)  | 0 s       | 0 s              | —    | —        |
| Read-only      | `check broken --deep`      | ~1.0 s (median wall) | —                | —         | —                | —    | —        |

- Read throughput ≈ **3,600 MiB/s** (decompressed payload); write throughput ≈ **1,690 MiB/s**.
- Full-rewrite output: 2.8 GB, **1.11× expansion** (output is uncompressed).

## Bottleneck analysis

1. **Single-threaded.** 99 % CPU = **1 of 24 cores.** Every other core is idle for
   the entire run. All speedup headroom is parallelism, not micro-optimization.
2. **Write-dominant.** Full rewrite is **~68 % write / ~32 % read / 0 % process.**
   The process stage is literally zero for pure copy. The wall-clock cross-check
   (read 35 % / write 65 %) agrees; the small gap is `pixi run` activation
   overhead folded into the read-only scenario.
3. **Write cost is not compression.** bagwiz writes **uncompressed** today
   (`mcap_compression="none"` is forced by the rewrite commands), so the 1.682 s
   write is redundant memcpy + libmcap chunk-CRC + uncompressed disk I/O — not
   zstd. Re-enabling compression would _add_ CPU cost but shrink disk I/O; it is a
   separate axis from parallelism.
4. **Sparse extraction is a read floor.** Keeping one tiny latched topic still
   costs **0.78 s** of pure read+decompress, because the dropped high-volume topics
   share MCAP chunks with everything else — chunk-index pruning rarely skips work
   on these bags. Any sparse command is bounded below by this read floor.
5. **Known per-message waste** (targets for later micro-opt, secondary to
   parallelism): the reader copies each payload into an intermediate buffer before
   libmcap copies again; the writer heap-allocates the topic name twice per message.

### Achievable ceiling (Amdahl)

With a 3-stage `read ‖ process ‖ write` pipeline, wall-clock floors at the slowest
stage. Here write is ~68 %, so the ceiling is roughly **1 / 0.68 ≈ 1.45×** before
the write stage itself becomes the wall. Exceeding ~1.5× requires parallelizing
_within_ the write path (parallel chunk encode/CRC, or `ZSTD_c_nbWorkers` once
compression is on) or overlapping output I/O. This is why the PRD sets a
**measure-and-report** target, not a fixed multiple.

## Speedup plan — per-command backend recommendation

| Command class          | Commands                                                            | Cost profile                           | Recommended backend                                                  | Reorder allowed?                            |
| ---------------------- | ------------------------------------------------------------------- | -------------------------------------- | -------------------------------------------------------------------- | ------------------------------------------- |
| Pure copy              | `topic drop`, `topic keep`, `topic rename`, `convert` (format only) | write-dominant, process=0              | **Pipelined** (read ‖ write), 2-stage                                | Emission-order only (seq key)               |
| Transform              | `convert msgtype geo`                                               | process non-trivial (decode/transform) | **Pipelined** 3-stage; consider decode workers only if process grows | Emission-order only; **no** content reorder |
| Inject                 | `tf_static_cp`, `traj join`                                         | write-dominant + a `finish()` append   | **Pipelined** + `finish()`; injected record stays last               | **Forbidden** — strict emission order       |
| Analysis / interactive | `ls`, `check`, `walk`, `tf tree`/`walk`, `traj dump`                | read-only / interactive                | **Unchanged** (no rewrite)                                           | n/a                                         |
| Decode/encode-heavy    | `generate video`                                                    | CPU-bound encode                       | Out of scope here (different profile)                                | n/a                                         |

Rationale: the pure-copy trio (MVP) gets the most from a pipeline because its write
stage dominates and its process stage is empty, so overlapping read with write is
the entire win. A decode-worker / parallel-map backend is **not** recommended for
MVP — the process stage is 0 % for copy and a thin slice even for transform, so
fanning out decode buys little while adding decoder thread-safety risk.

## Resolved PRD open questions

- **#1 (profiler placement):** the `BAGWIZ_PROFILE` profiler is now on current
  `main` (re-applied from the stale branch via `951b9c1` → `16cbc79`); baselines
  are taken from the up-to-date binary. **Resolved.**
- **#4 (canonical fixture):** `8Q6aVzvu_4V2JAmyk_2025-11-30T17-24-59+0900_27.mcap`
  (2.5 GB) is the canonical fixture; the **warm** full-rewrite number is the
  headline metric. Cold-cache (read ≈ 6 s, ~85 % disk I/O) is recorded separately
  and excluded from the headline, since it measures the disk, not bagwiz.
  **Resolved.**

## Correctness invariant for the backends (carried into Milestone #3)

The reader is **not** globally time-ordered and writers do not enforce monotonic
timestamps, so the invariant the accelerated backends must preserve is the
reader's **emission order** (a sequence-only reorder key). Acceptance for any
non-sequential backend is **byte-identical output** vs the sequential strategy and
vs today's binary (differential oracle), with ASan + TSan as CI gates. Inject and
transform commands are barred from any reordering backend.

## Milestone #3 — PipelinedBackend results

`PipelinedBackend` (`src/core/pipeline/pipelined_backend.cpp`) overlaps the read
and write stages on two threads: the calling thread reads + routes + copies each
kept message into a byte-bounded FIFO queue (`BoundedMessageQueue`, 128 MiB cap)
and a single writer thread drains it. A single consumer over a FIFO preserves the
reader's emission order, so the output is byte-identical to `SequentialBackend`.
The pure-copy trio (`topic drop`/`keep`/`rename`) defaults to it;
`BAGWIZ_BACKEND=sequential|pipelined` overrides per run on the same binary.

Measured on the canonical 2.5 GB fixture, warm cache, same 24-core host,
`topic drop /tf_static -o` (full rewrite), `/usr/bin/time` wall:

| Backend (`BAGWIZ_BACKEND`) | Wall (warm) | CPU                 | Peak RSS | Stage split (BAGWIZ_PROFILE)                                      |
| -------------------------- | ----------- | ------------------- | -------- | ----------------------------------------------------------------- |
| `sequential` (oracle)      | 2.53–2.71 s | ~100 % (1 core)     | ~50 MB   | read 0.78 s (32 %) / write 1.68 s (68 %)                          |
| `pipelined` (trio default) | 1.81–2.02 s | ~150 % (~1.5 cores) | ~208 MB  | read 0.86 s / write 1.79 s, **stage-busy sum 2.65 s > real wall** |

- **Speedup ≈ 1.40×** (best/best 2.53 / 1.81), against the **~1.45× Amdahl
  ceiling** from write-dominance — the pipeline is essentially write-bound, as
  predicted. The reported per-stage "wall" sum (2.65 s) exceeds the real elapsed
  wall (~1.8 s) precisely because read and write now run concurrently; that gap
  is the overlap that buys the speedup.
- **Byte-identical**: both outputs md5 `e505922a4ac60e0c4f00c0c7962a838c`
  (2,975,347,142 bytes), counts identical (31,648 copied / 30 dropped). The
  differential oracle (`expect_bags_equal`) and the per-run md5 in
  `scripts/bench-rewrite.sh` ([B'] section) gate this.
- **RSS cost**: ~50 MB → ~208 MB. The increase is the 128 MiB queue cap plus the
  owned payload copies the threaded hand-off requires (the cost the zero-copy
  Sequential path avoids). Bounded and tunable via the queue cap; a fast reader
  cannot balloon it past the cap thanks to backpressure.
- **Sanitizers**: the pipeline tests pass clean under **ThreadSanitizer** (no data
  races) and **AddressSanitizer + UBSan** (no use-after-free / leaks / UB). No
  sanitizer CI job exists in-repo yet; these were run via a separate colcon build
  base with `-fsanitize=thread` / `-fsanitize=address,undefined`. Note: TSan needs
  ASLR disabled on this kernel (`setarch "$(uname -m)" -R ctest …`) to avoid its
  "unexpected memory mapping" loader abort — that abort is an environment quirk,
  not a race (the thread-free `bag_copy_test` aborts identically without it).

Why only the trio: the 2-stage pipeline reorders nothing, but inject/transform
commands stay on Sequential per the PRD scope (and because they decode, which the
decoders are not thread-safe for). A decode-worker / parallel-map backend remains
unjustified — the process stage is 0 % here.
