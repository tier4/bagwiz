# db3 Page-Size Sweep

Sweep behind the SQLite page size used for newly written db3 bags
(`resolve_db3_page_size()` in `bagwiz_io/src/io/env_tuning.hpp`, applied by
`SqliteFileWriter`'s constructor and overridable via `BAGWIZ_DB3_PAGE_SIZE`).

SQLite defaults to 4 KiB pages. Rosbag payloads are large BLOBs, so a 2 MB
point cloud spills across roughly 500 chained overflow pages at 4 KiB versus
about 64 at 32 KiB.

## Setup

|         |                                                                            |
| ------- | -------------------------------------------------------------------------- |
| Host    | 24 physical cores, 62 GB RAM, NVMe (`/dev/nvme0n1p2`)                      |
| Command | `topic rename` — a full-copy rewrite, so every message is read and written |
| Cold    | `posix_fadvise(POSIX_FADV_DONTNEED)` over the input before each run        |
| Reps    | 3 per cell; the table reports medians (run-to-run spread was 0.01–0.05 s)  |

Three fixtures, chosen to bracket the shapes where page size behaves
differently:

| Fixture | Shape                                          | Why                                              |
| ------- | ---------------------------------------------- | ------------------------------------------------ |
| `tiny`  | 200 topics x 1,000,000 msg x 100 B (134 MB)    | Small-message-heavy: per-page overhead dominates |
| `small` | 630 topics x 91,421 msg, ~22 KB mean (2.07 GB) | A real Autoware recording, trimmed to 6 s        |
| `adv`   | 50 topics x 20,000 msg x 70 KiB (1.48 GB)      | Adversarial: messages straddle a 64 KiB page     |

## Results

Wall time (median of 3), and output size relative to the 4 KiB baseline:

| Fixture | 4096           | 16384             | **32768**             | 65536             |
| ------- | -------------- | ----------------- | --------------------- | ----------------- |
| `tiny`  | 1.070 s / ±0 % | 1.030 s / −1.85 % | **1.050 s / −2.03 %** | 1.070 s / −1.95 % |
| `small` | 1.860 s / ±0 % | 1.720 s / +0.36 % | **1.690 s / +0.90 %** | 1.690 s / +1.62 % |
| `adv`   | 1.280 s / ±0 % | 1.130 s / ±0 %    | **1.110 s / −2.22 %** | 1.110 s / +1.62 % |

Speedup over the 4 KiB default at 32 KiB: **−1.9 % (`tiny`), −9.1 % (`small`),
−13.3 % (`adv`)**.

## Why 32 KiB and not 64 KiB

32 KiB is better than or equal to 64 KiB on **every** fixture, in both
dimensions — it is not a trade-off:

- **Speed.** 32 KiB ties 64 KiB on `small` and `adv`. On `tiny` it is 1.9 %
  faster while 64 KiB gives no gain at all over the 4 KiB default.
- **Size.** 32 KiB produces a smaller file than 64 KiB on all three: +0.90 %
  vs +1.62 % (`small`), −2.22 % vs +1.62 % (`adv`), −2.03 % vs −1.95 % (`tiny`).

Issue #342 proposed 64 KiB based on a synthetic insert loop, where
32 KiB → 64 KiB was worth a further ~1.3 % on the write stage alone. In a real
rewrite that margin disappears into the read and pipeline costs, leaving only
the extra file size.

## The page-straddling case, which turned out not to matter

The `adv` fixture exists to test a specific worry: a message slightly larger
than one page should waste most of a second overflow page. At 64 KiB a 70 KiB
payload looked like it might cost ~83 % overhead.

It does not. SQLite stores a leading portion of the row inline in the b-tree
leaf (about 8 KB at a 64 KiB page size) and only the remainder goes to the
overflow chain, so 70 KiB fits in one overflow page rather than spilling into a
mostly-empty second one. Measured overhead at 64 KiB was **+1.6 %**, in line
with the other fixtures.

## Compatibility

`page_size` is a field in the SQLite file header and readers negotiate it
automatically. Verified by having upstream rosbag2 read a 32 KiB-page bag
written by bagwiz:

```console
$ ros2 bag info out/
Storage id:        sqlite3
Duration:          5.999998329s
Messages:          91421
```

Only newly written bags are affected; existing bags keep whatever page size
they were created with.

## Knobs that are not worth changing

Measured on the same fixtures and left alone:

- **Transaction batch size** (`kBatchSize = 1024`): 256 / 1024 / 4096 /
  unbounded were within noise of each other.
- **Deferring `timestamp_idx` to `close()`** (as `topic_timestamp_idx` already
  is): no measurable difference.
