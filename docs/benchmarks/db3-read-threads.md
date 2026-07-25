# db3 Read-Thread Sweep

Sweep behind the parallel db3 read path — the timestamp-slice schedule
(`bagwiz_io/src/io/sqlite3_slice_schedule.cpp`) plus the scanner worker pool
(`bagwiz_io/src/io/sqlite3_slice_prefetch.cpp`), both driven by
`BAGWIZ_READ_THREADS`.

## Setup

|         |                                                                               |
| ------- | ----------------------------------------------------------------------------- |
| Host    | 24 physical cores, 62 GB RAM, NVMe (`/dev/nvme0n1p2`)                         |
| Fixture | 21 GB / 904,370 messages / 765 topics / 60 s of a real Autoware recording     |
| Index   | `timestamp_idx` only — no `topic_timestamp_idx`, so the single-statement path |
| Command | Full unfiltered iteration of `open_read()` + `next()`, no write side          |
| Cold    | `posix_fadvise(POSIX_FADV_DONTNEED)` over the whole file before each run      |

## Results

Cold cache — the regime a multi-GB bag actually lives in:

| `BAGWIZ_READ_THREADS` | Wall   | Throughput | Speedup | Peak RSS |
| --------------------- | ------ | ---------- | ------- | -------- |
| 0 (serial)            | 69.1 s | 291 MiB/s  | —       | 352 MB   |
| 2                     | 46.3 s | 434 MiB/s  | 1.49x   | 638 MB   |
| 4                     | 34.3 s | 587 MiB/s  | 2.02x   | 881 MB   |
| 8 (default)           | 33.5 s | 600 MiB/s  | 2.06x   | 1111 MB  |

Warm cache:

| `BAGWIZ_READ_THREADS` | Wall    | Throughput |
| --------------------- | ------- | ---------- |
| 0 (serial)            | 2.365 s | 8501 MiB/s |
| 4                     | 2.170 s | 9264 MiB/s |
| 8 (default)           | 2.145 s | 9373 MiB/s |

Every run emitted a byte-identical message stream: an order-sensitive checksum
over all 904,370 `(timestamp, payload)` pairs was `fa338a28a19aa758` at every
thread count, and a full `convert format` rewrite of the fixture produced a
**byte-identical** 21 GB MCAP under `BAGWIZ_READ_THREADS=0` and `=8`.

End-to-end on a real command, both runs cold, reading 21 GB and writing 21 GB
to the same NVMe:

| `BAGWIZ_READ_THREADS` | `convert format` wall |
| --------------------- | --------------------- |
| 0 (serial)            | 294.2 s               |
| 8 (default)           | 194.1 s               |

## Reading the numbers

Cold-cache scaling saturates at ~4 workers and tracks the device: a single
`sqlite3_step()` loop never builds enough NVMe queue depth to get past
~291 MiB/s, while four independent connections reach ~600 MiB/s. The default of
8 costs nothing measurable over 4 and matches the mcap path's default, so both
backends stay on one knob.

Warm cache is close to a wash (1.10x). The serial path hands out SQLite's own
row buffer zero-copy, whereas the prefetcher must **own** the bytes it read
ahead — that copy eats most of the parallel gain once the device is out of the
picture. The parallel path is still never slower, and the case worth optimizing
is the cold one.

## Memory cost

The lookahead window is `threads + 2` retained slices, each holding roughly
`BAGWIZ_DB3_SLICE_BYTES` (32 MiB by default) of payload, plus the recycled
buffer pool and `std::vector` growth slack. Measured peak RSS on the fixture
grows from 352 MB serial to 1.1 GB at 8 workers. Hosts that care can lower
`BAGWIZ_DB3_SLICE_BYTES`, or drop to `BAGWIZ_READ_THREADS=4` for 881 MB at
essentially the same speed.

## Rejected alternative: rowid-range partitioning

Partitioning by `messages.id` range is the obvious split (rowid ranges are free
to derive from `MIN/MAX(id)`), but it cannot preserve emission order. The
serial scan is `SCAN messages USING INDEX timestamp_idx`, whose key is
`(timestamp, rowid)`, so a partition must come back in that same order. Asking
for it explicitly re-introduces a sorter:

```text
EXPLAIN QUERY PLAN
SELECT topic_id, timestamp, data FROM messages
 WHERE id >= 1 AND id < 100000 ORDER BY timestamp, id;

|--SEARCH messages USING INTEGER PRIMARY KEY (rowid>? AND rowid<?)
`--USE TEMP B-TREE FOR ORDER BY
```

That temp B-tree carries the `data` BLOB and spills gigabytes of payload to the
temp store — the exact cost removed in #339. A timestamp range keeps the index:

```text
EXPLAIN QUERY PLAN
SELECT topic_id, timestamp, data FROM messages
 WHERE timestamp >= ? AND timestamp < ? ORDER BY timestamp;

`--SEARCH messages USING INDEX timestamp_idx (timestamp>? AND timestamp<?)
```

Half-open timestamp ranges are also disjoint and ascending, so the consumer
concatenates them instead of running a k-way merge — there is no tiebreak that
could diverge from the serial order.

## Why uniform time slicing

Boundaries are spaced uniformly in time, with no quantile probing. Splitting
the fixture's extent into 16 equal parts gives 55,660–57,134 rows and
1223–1292 MiB per part (±3%), because sensors publish at fixed rates. Slices
that do come out uneven are absorbed by the work queue, which hands the next
slice to whichever worker finished first.
