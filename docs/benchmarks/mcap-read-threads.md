# MCAP Read-Thread Sweep

Sweep that set the default decompress-worker count of the parallel indexed
mcap read path (`resolve_read_threads()` in
`bagwiz_io/src/io/mcap_reader.cpp`, overridable via `BAGWIZ_READ_THREADS`).

## Setup

|         |                                                     |
| ------- | --------------------------------------------------- |
| Host    | Intel Core i9-14900K (8P+16E, 24 physical cores)    |
| Fixture | A 4.7 GB zstd-compressed MCAP recording             |
| Command | `check broken --deep` (read-only, decompress-bound) |
| Cache   | warm (OS page cache primed before timed runs)       |

## Results

| `BAGWIZ_READ_THREADS` | Wall   |
| --------------------- | ------ |
| 0 (synchronous)       | 1.16 s |
| 2                     | 0.69 s |
| 4 (previous default)  | 0.45 s |
| 8 (current default)   | 0.37 s |
| 16                    | 0.43 s |

8 workers is ~15 % faster than the previous default of 4 on read-heavy
commands, and 16 already regresses on this 24-core host, so the default sits
at the knee of the curve.

## Small machines

The sweep host has 24 physical cores; a machine with fewer cores can prefer
fewer workers, so the default is capped at the host's hardware concurrency
(`min(8, hardware_concurrency())`). Hosts with 4 or fewer hardware threads
therefore keep the previous behavior, and `BAGWIZ_READ_THREADS` still
overrides the default explicitly in both directions.

## Memory cost

The `ChunkPrefetcher` lookahead window scales with the worker count
(`num_threads + 2` retained decompressed chunks), so moving the default from
4 to 8 grows peak usage by ~4 retained chunks — on the order of 10 MB at
typical chunk sizes. Negligible next to the bag sizes this path targets.
