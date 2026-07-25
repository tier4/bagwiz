// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__SQLITE3_SLICE_SCHEDULE_HPP_
#define IO__SQLITE3_SLICE_SCHEDULE_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// Src-local building block of the parallel db3 read path: turns a bag's time
// extent into the disjoint half-open timestamp ranges the slice workers scan.
namespace bagwiz::io::detail
{

// One half-open time range, [start_ns, end_ns). An unset bound is unbounded on
// that side, matching ReadFilter's convention.
struct SliceRef
{
  std::optional<std::int64_t> start_ns;  // inclusive
  std::optional<std::int64_t> end_ns;    // exclusive
};

struct SliceScheduleParams
{
  // The bag's own extent, i.e. MIN(timestamp) and MAX(timestamp), both
  // inclusive. Only meaningful for a non-empty bag; callers must not build a
  // schedule for a bag whose MIN(timestamp) is NULL.
  std::int64_t extent_start_ns = 0;
  std::int64_t extent_end_ns = 0;
  // The caller's ReadFilter bounds, reproduced verbatim on the outermost
  // slices so the union of slices is exactly the serial scan's row set.
  std::optional<std::int64_t> filter_start_ns;  // inclusive
  std::optional<std::int64_t> filter_end_ns;    // exclusive
  // Slice count is sized off the file so each slice carries roughly
  // `target_slice_bytes` of payload; peak prefetch memory is about
  // (workers + 2) * target_slice_bytes.
  std::uintmax_t file_size_bytes = 0;
  std::uint64_t target_slice_bytes = 0;
  std::size_t max_slices = 0;
};

// Split the (filter-clamped) extent into contiguous half-open timestamp
// ranges, uniform in time. Message rate is close enough to uniform in a real
// bag that equal time slices carry equal bytes (measured +/-3% over a 21 GB
// validation bag), so no quantile probing is needed.
//
// Returns an EMPTY schedule — meaning "stay on the serial path" — when the
// clamped extent is empty, when it spans fewer distinct nanoseconds than the
// requested slice count would need, when the file is too small to warrant more
// than one slice, or when target_slice_bytes is 0.
std::vector<SliceRef> build_slice_schedule(const SliceScheduleParams & params);

}  // namespace bagwiz::io::detail

#endif  // IO__SQLITE3_SLICE_SCHEDULE_HPP_
