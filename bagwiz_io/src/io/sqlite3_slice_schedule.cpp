// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "sqlite3_slice_schedule.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace bagwiz::io::detail
{

std::vector<SliceRef> build_slice_schedule(const SliceScheduleParams & params)
{
  if (params.target_slice_bytes == 0 || params.max_slices == 0) {
    return {};
  }

  // Clamp the bag extent by the caller's filter. end_ns is exclusive, so the
  // inclusive upper bound is one below it; guard the underflow explicitly
  // rather than relying on INT64_MIN never showing up.
  std::int64_t lo = params.extent_start_ns;
  if (params.filter_start_ns.has_value()) {
    lo = std::max(lo, *params.filter_start_ns);
  }
  std::int64_t hi = params.extent_end_ns;
  if (params.filter_end_ns.has_value()) {
    if (*params.filter_end_ns == std::numeric_limits<std::int64_t>::min()) {
      return {};
    }
    hi = std::min(hi, *params.filter_end_ns - 1);
  }
  if (lo > hi) {
    return {};
  }

  // Distinct nanoseconds the schedule may split across. Computed in unsigned
  // arithmetic because hi - lo overflows a signed int64 for a wide extent.
  const std::uint64_t span = static_cast<std::uint64_t>(hi) - static_cast<std::uint64_t>(lo) + 1ULL;

  std::uint64_t count =
    static_cast<std::uint64_t>(params.file_size_bytes) / params.target_slice_bytes;
  count = std::min<std::uint64_t>(count, params.max_slices);
  count = std::min<std::uint64_t>(count, span);
  if (count < 2ULL) {
    return {};  // one slice is just the serial scan with extra machinery
  }

  // Integer step >= 1 because count <= span, so interior boundaries are
  // strictly increasing. Truncation leaves at most `count` ns unassigned at
  // the top, which the last slice absorbs by keeping the caller's upper bound.
  const std::uint64_t step = span / count;

  std::vector<SliceRef> slices;
  slices.reserve(static_cast<std::size_t>(count));
  for (std::uint64_t k = 0; k < count; ++k) {
    SliceRef slice;
    slice.start_ns = k == 0 ? params.filter_start_ns
                            : std::optional<std::int64_t>(lo + static_cast<std::int64_t>(step * k));
    slice.end_ns = k + 1 == count
                     ? params.filter_end_ns
                     : std::optional<std::int64_t>(lo + static_cast<std::int64_t>(step * (k + 1)));
    slices.push_back(slice);
  }
  return slices;
}

}  // namespace bagwiz::io::detail
