// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_image_pairer.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

void ScanImagePairer::push_scan(
  std::int64_t stamp_ns, std::vector<std::array<float, 3>> world_points)
{
  scans_.push_back(ScanSlot{stamp_ns, std::move(world_points)});
  while (scans_.size() > kScanSlotHistorySize) {
    scans_.pop_front();
  }
}

void ScanImagePairer::push_image(PendingImage image)
{
  pending_images_.push_back(std::move(image));
}

bool ScanImagePairer::has_decidable() const
{
  if (pending_images_.empty()) {
    return false;
  }
  if (finished_) {
    return true;
  }
  return std::any_of(scans_.begin(), scans_.end(), [&](const ScanSlot & s) {
    return s.stamp_ns >= pending_images_.front().stamp_ns;
  });
}

ScanImagePairer::Decision ScanImagePairer::decide_front()
{
  const auto & image = pending_images_.front();
  const ScanSlot * best = nullptr;
  for (const auto & slot : scans_) {
    if (
      best == nullptr ||
      std::abs(slot.stamp_ns - image.stamp_ns) < std::abs(best->stamp_ns - image.stamp_ns)) {
      best = &slot;
    }
  }
  std::span<const std::array<float, 3>> dynamic;
  if (best != nullptr && std::abs(best->stamp_ns - image.stamp_ns) <= pair_window_ns_) {
    dynamic = best->world_points;
  }
  Decision decision{std::move(pending_images_.front()), dynamic};
  pending_images_.pop_front();
  return decision;
}

}  // namespace bagwiz::core::slam
