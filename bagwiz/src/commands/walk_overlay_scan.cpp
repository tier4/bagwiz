// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_overlay_scan.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

}  // namespace

void scan_overlay_inputs(
  const std::filesystem::path & input, const std::vector<std::string> & pcd_topics,
  const std::atomic<bool> & cancel, const std::function<void(double)> & progress,
  OverlayScanResult & out)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    out.error = "failed to open '" + input.string() + "': " + e.what();
    return;
  }

  std::unordered_map<std::string, std::size_t> pcd_slot;
  for (std::size_t i = 0; i < pcd_topics.size(); ++i) {
    pcd_slot.emplace(pcd_topics[i], i);
  }
  out.entries.assign(pcd_topics.size(), {});

  io::ReadFilter filter;
  for (const auto & t : reader->topics()) {
    if (t.type != kTfMessageType) {
      continue;
    }
    filter.topics.push_back(t.name);
    // Only TF rows need their payload; point-cloud rows are timestamp-only,
    // which lets the storage layer skip their (potentially huge) BLOBs.
    filter.payload_topics.push_back(t.name);
  }
  if (filter.payload_topics.empty()) {
    out.error = "no tf2_msgs/msg/TFMessage topics found; cannot resolve point-cloud transform";
    return;
  }
  for (const auto & topic : pcd_topics) {
    filter.topics.push_back(topic);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
  for (const auto & t : reader->topics()) {
    if (t.type != kTfMessageType) {
      continue;
    }
    auto open = core::decoder::open_decoder(t);
    if (!open.ok()) {
      out.error = "could not open decoder for TF topic '" + t.name + "': " + open.error;
      return;
    }
    decoders.emplace(t.name, std::move(open.decoder));
  }

  // The time extent comes from the storage's timestamp index (no scan) and
  // turns the current row's timestamp into a 0..1 progress fraction.
  const auto extent = reader->compute_time_extent();
  int last_percent = -1;

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (cancel.load(std::memory_order_relaxed)) {
        return;  // cancelled: the caller discards `out`
      }
      if (progress && extent.has_data && extent.end_ns > extent.start_ns) {
        const double fraction = std::clamp(
          static_cast<double>(raw.timestamp_ns - extent.start_ns) /
            static_cast<double>(extent.end_ns - extent.start_ns),
          0.0, 1.0);
        const int percent = static_cast<int>(fraction * 100.0);
        if (percent != last_percent) {
          last_percent = percent;
          progress(fraction);
        }
      }

      if (const auto it = pcd_slot.find(raw.topic->name); it != pcd_slot.end()) {
        out.entries[it->second].push_back({raw.timestamp_ns, raw.timestamp_ns});
        continue;
      }
      const auto dec = decoders.find(raw.topic->name);
      if (dec == decoders.end()) {
        continue;
      }
      const auto decoded = dec->second->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode TF message on '" + raw.topic->name + "': " + decoded.error;
        return;
      }
      const bool is_static = core::is_static_tf_topic(raw.topic->name);
      for (const auto & t : core::extract_tf_message(*decoded.value)) {
        out.tf_buffer.setTransform(t, "bagwiz", is_static);
      }
    }
  } catch (const std::exception & e) {
    out.error = "error reading bag: " + std::string(e.what());
    return;
  }

  if (cancel.load(std::memory_order_relaxed)) {
    return;
  }
  for (std::size_t i = 0; i < pcd_topics.size(); ++i) {
    if (out.entries[i].empty()) {
      out.error = "point-cloud topic '" + pcd_topics[i] + "' has no messages";
      return;
    }
  }
  if (progress && last_percent < 100) {
    progress(1.0);
  }
}

}  // namespace bagwiz::commands
