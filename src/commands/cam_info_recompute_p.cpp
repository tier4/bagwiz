// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_recompute_p.hpp"

#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/cdr_walker/cdr_writer.hpp"
#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/projection_matrix.hpp"
#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;
namespace cdr = bagwiz::core::cdr_walker;

constexpr const char * kLogger = "bagwiz.cmd.cam-info.recompute-p";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// Below this, a change in p is cv::getOptimalNewCameraMatrix drifting between
// OpenCV versions rather than a corrected calibration. Saying so keeps a
// sub-pixel diff from reading as a real fix. Sized from the observed 4.5.4 ->
// 4.13.0 drift (up to 0.77px on a 1920x1280 plumb_bob calibration); a genuinely
// wrong p is off by tens of pixels, far above this.
constexpr double kDriftPx = 1.5;

std::string take_rmw_error()
{
  const rcutils_error_state_t * s = rcutils_get_error_state();
  std::string err = (s != nullptr) ? s->message : "(no error message)";
  rcutils_reset_error();
  return err;
}

// Largest absolute per-entry change between two projection matrices. Every entry
// of p is either a pixel quantity or a fixed 0/1, so the max reads as pixels.
[[nodiscard]] double max_abs_delta(
  const std::array<double, 12> & before, const std::array<double, 12> & after)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < 12; ++i) {
    worst = std::max(worst, std::abs(after[i] - before[i]));
  }
  return worst;
}

// Explain how far p moved so a sub-pixel result is legible as OpenCV version
// drift rather than looking like the command did nothing. See kDriftPx.
void log_delta(const double delta, const double alpha)
{
  const std::string backend = img::projection_backend_version();
  if (delta == 0.0) {
    BAGWIZ_LOG_INFO(
      kLogger, "p was already the alpha=%.2f solution (OpenCV %s); it is unchanged.", alpha,
      backend.c_str());
  } else if (delta < kDriftPx) {
    BAGWIZ_LOG_INFO(
      kLogger,
      "p changed by at most %.3f px (alpha=%.2f, OpenCV %s). A sub-pixel change like this is "
      "cv::getOptimalNewCameraMatrix differing across OpenCV versions, not a corrected "
      "calibration.",
      delta, alpha, backend.c_str());
  } else {
    BAGWIZ_LOG_INFO(
      kLogger, "p changed by up to %.3f px (alpha=%.2f, OpenCV %s).", delta, alpha,
      backend.c_str());
  }
}

[[nodiscard]] bool is_yaml_path(const std::filesystem::path & path)
{
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return ext == ".yaml" || ext == ".yml";
}

[[nodiscard]] img::ProjectionMatrixInput to_projection_input(const img::CameraCalibration & c)
{
  img::ProjectionMatrixInput in;
  in.k = c.k;
  in.r = c.r;
  in.p = c.p;
  in.d = c.d;
  in.distortion_model = c.distortion_model;
  in.width = c.width;
  in.height = c.height;
  return in;
}

// --- YAML mode -------------------------------------------------------------

// Write `contents` to `path` via a sibling temporary + rename, so a failure
// partway through cannot leave a half-written calibration behind. Mirrors the
// atomicity core::write_bag_inplace() gives the bag paths.
[[nodiscard]] bool write_file_atomically(
  const std::filesystem::path & path, const std::string & contents, std::string & error)
{
  const std::filesystem::path tmp = path.parent_path() / (path.filename().string() + ".bagwiz.tmp");
  try {
    {
      std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
      if (!out) {
        error = "could not open '" + tmp.string() + "' for writing";
        return false;
      }
      out << contents;
      out.flush();
      if (!out) {
        error = "failed while writing '" + tmp.string() + "'";
        return false;
      }
    }
    std::filesystem::rename(tmp, path);
  } catch (const std::exception & e) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    error = e.what();
    return false;
  }
  return true;
}

// Recompute the projection_matrix block of a camera_calibration file and
// re-emit it. Every other value is carried across unchanged.
int run_yaml_mode(const CamInfoRecomputePArgs & args)
{
  if (!args.topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "'%s' is a calibration YAML, which carries no topics, so --topics does not apply. Drop "
      "--topics, or pass a bag as <input>.",
      args.input_path.string().c_str());
    return 1;
  }

  const auto parsed = img::parse_camera_calibration_yaml(args.input_path);
  if (!parsed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load calibration from '%s': %s", args.input_path.c_str(),
      parsed.error.c_str());
    return 1;
  }

  const auto computed =
    img::compute_projection_matrix(to_projection_input(*parsed.calibration), args.alpha);
  if (!computed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Cannot recompute p for '%s': %s", args.input_path.c_str(), computed.error.c_str());
    return 1;
  }

  const std::filesystem::path destination =
    args.output_path.has_value() ? *args.output_path : args.input_path;
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(destination, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }

  log_delta(max_abs_delta(parsed.calibration->p, *computed.p), args.alpha);

  // Immutable update: emit a copy carrying the new p rather than mutating the
  // parsed calibration in place.
  img::CameraCalibration updated = *parsed.calibration;
  updated.p = *computed.p;

  std::string error;
  if (!write_file_atomically(destination, img::emit_camera_calibration_yaml(updated), error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", destination.c_str(), error.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "cam-info recompute-p: rewrote projection_matrix in '%s'.",
    destination.string().c_str());
  return 0;
}

// --- bag -> YAML export ----------------------------------------------------

// True when two calibrations would produce the same YAML. Used to notice a bag
// whose CameraInfo stream is not constant, which a single YAML cannot represent.
[[nodiscard]] bool same_calibration(const img::CameraInfo & a, const img::CameraInfo & b)
{
  return a.width == b.width && a.height == b.height && a.distortion_model == b.distortion_model &&
         a.d == b.d && a.k == b.k && a.r == b.r && a.p == b.p;
}

// Export one CameraInfo topic's calibration from a bag as a camera_calibration
// YAML, with p recomputed. Reached when <input> is a bag but -o names a
// .yaml/.yml file: the request is "give me the calibration, not a rewritten
// bag", so nothing bag-shaped is written.
int run_bag_to_yaml_mode(const CamInfoRecomputePArgs & args)
{
  const std::filesystem::path & destination = *args.output_path;

  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "'%s' is a bag, so --topics is required to say which CameraInfo topic's calibration to "
      "write to '%s'.",
      args.input_path.string().c_str(), destination.string().c_str());
    return 1;
  }
  // A camera_calibration YAML holds exactly one calibration, so several topics
  // have no single answer. Refuse rather than silently picking one.
  if (args.topics.size() != 1) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "-o '%s' is a calibration YAML, which holds a single calibration, but %zu topics were given. "
      "Name exactly one topic, or run the command once per topic with a different -o.",
      destination.string().c_str(), args.topics.size());
    return 1;
  }
  const std::string & topic = args.topics.front();

  if (const auto r = core::prepare_output_path(destination, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  const io::TopicInfo * info = nullptr;
  std::string camera_info_topics;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      info = &t;
    }
    if (t.type == kCameraInfoType) {
      if (!camera_info_topics.empty()) {
        camera_info_topics += ", ";
      }
      camera_info_topics += t.name;
    }
  }
  if (info == nullptr) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' is not present in %s. Available %s topic(s): %s", topic.c_str(),
      args.input_path.c_str(), kCameraInfoType,
      camera_info_topics.empty() ? "(none)" : camera_info_topics.c_str());
    return 1;
  }
  if (info->type != kCameraInfoType) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' has type '%s', expected '%s'.", topic.c_str(), info->type.c_str(),
      kCameraInfoType);
    return 1;
  }

  // Read the topic's messages. A CameraInfo stream is small and the filter is
  // pushed into the storage layer, so this does not scan the whole bag.
  io::ReadFilter filter;
  filter.topics = {topic};
  reader->set_filter(filter);

  std::optional<img::CameraInfo> first;
  std::uint64_t count = 0;
  bool varies = false;
  try {
    io::RawMessage raw;
    while (reader->next(raw)) {
      if (raw.topic->name != topic) {
        continue;
      }
      const auto parsed = img::extract_camera_info(raw.payload);
      if (!parsed.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Could not parse a message on '%s' as %s: %s", topic.c_str(), kCameraInfoType,
          parsed.error.c_str());
        return 1;
      }
      ++count;
      if (!first.has_value()) {
        first = *parsed.info;
      } else if (!varies && !same_calibration(*first, *parsed.info)) {
        varies = true;
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed reading '%s': %s", topic.c_str(), e.what());
    return 1;
  }

  if (!first.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' carried no messages, so it has no calibration to write.", topic.c_str());
    return 1;
  }
  // One YAML cannot represent a calibration that changes mid-bag; say which one
  // was taken rather than let the others vanish silently.
  if (varies) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "Topic '%s' does not carry a constant calibration across its %" PRIu64
      " message(s); the first message's calibration was used.",
      topic.c_str(), count);
  }

  img::ProjectionMatrixInput in;
  in.k = first->k;
  in.r = first->r;
  in.p = first->p;
  in.d = first->d;
  in.distortion_model = first->distortion_model;
  in.width = first->width;
  in.height = first->height;

  const auto computed = img::compute_projection_matrix(in, args.alpha);
  if (!computed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Cannot recompute p for '%s': %s", topic.c_str(), computed.error.c_str());
    return 1;
  }

  log_delta(max_abs_delta(first->p, *computed.p), args.alpha);

  img::CameraCalibration out;
  out.width = first->width;
  out.height = first->height;
  out.distortion_model = first->distortion_model;
  out.d = first->d;
  out.k = first->k;
  out.r = first->r;
  out.p = *computed.p;
  // camera_name is not a CameraInfo field, so the bag cannot supply one. Leave
  // it unset rather than invent a name from the topic or frame_id: the key is
  // optional, and a wrong name is worse than an absent one.

  std::string error;
  if (!write_file_atomically(destination, img::emit_camera_calibration_yaml(out), error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", destination.c_str(), error.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "cam-info recompute-p: wrote '%s' from '%s' on %s (%" PRIu64
    " message(s) read); the bag was not modified.",
    destination.string().c_str(), topic.c_str(), args.input_path.string().c_str(), count);
  return 0;
}

// --- bag mode --------------------------------------------------------------

// Recomputes p on one or more CameraInfo topics. Non-target topics pass through
// verbatim. For a target topic the original CDR is deserialized into a typed
// sensor_msgs/msg/CameraInfo (so header, binning, and roi survive untouched), p
// is recomputed from that same message's own k/d/width/height, and the message
// is re-serialized. transform() runs on the single producer thread, so the
// scratch state below is never shared across threads.
class CamInfoRecomputePProcessor : public core::pipeline::Processor
{
public:
  CamInfoRecomputePProcessor(
    const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport,
    double alpha)
  : targets_(topics.begin(), topics.end()), typesupport_(typesupport), alpha_(alpha)
  {
    for (const auto & topic : topics) {
      rewritten_.emplace(topic, 0);
    }
  }

  [[nodiscard]] std::uint64_t rewritten_count(const std::string & topic) const
  {
    const auto it = rewritten_.find(topic);
    return it == rewritten_.end() ? 0 : it->second;
  }

  // Largest change applied to any message's p over the pass, for log_delta().
  [[nodiscard]] double max_delta() const { return max_delta_; }

  [[nodiscard]] core::pipeline::Emit route(const std::string & in_topic) const override
  {
    return core::pipeline::Emit{true, in_topic};
  }

  [[nodiscard]] bool transforms() const override { return true; }

  [[nodiscard]] core::pipeline::TransformAction transform(
    const std::string & in_topic, std::span<const std::byte> in,
    std::vector<std::byte> & out) const override
  {
    if (targets_.find(in_topic) == targets_.end()) {
      return core::pipeline::TransformAction::kPassthrough;
    }

    // Wrap the reader's payload without taking ownership (no _fini on this view;
    // the bytes belong to the reader).
    rmw_serialized_message_t in_view = rmw_get_zero_initialized_serialized_message();
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast) rmw API is non-const but only reads.
    in_view.buffer = const_cast<std::uint8_t *>(reinterpret_cast<const std::uint8_t *>(in.data()));
    in_view.buffer_length = in.size();
    in_view.buffer_capacity = in.size();
    in_view.allocator = rcutils_get_default_allocator();

    sensor_msgs::msg::CameraInfo msg;
    if (rmw_deserialize(&in_view, typesupport_, &msg) != RMW_RET_OK) {
      throw std::runtime_error(
        "failed to deserialize a message on '" + in_topic + "' as " + kCameraInfoType + ": " +
        take_rmw_error());
    }

    img::ProjectionMatrixInput input;
    std::copy(msg.k.begin(), msg.k.end(), input.k.begin());
    std::copy(msg.r.begin(), msg.r.end(), input.r.begin());
    std::copy(msg.p.begin(), msg.p.end(), input.p.begin());
    input.d.assign(msg.d.begin(), msg.d.end());
    input.distortion_model = msg.distortion_model;
    input.width = msg.width;
    input.height = msg.height;

    const std::array<double, 12> new_p = recompute_cached(in_topic, input);

    max_delta_ = std::max(max_delta_, max_abs_delta(input.p, new_p));
    std::copy(new_p.begin(), new_p.end(), msg.p.begin());

    // Direct CDR serialization avoids the per-message rmw_serialize cost. The
    // CameraInfo layout is fixed except for the variable-length D array, so it
    // can be emitted with CdrWriter using the ROS 2 message definition's field
    // order.
    cdr::CdrWriter writer;
    writer.write_i32(msg.header.stamp.sec);
    writer.write_u32(msg.header.stamp.nanosec);
    writer.write_string(msg.header.frame_id);
    writer.write_u32(msg.height);
    writer.write_u32(msg.width);
    writer.write_string(msg.distortion_model);
    writer.write_sequence_length(static_cast<std::uint32_t>(msg.d.size()));
    for (const double v : msg.d) {
      writer.write_f64(v);
    }
    for (const double v : msg.k) {
      writer.write_f64(v);
    }
    for (const double v : msg.r) {
      writer.write_f64(v);
    }
    for (const double v : msg.p) {
      writer.write_f64(v);
    }
    writer.write_u32(msg.binning_x);
    writer.write_u32(msg.binning_y);
    writer.write_u32(msg.roi.x_offset);
    writer.write_u32(msg.roi.y_offset);
    writer.write_u32(msg.roi.height);
    writer.write_u32(msg.roi.width);
    writer.write_bool(msg.roi.do_rectify);
    out = writer.take();

    ++rewritten_.at(in_topic);
    return core::pipeline::TransformAction::kWrite;
  }

private:
  // A CameraInfo stream is near-always constant, so recomputing per message
  // would call into OpenCV tens of thousands of times for one answer. Cache the
  // last input/output pair and reuse it while the calibration holds still.
  [[nodiscard]] std::array<double, 12> recompute_cached(
    const std::string & topic, const img::ProjectionMatrixInput & input) const
  {
    if (cache_valid_ && same_inputs(input, cached_input_)) {
      return cached_p_;
    }
    const auto computed = img::compute_projection_matrix(input, alpha_);
    if (!computed.ok()) {
      throw std::runtime_error(
        "cannot recompute p for a message on '" + topic + "': " + computed.error);
    }
    cached_input_ = input;
    cached_p_ = *computed.p;
    cache_valid_ = true;
    return cached_p_;
  }

  [[nodiscard]] static bool same_inputs(
    const img::ProjectionMatrixInput & a, const img::ProjectionMatrixInput & b)
  {
    return a.width == b.width && a.height == b.height && a.distortion_model == b.distortion_model &&
           a.k == b.k && a.r == b.r && a.p == b.p && a.d == b.d;
  }

  std::unordered_set<std::string> targets_;
  const rosidl_message_type_support_t * typesupport_;
  double alpha_;
  // Mutable because transform() is const per the Processor contract, which also
  // guarantees it runs on the single producer thread -- so these are
  // single-writer and need no synchronization. rewritten_'s keys are fixed at
  // construction; transform() only updates existing values via at().
  mutable std::unordered_map<std::string, std::uint64_t> rewritten_;
  mutable double max_delta_ = 0.0;
  mutable img::ProjectionMatrixInput cached_input_;
  mutable std::array<double, 12> cached_p_{};
  mutable bool cache_valid_ = false;
};

// One full pass: open `input_path`, declare every topic verbatim (the topic type
// is unchanged), then stream-copy -- recomputing p on each topic in
// `target_topics` and forwarding everything else untouched. The writer factory
// is parameterised so the in-place path can hand in a tmp location.
int execute_pass(
  const std::filesystem::path & input_path, const std::vector<std::string> & target_topics,
  const rosidl_message_type_support_t * typesupport, double alpha,
  const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
    return 1;
  }

  for (const auto & t : topics) {
    try {
      writer->declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  CamInfoRecomputePProcessor processor(target_topics, typesupport, alpha);
  core::pipeline::RewriteCounts counts;
  try {
    auto backend = core::pipeline::make_backend(core::pipeline::BackendKind::Pipelined);
    counts =
      core::pipeline::run_pipeline(*reader, *writer, processor, *backend, "cam-info recompute-p");
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "cam-info recompute-p read/write failed: %s", e.what());
    return 1;
  }

  try {
    writer->close();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
    return 1;
  }

  const std::uint64_t forwarded = counts.copied - counts.transformed;
  std::string topic_list;
  for (const auto & t : target_topics) {
    if (!topic_list.empty()) {
      topic_list += ", ";
    }
    topic_list += t;
  }
  BAGWIZ_LOG_INFO(
    kLogger,
    "cam-info recompute-p: recomputed p on %" PRIu64
    " message(s) across %zu topic(s) [%s], "
    "copied %" PRIu64 " other message(s) verbatim.",
    counts.transformed, target_topics.size(), topic_list.c_str(), forwarded);
  if (counts.transformed > 0) {
    log_delta(processor.max_delta(), alpha);
  }
  // Surface any target that carried no messages so a typo'd-but-existing or
  // empty topic does not silently no-op when the others were rewritten.
  for (const auto & t : target_topics) {
    if (processor.rewritten_count(t) == 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "Topic '%s' carried no messages; its p was left unchanged.", t.c_str());
    }
  }
  return 0;
}

int run_bag_mode(const CamInfoRecomputePArgs & args)
{
  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "'%s' is a bag, so --topics is required to say which CameraInfo topic's p to recompute. Pass "
      "-t/--topics <topic>..., or pass a .yaml calibration file as <input>.",
      args.input_path.string().c_str());
    return 1;
  }

  // 1. Inspect the bag and confirm every requested <topic> exists and is a
  //    CameraInfo topic, before anything is written.
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  std::unordered_map<std::string, const io::TopicInfo *> topics_by_name;
  std::string camera_info_topics;
  for (const auto & t : reader->topics()) {
    topics_by_name.emplace(t.name, &t);
    if (t.type == kCameraInfoType) {
      if (!camera_info_topics.empty()) {
        camera_info_topics += ", ";
      }
      camera_info_topics += t.name;
    }
  }

  // Validate each requested topic, deduplicating while preserving command-line
  // order. Collect all failures before bailing so one run reports every bad
  // topic.
  std::vector<std::string> target_topics;
  std::unordered_set<std::string> seen;
  bool all_valid = true;
  for (const auto & topic : args.topics) {
    if (!seen.insert(topic).second) {
      continue;  // duplicate on the command line; validated on its first occurrence
    }
    const auto it = topics_by_name.find(topic);
    if (it == topics_by_name.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s.", topic.c_str(), args.input_path.c_str());
      all_valid = false;
      continue;
    }
    if (it->second->type != kCameraInfoType) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' has type '%s', expected '%s'.", topic.c_str(),
        it->second->type.c_str(), kCameraInfoType);
      all_valid = false;
      continue;
    }
    target_topics.push_back(topic);
  }
  if (!all_valid) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Available %s topic(s): %s", kCameraInfoType,
      camera_info_topics.empty() ? "(none)" : camera_info_topics.c_str());
    return 1;
  }

  // Release the inspection reader before opening the read/write passes.
  reader.reset();

  // 2. Load the introspection typesupport used for both deserialize and
  //    serialize of the CameraInfo payloads.
  const auto intro = core::load_introspection(kCameraInfoType);
  if (!intro.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load introspection typesupport for %s: %s", kCameraInfoType,
      intro.error.c_str());
    return 1;
  }

  // 3a. Explicit -o: write a fresh bag, leaving <input> untouched.
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    auto make_writer = [&]() {
      io::CreateOptions copts;
      copts.format = io::Format::Auto;
      copts.layout = io::Layout::Auto;
      copts.mcap_compression = "none";
      return io::open_write(*args.output_path, copts);
    };
    return execute_pass(args.input_path, target_topics, intro.typesupport, args.alpha, make_writer);
  }

  // 3b. In-place: pin format/layout to <input>'s identity (the tmp suffix that
  //     write_bag_inplace uses cannot be interpreted by Auto resolution).
  const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not detect storage format of input bag '%s'.",
      args.input_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  int pass_status = 0;
  try {
    core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_pass(
        args.input_path, target_topics, intro.typesupport, args.alpha,
        [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("cam-info recompute-p: pass failed; aborting in-place swap");
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;
    }
    BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace

int run_cam_info_recompute_p(const CamInfoRecomputePArgs & args)
{
  // Extensions pick the mode. <input> says where the calibration comes from: a
  // .yaml/.yml file, or else a bag (a directory, or a .mcap/.db3 file). For a bag
  // input, -o then says what to produce -- a .yaml/.yml output means "give me the
  // calibration" rather than "rewrite the bag", so the bag is left alone.
  //
  //   <input>   -o            -> mode
  //   YAML      (none)/YAML   -> rewrite the YAML
  //   bag       (none)/bag    -> rewrite the bag
  //   bag       YAML          -> export the calibration, bag untouched
  //   YAML      bag           -> refused below: a YAML has no messages to build one from
  const bool input_is_yaml = is_yaml_path(args.input_path);
  const bool output_is_yaml = args.output_path.has_value() && is_yaml_path(*args.output_path);

  if (input_is_yaml) {
    if (args.output_path.has_value() && !output_is_yaml) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "<input> '%s' is a calibration YAML but -o '%s' is not, and a YAML carries no messages to "
        "build a bag from. Give -o a .yaml/.yml path, or pass a bag as <input>.",
        args.input_path.string().c_str(), args.output_path->string().c_str());
      return 1;
    }
    return run_yaml_mode(args);
  }
  if (output_is_yaml) {
    return run_bag_to_yaml_mode(args);
  }
  return run_bag_mode(args);
}

}  // namespace bagwiz::commands
