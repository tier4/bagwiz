// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_recompute_p.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/atomic_write.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/image/projection_matrix.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "cam_info_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <sensor_msgs/msg/camera_info.hpp>

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
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;

constexpr const char * kLogger = "bagwiz.cmd.cam-info.recompute-p";

// Below this, a change in p is cv::getOptimalNewCameraMatrix drifting between
// OpenCV versions rather than a corrected calibration. Saying so keeps a
// sub-pixel diff from reading as a real fix. Sized from the observed 4.5.4 ->
// 4.13.0 drift (up to 0.77px on a 1920x1280 plumb_bob calibration); a genuinely
// wrong p is off by tens of pixels, far above this.
constexpr double kDriftPx = 1.5;

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
  if (!core::write_file_atomically(
        destination, img::emit_camera_calibration_yaml(updated), error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", destination.c_str(), error.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "cam-info recompute-p: rewrote projection_matrix in '%s'.",
    destination.string().c_str());
  return 0;
}

// --- bag mode --------------------------------------------------------------

// Recomputes p on one or more CameraInfo topics: the shared CameraInfoProcessor
// skeleton routes every topic through under its own name and round-trips each
// target payload through rmw (so header, binning, and roi survive untouched);
// this class only recomputes p from that same message's own k/d/width/height.
// mutate() runs on the single producer thread, so the scratch state below is
// never shared across threads.
class CamInfoRecomputePProcessor : public CameraInfoProcessor
{
public:
  CamInfoRecomputePProcessor(
    const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport,
    double alpha)
  : CameraInfoProcessor(topics, typesupport), alpha_(alpha)
  {
  }

  // Largest change applied to any message's p over the pass, for log_delta().
  [[nodiscard]] double max_delta() const { return max_delta_; }

protected:
  void mutate(const std::string & in_topic, sensor_msgs::msg::CameraInfo & msg) const override
  {
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

  double alpha_;
  // Mutable because mutate() is const per the CameraInfoProcessor contract,
  // which also guarantees it runs on the single producer thread -- so these are
  // single-writer and need no synchronization.
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
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
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
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());
  const CameraInfoTargets targets =
    validate_camera_info_targets(topics, args.topics, args.input_path, kLogger);
  if (!targets.all_valid) {
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

  // 3. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a fresh bag (a directory output inherits <input>'s backend)
  //    and leaves <input> untouched; otherwise <input> is rewritten atomically
  //    via a sibling tmp, preserving its storage identity.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error = "Could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "cam-info recompute-p: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer) {
      return execute_pass(
        args.input_path, targets.topics, intro.typesupport, args.alpha, open_writer);
    });
}

}  // namespace

int run_cam_info_recompute_p(const CamInfoRecomputePArgs & args)
{
  // <input> alone picks the mode: a .yaml/.yml file is a camera_calibration
  // YAML, anything else is a bag (a directory, or a .mcap/.db3 file). -o only
  // ever says where the result goes -- it is always the same shape as <input>,
  // so its extension chooses nothing. `bagwiz cam-info dump` is the command for
  // pulling a bag's calibration out as a YAML.
  if (is_yaml_path(args.input_path)) {
    return run_yaml_mode(args);
  }
  return run_bag_mode(args);
}

}  // namespace bagwiz::commands
