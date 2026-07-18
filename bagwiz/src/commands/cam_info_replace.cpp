// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_replace.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "cam_info_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <sensor_msgs/msg/camera_info.hpp>

#include <rmw/types.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;

constexpr const char * kLogger = "bagwiz.cmd.cam-info.replace";

// Rewrites the calibration on one or more CameraInfo topics: the shared
// CameraInfoProcessor skeleton routes every topic through under its own name
// and round-trips each target payload through rmw (so the header, binning, and
// roi survive untouched); this class only overwrites the calibration fields
// from the (single, shared) YAML.
class CamInfoReplaceProcessor : public CameraInfoProcessor
{
public:
  CamInfoReplaceProcessor(
    const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport,
    const img::CameraCalibration & calibration, const std::optional<std::string> & frame_id)
  : CameraInfoProcessor(topics, typesupport), calibration_(calibration), frame_id_(frame_id)
  {
  }

protected:
  // Overwrite only the calibration fields; header (stamp + frame_id),
  // binning_x/y, and roi are left as deserialized.
  void mutate(const std::string &, sensor_msgs::msg::CameraInfo & msg) const override
  {
    msg.height = calibration_.height;
    msg.width = calibration_.width;
    msg.distortion_model = calibration_.distortion_model;
    msg.d.assign(calibration_.d.begin(), calibration_.d.end());
    for (std::size_t i = 0; i < 9; ++i) {
      msg.k[i] = calibration_.k[i];
    }
    for (std::size_t i = 0; i < 9; ++i) {
      msg.r[i] = calibration_.r[i];
    }
    for (std::size_t i = 0; i < 12; ++i) {
      msg.p[i] = calibration_.p[i];
    }
    if (frame_id_.has_value()) {
      msg.header.frame_id = *frame_id_;
    }
  }

private:
  const img::CameraCalibration & calibration_;
  const std::optional<std::string> & frame_id_;
};

// One full pass: open `input_path`, declare every topic verbatim (the topic
// type is unchanged), then stream-copy — rewriting the calibration on each topic
// in `target_topics` and forwarding everything else untouched. The writer factory
// is parameterised so the in-place path can hand in a tmp location.
int execute_pass(
  const std::filesystem::path & input_path, const std::vector<std::string> & target_topics,
  const rosidl_message_type_support_t * typesupport, const img::CameraCalibration & calibration,
  const std::optional<std::string> & frame_id, const io::WriterFactory & open_writer)
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

  CamInfoReplaceProcessor processor(target_topics, typesupport, calibration, frame_id);
  core::pipeline::RewriteCounts counts;
  try {
    auto backend = core::pipeline::make_backend(core::pipeline::BackendKind::Pipelined);
    counts =
      core::pipeline::run_pipeline(*reader, *writer, processor, *backend, "cam-info replace");
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "cam-info replace read/write failed: %s", e.what());
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
    "cam-info replace: rewrote calibration on %" PRIu64
    " message(s) across %zu topic(s) [%s], "
    "copied %" PRIu64 " other message(s) verbatim.",
    counts.transformed, target_topics.size(), topic_list.c_str(), forwarded);
  // Surface any target that carried no messages so a typo'd-but-existing or
  // empty topic does not silently no-op when the others were rewritten.
  for (const auto & t : target_topics) {
    if (processor.rewritten_count(t) == 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "Topic '%s' carried no messages; its calibration was left unchanged.", t.c_str());
    }
  }
  return 0;
}

}  // namespace

int run_cam_info_replace(const CamInfoReplaceArgs & args)
{
  // 1. Inspect the bag and confirm every requested <topic> exists and is a
  //    CameraInfo topic. The CLI guarantees args.topics is non-empty (->required).
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

  // 2. Parse the camera_calibration YAML.
  const auto parsed = img::parse_camera_calibration_yaml(args.yaml_path);
  if (!parsed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load calibration from '%s': %s", args.yaml_path.c_str(),
      parsed.error.c_str());
    return 1;
  }
  const img::CameraCalibration & calibration = *parsed.calibration;

  // 3. Load the introspection typesupport used for both deserialize and
  //    serialize of the CameraInfo payloads.
  const auto intro = core::load_introspection(kCameraInfoType);
  if (!intro.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load introspection typesupport for %s: %s", kCameraInfoType,
      intro.error.c_str());
    return 1;
  }

  // 4. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a fresh bag (format/layout resolved from the output path's
  //    extension) and leaves <input> untouched; otherwise <input> is rewritten
  //    atomically via a sibling tmp, preserving its storage identity.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error = "Could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "cam-info replace: pass failed; aborting in-place swap";
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer) {
      return execute_pass(
        args.input_path, targets.topics, intro.typesupport, calibration, args.frame_id,
        open_writer);
    });
}

}  // namespace bagwiz::commands
