// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/cam_info_dump.hpp"

#include "bagwiz/core/base/atomic_write.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/camera_calibration_yaml.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <fmt/core.h>

#include <cinttypes>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;

constexpr const char * kLogger = "bagwiz.cmd.cam-info.dump";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// True when two calibrations would produce the same YAML. Used to notice a bag
// whose CameraInfo stream is not constant, which a single YAML cannot represent.
[[nodiscard]] bool same_calibration(const img::CameraInfo & a, const img::CameraInfo & b)
{
  return a.width == b.width && a.height == b.height && a.distortion_model == b.distortion_model &&
         a.d == b.d && a.k == b.k && a.r == b.r && a.p == b.p;
}

}  // namespace

int run_cam_info_dump(const CamInfoDumpArgs & args)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();

  const io::TopicInfo * info = nullptr;
  std::string camera_info_topics;
  for (const auto & t : reader->topics()) {
    if (t.name == args.topic) {
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
      kLogger, "Topic '%s' is not present in %s. Available %s topic(s): %s", args.topic.c_str(),
      args.input_path.c_str(), kCameraInfoType,
      camera_info_topics.empty() ? "(none)" : camera_info_topics.c_str());
    return 1;
  }
  if (info->type != kCameraInfoType) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' has type '%s', expected '%s'.", args.topic.c_str(), info->type.c_str(),
      kCameraInfoType);
    return 1;
  }

  // Read the topic's messages. A CameraInfo stream is small and the filter is
  // pushed into the storage layer, so this does not scan the whole bag.
  io::ReadFilter filter;
  filter.topics = {args.topic};
  reader->set_filter(filter);

  std::optional<img::CameraInfo> first;
  std::uint64_t count = 0;
  bool varies = false;
  try {
    io::RawMessage raw;
    while (reader->next(raw)) {
      if (raw.topic->name != args.topic) {
        continue;
      }
      const auto parsed = img::extract_camera_info(raw.payload);
      if (!parsed.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Could not parse a message on '%s' as %s: %s", args.topic.c_str(),
          kCameraInfoType, parsed.error.c_str());
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
    BAGWIZ_LOG_ERROR(kLogger, "Failed reading '%s': %s", args.topic.c_str(), e.what());
    return 1;
  }

  if (!first.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' carried no messages, so it has no calibration to write.",
      args.topic.c_str());
    return 1;
  }
  // One YAML cannot represent a calibration that changes mid-bag; say which one
  // was taken rather than let the others vanish silently.
  if (varies) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "Topic '%s' does not carry a constant calibration across its %" PRIu64
      " message(s); the first message's calibration was used.",
      args.topic.c_str(), count);
  }

  img::CameraCalibration out;
  out.width = first->width;
  out.height = first->height;
  out.distortion_model = first->distortion_model;
  out.d = first->d;
  out.k = first->k;
  out.r = first->r;
  out.p = first->p;
  // p is copied, not recomputed: `cam-info recompute-p` is the command for that,
  // and doing it here would make a dump silently disagree with the bag.
  //
  // camera_name is not a CameraInfo field, so the bag cannot supply one. Leave
  // it unset rather than invent a name from the topic or frame_id: the key is
  // optional, and a wrong name is worse than an absent one.

  const std::string yaml = img::emit_camera_calibration_yaml(out);

  if (!args.output_path.has_value()) {
    // The calibration is this command's data output, so it goes to stdout while
    // every diagnostic above went to stderr -- `bagwiz cam-info dump <bag>
    // <topic> > calib.yaml` is pipe-clean. See core/base/logging.hpp.
    fmt::print(stdout, "{}", yaml);
    BAGWIZ_LOG_INFO(
      kLogger, "cam-info dump: wrote '%s' from '%s' to stdout (%" PRIu64 " message(s) read).",
      args.topic.c_str(), args.input_path.string().c_str(), count);
    return 0;
  }

  // Claim the output only now, once the run is certain to produce a calibration:
  // prepare_output_path() deletes an existing path under --overwrite, so doing
  // this any earlier would let a bad <topic> destroy the user's -o file and then
  // fail. Only the read can tell an empty topic from a populated one, so every
  // refusal above has to come first.
  if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }

  std::string error;
  if (!core::write_file_atomically(*args.output_path, yaml, error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", args.output_path->c_str(), error.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "cam-info dump: wrote '%s' from '%s' on %s (%" PRIu64 " message(s) read).",
    args.output_path->string().c_str(), args.topic.c_str(), args.input_path.string().c_str(),
    count);
  return 0;
}

}  // namespace bagwiz::commands
