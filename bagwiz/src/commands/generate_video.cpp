// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video.hpp"

#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <filesystem>
#include <string>

namespace bagwiz::commands
{

int run_generate_video(const GenerateVideoArgs & args)
{
  // Validate the source topic, camera info, point-cloud topics, and the output
  // path before touching anything expensive.
  const auto validation = validate_video_inputs(args);
  if (!validation.ok()) {
    return 1;
  }
  if (const auto err = validate_video_output_path(args.output_path, args.overwrite); !err.empty()) {
    return 1;
  }

  // Pass 1: derive the frame rate and scan the point-cloud overlay topics.
  auto scan = scan_video_inputs(args);
  if (!scan.ok()) {
    return 1;
  }

  // Load camera info + TF before pass 2 so a failure aborts before the encode.
  VideoGeometry geometry;
  if (const auto err = load_video_geometry(args, validation.camera_info_topic, geometry);
      !err.empty()) {
    return 1;
  }

  // Pass 2: decode + encode to a sibling temp path, renamed into place on
  // success. The guard removes the temp on any error exit, so no partial
  // output and no leftover temp survive a failure.
  const std::filesystem::path tmp_path = partial_tmp_path_for(args.output_path);
  PartialFileGuard guard(tmp_path);
  auto reader = open_encode_reader(args);
  if (!reader) {
    return 1;
  }

  FrameNormalizer normalizer(validation.check.topic_type, args.resize_scale);
  VideoFrameEncoder encoder(
    tmp_path, scan.fps, args, geometry.camera_info ? &*geometry.camera_info : nullptr,
    scan.global_property_min, scan.global_property_max);
  if (
    run_encode_pass(
      *reader, args, scan, geometry.camera_info ? &*geometry.camera_info : nullptr,
      geometry.tf_buffer ? &*geometry.tf_buffer : nullptr, normalizer, encoder) != 0) {
    return 1;
  }
  if (const auto err =
        finish_video_encode(encoder, args.topic, tmp_path, args.output_path, args.overwrite);
      !err.empty()) {
    return 1;
  }
  log_video_summary(
    args.output_path, encoder.written(), encoder.width(), encoder.height(), scan.fps);
  return 0;
}

}  // namespace bagwiz::commands
