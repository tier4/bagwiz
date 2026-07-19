// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "traj_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

namespace bagwiz::commands
{

std::unique_ptr<core::decoder::Decoder> open_topic_decoder(
  io::BagReader & reader, const std::string & topic, const char * logger)
{
  for (const auto & ti : reader.topics()) {
    if (ti.name != topic) {
      continue;
    }
    auto open = core::decoder::open_decoder(ti);
    if (!open.ok()) {
      BAGWIZ_LOG_ERROR(
        logger, "Could not open decoder for topic '%s': %s", ti.name.c_str(), open.error.c_str());
      return nullptr;
    }
    return std::move(open.decoder);
  }
  BAGWIZ_LOG_ERROR(logger, "Could not open decoder for topic '%s'.", topic.c_str());
  return nullptr;
}

bool write_tum_file(
  const std::filesystem::path & output_path, std::span<const core::TrajectoryPose> poses,
  const char * logger)
{
  std::ofstream out(output_path, std::ios::out | std::ios::trunc);
  if (!out) {
    BAGWIZ_LOG_ERROR(logger, "Failed to open output path %s for writing", output_path.c_str());
    return false;
  }
  core::write_tum(out, poses);
  out.close();
  return true;
}

}  // namespace bagwiz::commands
