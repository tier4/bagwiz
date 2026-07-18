// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/static_extrinsic.hpp"

#include "bagwiz/core/tf/tf_chain.hpp"

#include <exception>
#include <string>
#include <utility>

namespace bagwiz::core::pointcloud
{

StaticExtrinsicResult resolve_static_extrinsic(
  const tf2::BufferCore & buffer, const std::string & target_frame,
  const std::string & source_frame)
{
  StaticExtrinsicResult result;
  result.missing = core::missing_frames(buffer, target_frame, source_frame);
  if (!result.missing.empty()) {
    return result;
  }
  try {
    result.transform = buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const std::exception & e) {
    result.lookup_error = e.what();
  }
  return result;
}

}  // namespace bagwiz::core::pointcloud
