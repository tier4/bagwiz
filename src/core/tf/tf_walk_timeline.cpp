// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_walk_timeline.hpp"

#include <tf2/exceptions.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core
{

std::vector<tf2::TimePoint> build_tf_walk_timeline(std::vector<tf2::TimePoint> stamps)
{
  std::sort(stamps.begin(), stamps.end());
  stamps.erase(std::unique(stamps.begin(), stamps.end()), stamps.end());
  return stamps;
}

TfWalkStep resolve_tf_walk_step(
  const tf2::BufferCore & buffer, tf2::TimePoint time, const std::string & of_frame,
  const std::string & ref_frame)
{
  TfWalkStep step;
  step.time = time;
  try {
    // lookupTransform(target=ref, source=of): the result is <of>'s pose
    // expressed in <ref>. Equivalent to `ros2 run tf2_ros tf2_echo <ref> <of>`
    // (tf2_echo takes the reference frame first).
    step.transform = buffer.lookupTransform(ref_frame, of_frame, time);
  } catch (const tf2::TransformException & e) {
    step.error = e.what();
  }
  return step;
}

}  // namespace bagwiz::core
