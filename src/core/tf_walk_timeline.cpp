// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_walk_timeline.hpp"

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
  const tf2::BufferCore & buffer, tf2::TimePoint time, const std::string & from_frame,
  const std::string & to_frame)
{
  TfWalkStep step;
  step.time = time;
  try {
    // lookupTransform(target=to, source=from): the result is <from>'s origin
    // expressed in <to>, matching `ros2 run tf2_ros tf2_echo <from> <to>`.
    step.transform = buffer.lookupTransform(to_frame, from_frame, time);
  } catch (const tf2::TransformException & e) {
    step.error = e.what();
  }
  return step;
}

}  // namespace bagwiz::core
