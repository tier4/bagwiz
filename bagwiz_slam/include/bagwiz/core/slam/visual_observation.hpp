// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__VISUAL_OBSERVATION_HPP_
#define BAGWIZ__CORE__SLAM__VISUAL_OBSERVATION_HPP_

#include <cstdint>

namespace bagwiz::core::slam
{

// One feature observation emitted by the visual frontend (Phase 2 of the
// map slam camera constraints). Coordinates are UNDISTORTED NORMALIZED image
// coordinates (X/Z, Y/Z in the camera optical frame), so downstream factor
// construction is camera-model agnostic; the frontend owns intrinsics and
// distortion. track_id is unique per (frontend instance) camera; camera_id
// indexes the CloudMapperConfig::visual_cameras extrinsic table.
struct VisualObservation
{
  std::int32_t camera_id = 0;
  std::uint64_t track_id = 0;
  std::int64_t stamp_ns = 0;
  double x = 0.0;
  double y = 0.0;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__VISUAL_OBSERVATION_HPP_
