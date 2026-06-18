// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROPERTY_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROPERTY_HPP_

#include <cstdint>

namespace bagwiz::core::pointcloud
{

enum class PointCloudProperty {
  kX,          // x coordinate
  kY,          // y coordinate
  kZ,          // z coordinate
  kDistance,   // Euclidean distance from the sensor origin
  kIntensity,  // point intensity/reflectivity
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROPERTY_HPP_
