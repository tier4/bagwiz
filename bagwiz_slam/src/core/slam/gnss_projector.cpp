// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_projector.hpp"

#include <GeographicLib/LocalCartesian.hpp>

#include <array>
#include <memory>

namespace bagwiz::core::slam
{

// Holds the GeographicLib ENU solver plus a latch for the origin. Mirrors the
// ENU branch of geo_pose_convert's Projector, but the origin is chosen lazily
// from the first fix rather than configured up front.
struct GnssProjector::Impl
{
  GeographicLib::LocalCartesian enu;  // default datum (0,0,0) until origin is set
  bool origin_set = false;
};

GnssProjector::GnssProjector() : impl_(std::make_unique<Impl>())
{
}
GnssProjector::~GnssProjector() = default;
GnssProjector::GnssProjector(GnssProjector &&) noexcept = default;
GnssProjector & GnssProjector::operator=(GnssProjector &&) noexcept = default;

std::array<double, 3> GnssProjector::project(double latitude, double longitude, double altitude)
{
  if (!impl_->origin_set) {
    // Latch the ENU origin to the first fix. Forward(origin) is then the zero
    // vector by construction.
    impl_->enu.Reset(latitude, longitude, altitude);
    impl_->origin_set = true;
  }

  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  impl_->enu.Forward(latitude, longitude, altitude, east, north, up);
  return {east, north, up};
}

bool GnssProjector::has_origin() const noexcept
{
  return impl_->origin_set;
}

}  // namespace bagwiz::core::slam
