// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GNSS_PROJECTOR_HPP_
#define BAGWIZ__CORE__SLAM__GNSS_PROJECTOR_HPP_

#include <array>
#include <memory>

// Projects WGS84 GNSS fixes (latitude/longitude/altitude) into a local
// East-North-Up tangent plane in meters, so the SLAM GNSS constraint can align
// the map's world frame to GNSS with a planar rigid transform. The origin is
// latched lazily on the first projected fix: the absolute datum is irrelevant to
// the constraint (it estimates the world<-GNSS transform anyway), so any
// consistent metric frame works, and anchoring at the first fix keeps
// coordinates small and well-conditioned.
//
// GeographicLib is hidden behind a pimpl so this header stays dependency-free
// (mirroring geo_pose_convert's GeoPoseConverter). The projection itself reuses
// the same GeographicLib::LocalCartesian forward solve as `bagwiz convert
// msg geo`'s ENU route. Stateful and thread-incompatible; construct one per
// run.
namespace bagwiz::core::slam
{

class GnssProjector
{
public:
  GnssProjector();
  ~GnssProjector();

  GnssProjector(const GnssProjector &) = delete;
  GnssProjector & operator=(const GnssProjector &) = delete;
  GnssProjector(GnssProjector &&) noexcept;
  GnssProjector & operator=(GnssProjector &&) noexcept;

  // Project a WGS84 fix to local ENU meters {east, north, up}. The first call
  // latches the ENU origin to that fix (returning the zero vector); every later
  // call is relative to it.
  [[nodiscard]] std::array<double, 3> project(double latitude, double longitude, double altitude);

  // True once the origin has been latched (i.e. project() has been called).
  [[nodiscard]] bool has_origin() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__GNSS_PROJECTOR_HPP_
