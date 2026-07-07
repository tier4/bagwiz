// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/deskew.hpp"

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace bagwiz::core::pointcloud
{

namespace
{

// Separates sweep-relative times (< ~seconds) from epoch-absolute (~1.7e9 s).
constexpr double kRelativeTimeThresholdSec = 1.0e6;

tf2::Transform pose_to_tf2(const core::TrajectoryPose & p)
{
  return tf2::Transform(tf2::Quaternion(p.qx, p.qy, p.qz, p.qw), tf2::Vector3(p.tx, p.ty, p.tz));
}

tf2::Transform transform_to_tf2(const geometry_msgs::msg::Transform & t)
{
  return tf2::Transform(
    tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}

const PointField * find_xyz(const PointCloud2 & c, const char * name)
{
  for (const auto & f : c.fields) {
    if (f.name == name) return &f;
  }
  return nullptr;
}

bool is_float(PointFieldType dt)
{
  return dt == PointFieldType::kFloat32 || dt == PointFieldType::kFloat64;
}

double load_xyz(const std::byte * base, PointFieldType dt)
{
  if (dt == PointFieldType::kFloat32) {
    float v;
    std::memcpy(&v, base, 4);
    return v;
  }
  double v;
  std::memcpy(&v, base, 8);
  return v;
}

void store_xyz(std::byte * base, PointFieldType dt, double val)
{
  if (dt == PointFieldType::kFloat32) {
    float v = static_cast<float>(val);
    std::memcpy(base, &v, 4);
  } else {
    std::memcpy(base, &val, 8);
  }
}

// After deskew every point shares t_ref; write a constant so downstream can't re-deskew.
void write_ref_time(std::byte * base, PointFieldType dt, bool relative, std::int64_t t_ref_ns)
{
  const double abs_sec = static_cast<double>(t_ref_ns) / 1.0e9;
  switch (dt) {
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(base, &v, 4);  // ns-relative -> 0
      break;
    }
    case PointFieldType::kFloat32: {
      float v = relative ? 0.0f : static_cast<float>(abs_sec);
      std::memcpy(base, &v, 4);
      break;
    }
    case PointFieldType::kFloat64: {
      double v = relative ? 0.0 : abs_sec;
      std::memcpy(base, &v, 8);
      break;
    }
    default:
      break;
  }
}

}  // namespace

DeskewResult deskew_pointcloud2(
  const PointCloud2 & input, std::int64_t t_ref_ns,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic)
{
  DeskewResult out;
  if (input.is_bigendian) {
    out.error = "big-endian point clouds are not supported";
    return out;
  }
  const PointField * fx = find_xyz(input, "x");
  const PointField * fy = find_xyz(input, "y");
  const PointField * fz = find_xyz(input, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    out.error = "cloud is missing one of the x/y/z fields";
    return out;
  }
  if (!is_float(fx->datatype) || fx->datatype != fy->datatype || fx->datatype != fz->datatype) {
    out.error = "x/y/z must all be the same float type (FLOAT32 or FLOAT64)";
    return out;
  }
  if (fx->count != 1 || fy->count != 1 || fz->count != 1) {
    out.error = "x/y/z count must be 1";
    return out;
  }
  if (input.point_step == 0) {
    out.error = "point_step is zero";
    return out;
  }
  const auto fits = [&](std::uint32_t offset, PointFieldType dt) {
    return static_cast<std::size_t>(offset) + datatype_size(dt) <= input.point_step;
  };
  if (
    !fits(fx->offset, fx->datatype) || !fits(fy->offset, fy->datatype) ||
    !fits(fz->offset, fz->datatype)) {
    out.error = "x/y/z field exceeds point_step";
    return out;
  }
  const std::uint32_t rstep = input.row_step != 0 ? input.row_step : input.width * input.point_step;
  if (static_cast<std::size_t>(input.width) * input.point_step > rstep) {
    out.error = "row_step is smaller than width*point_step";
    return out;
  }
  if (input.data.size() < static_cast<std::size_t>(input.height) * rstep) {
    out.error = "point data buffer too small";
    return out;
  }

  out.points_total = static_cast<std::uint64_t>(input.width) * input.height;

  const auto time_field = find_point_time_field(input);
  if (!time_field) {
    out.cloud = input;
    out.points_no_time = out.points_total;
    return out;
  }
  if (!fits(time_field->offset, time_field->datatype)) {
    // find_point_time_field / point_time_seconds do not bounds-check the
    // field against point_step (point_time.hpp: that is the caller's job) --
    // a malformed cloud whose declared time field runs past point_step would
    // otherwise read past its own point and write past it too, corrupting
    // the next point (or, for the last point, the end of `data`). Treat it
    // exactly like "no usable time field".
    out.cloud = input;
    out.points_no_time = out.points_total;
    return out;
  }

  const auto ref_pose = core::lookup_pose(t_ref_ns, trajectory);
  if (!ref_pose) {
    out.cloud = input;
    out.points_no_pose = out.points_total;
    return out;
  }
  const tf2::Transform t_ref_inv = pose_to_tf2(*ref_pose).inverse();
  const tf2::Transform e = extrinsic ? transform_to_tf2(*extrinsic) : tf2::Transform::getIdentity();
  const tf2::Transform e_inv = e.inverse();

  // Detect relative vs absolute times (one scan of the time field).
  double max_abs_sec = 0.0;
  for (std::uint32_t r = 0; r < input.height; ++r) {
    for (std::uint32_t col = 0; col < input.width; ++col) {
      const std::byte * b = input.data.data() + static_cast<std::size_t>(r) * rstep +
                            static_cast<std::size_t>(col) * input.point_step + time_field->offset;
      const double s = point_time_seconds(b, *time_field);
      if (std::isfinite(s)) max_abs_sec = std::max(max_abs_sec, std::abs(s));
    }
  }
  const bool relative = max_abs_sec < kRelativeTimeThresholdSec;

  PointCloud2 result = input;  // owns its own data; rewrite xyz + time below
  for (std::uint32_t r = 0; r < result.height; ++r) {
    for (std::uint32_t col = 0; col < result.width; ++col) {
      std::byte * base = result.data.data() + static_cast<std::size_t>(r) * rstep +
                         static_cast<std::size_t>(col) * result.point_step;
      const double x = load_xyz(base + fx->offset, fx->datatype);
      const double y = load_xyz(base + fy->offset, fy->datatype);
      const double z = load_xyz(base + fz->offset, fz->datatype);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++out.points_nonfinite;
        continue;
      }
      const double sec = point_time_seconds(base + time_field->offset, *time_field);
      if (!std::isfinite(sec)) {
        ++out.points_no_time;
        continue;
      }
      const std::int64_t t_i_ns =
        relative ? t_ref_ns + static_cast<std::int64_t>(std::llround(sec * 1.0e9))
                 : static_cast<std::int64_t>(std::llround(sec * 1.0e9));
      const auto pose_i = core::lookup_pose(t_i_ns, trajectory);
      if (!pose_i) {
        ++out.points_no_pose;
        continue;
      }
      const tf2::Transform rel = e_inv * (t_ref_inv * pose_to_tf2(*pose_i)) * e;
      const tf2::Vector3 p = rel * tf2::Vector3(x, y, z);
      store_xyz(base + fx->offset, fx->datatype, p.x());
      store_xyz(base + fy->offset, fy->datatype, p.y());
      store_xyz(base + fz->offset, fz->datatype, p.z());
      write_ref_time(base + time_field->offset, time_field->datatype, relative, t_ref_ns);
      ++out.points_deskewed;
    }
  }
  out.cloud = std::move(result);
  return out;
}

}  // namespace bagwiz::core::pointcloud
