// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/deskew.hpp"

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace bagwiz::core::pointcloud
{

namespace
{

// Separates sweep-relative times (< ~seconds) from epoch-absolute (~1.7e9 s).
constexpr double kRelativeTimeThresholdSec = 1.0e6;

// Mirror of core::interpolate_poses (bagwiz_tf/src/core/tf/trajectory.cpp),
// copied operation-for-operation so the per-point hot loop can inline it and
// so both produce bit-identical results. Any change to that function must be
// mirrored here.
core::TrajectoryPose interpolate_pose_inline(
  const core::TrajectoryPose & a, const core::TrajectoryPose & b, double t)
{
  core::TrajectoryPose out;
  out.timestamp_ns = a.timestamp_ns;
  const double t1 = 1.0 - t;
  out.tx = t1 * a.tx + t * b.tx;
  out.ty = t1 * a.ty + t * b.ty;
  out.tz = t1 * a.tz + t * b.tz;

  // SLERP on the quaternion.
  double dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
  double bx = b.qx;
  double by = b.qy;
  double bz = b.qz;
  double bw = b.qw;
  if (dot < 0.0) {
    dot = -dot;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }
  if (dot > 0.9995) {
    out.qx = a.qx + t * (bx - a.qx);
    out.qy = a.qy + t * (by - a.qy);
    out.qz = a.qz + t * (bz - a.qz);
    out.qw = a.qw + t * (bw - a.qw);
  } else {
    const double omega = std::acos(dot);
    const double sin_omega = std::sin(omega);
    const double s0 = std::sin(t1 * omega) / sin_omega;
    const double s1 = std::sin(t * omega) / sin_omega;
    out.qx = s0 * a.qx + s1 * bx;
    out.qy = s0 * a.qy + s1 * by;
    out.qz = s0 * a.qz + s1 * bz;
    out.qw = s0 * a.qw + s1 * bw;
  }
  // Normalize to be safe.
  const double norm =
    std::sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw);
  if (norm > 0.0) {
    out.qx /= norm;
    out.qy /= norm;
    out.qz /= norm;
    out.qw /= norm;
  }
  return out;
}

// Minimal double 3x3 rotation + translation machinery. Every function below
// reproduces the exact formula and association order of its tf2::LinearMath
// counterpart (Matrix3x3::setRotation, Matrix3x3 / Transform operator*,
// Transform::inverse), so composing these structs yields bit-identical
// results to composing tf2::Transform objects.
struct Mat3
{
  double m[3][3];
};

struct Vec3
{
  double x, y, z;
};

// tf2::Matrix3x3::setRotation: no quaternion renormalisation.
Mat3 mat_from_quat(double qx, double qy, double qz, double qw)
{
  const double d = qx * qx + qy * qy + qz * qz + qw * qw;
  const double s = 2.0 / d;
  const double xs = qx * s, ys = qy * s, zs = qz * s;
  const double wx = qw * xs, wy = qw * ys, wz = qw * zs;
  const double xx = qx * xs, xy = qx * ys, xz = qx * zs;
  const double yy = qy * ys, yz = qy * zs, zz = qz * zs;
  Mat3 r;
  r.m[0][0] = 1.0 - (yy + zz);
  r.m[0][1] = xy - wz;
  r.m[0][2] = xz + wy;
  r.m[1][0] = xy + wz;
  r.m[1][1] = 1.0 - (xx + zz);
  r.m[1][2] = yz - wx;
  r.m[2][0] = xz - wy;
  r.m[2][1] = yz + wx;
  r.m[2][2] = 1.0 - (xx + yy);
  return r;
}

Mat3 mat_transpose(const Mat3 & a)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r.m[i][j] = a.m[j][i];
    }
  }
  return r;
}

// tf2::Matrix3x3::operator*: row-of-a dotted with column-of-b, k = 0, 1, 2.
Mat3 mat_mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r.m[i][j] = b.m[0][j] * a.m[i][0] + b.m[1][j] * a.m[i][1] + b.m[2][j] * a.m[i][2];
    }
  }
  return r;
}

// tf2::operator*(Matrix3x3, Vector3): row i dotted with v, left to right.
Vec3 mat_vec(const Mat3 & a, const Vec3 & v)
{
  return {
    a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
    a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
    a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

Vec3 vec_neg(const Vec3 & v)
{
  return {-v.x, -v.y, -v.z};
}

Vec3 vec_add(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
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
  PointCloud2 input, std::int64_t t_ref_ns, std::span<const core::TrajectoryPose> trajectory,
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
    out.cloud = std::move(input);
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
    out.cloud = std::move(input);
    out.points_no_time = out.points_total;
    return out;
  }

  const auto ref_pose = core::lookup_pose(t_ref_ns, trajectory);
  if (!ref_pose) {
    out.cloud = std::move(input);
    out.points_no_pose = out.points_total;
    return out;
  }

  // T_ref_inv and the extrinsic E / E_inv, once per cloud (tf2::Transform
  // semantics; see the Mat3/Vec3 note above).
  const Mat3 r_ref = mat_from_quat(ref_pose->qx, ref_pose->qy, ref_pose->qz, ref_pose->qw);
  const Mat3 r_ri = mat_transpose(r_ref);
  const Vec3 t_ri = mat_vec(r_ri, vec_neg({ref_pose->tx, ref_pose->ty, ref_pose->tz}));

  Mat3 r_e{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  Vec3 t_e{0.0, 0.0, 0.0};
  if (extrinsic) {
    r_e = mat_from_quat(
      extrinsic->rotation.x, extrinsic->rotation.y, extrinsic->rotation.z, extrinsic->rotation.w);
    t_e = {extrinsic->translation.x, extrinsic->translation.y, extrinsic->translation.z};
  }
  const Mat3 r_ei = mat_transpose(r_e);
  const Vec3 t_ei = mat_vec(r_ei, vec_neg(t_e));

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

  // Cursor over the sorted trajectory, kept at the lower_bound position for
  // the current point time. Point times within a scan are (nearly)
  // non-decreasing, so the position usually advances a few poses per cloud
  // instead of paying a binary search per point; a backwards jump
  // (non-monotone point times) falls back to std::lower_bound. The branch
  // logic below mirrors core::lookup_pose exactly, including endpoint
  // clamping, exact-stamp hits, and interpolation.
  const std::size_t n_poses = trajectory.size();
  std::size_t lo = 0;
  std::int64_t prev_t = std::numeric_limits<std::int64_t>::min();

  for (std::uint32_t r = 0; r < input.height; ++r) {
    for (std::uint32_t col = 0; col < input.width; ++col) {
      std::byte * base = input.data.data() + static_cast<std::size_t>(r) * rstep +
                         static_cast<std::size_t>(col) * input.point_step;
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

      if (t_i_ns < prev_t) {
        const auto cmp = [](const core::TrajectoryPose & p, std::int64_t t) {
          return p.timestamp_ns < t;
        };
        lo = static_cast<std::size_t>(
          std::lower_bound(trajectory.begin(), trajectory.end(), t_i_ns, cmp) - trajectory.begin());
      } else {
        while (lo < n_poses && trajectory[lo].timestamp_ns < t_i_ns) {
          ++lo;
        }
      }
      prev_t = t_i_ns;

      core::TrajectoryPose pose_i;
      if (lo == n_poses) {
        pose_i = trajectory.back();
      } else if (trajectory[lo].timestamp_ns == t_i_ns) {
        pose_i = trajectory[lo];
      } else if (lo == 0) {
        pose_i = trajectory.front();
      } else {
        const auto & prev = trajectory[lo - 1];
        const auto & next = trajectory[lo];
        const double dt = static_cast<double>(next.timestamp_ns - prev.timestamp_ns);
        if (dt <= 0.0) {
          pose_i = prev;
        } else {
          const double t = static_cast<double>(t_i_ns - prev.timestamp_ns) / dt;
          pose_i = interpolate_pose_inline(prev, next, t);
        }
      }

      // rel = E_inv * (T_ref_inv * T_i) * E, composed in the same order as
      // the equivalent tf2::Transform chain.
      const Mat3 r_i = mat_from_quat(pose_i.qx, pose_i.qy, pose_i.qz, pose_i.qw);
      const Vec3 t_i{pose_i.tx, pose_i.ty, pose_i.tz};
      const Mat3 inner_r = mat_mul(r_ri, r_i);
      const Vec3 inner_t = vec_add(mat_vec(r_ri, t_i), t_ri);
      const Mat3 rel2_r = mat_mul(r_ei, inner_r);
      const Vec3 rel2_t = vec_add(mat_vec(r_ei, inner_t), t_ei);
      const Mat3 rel_r = mat_mul(rel2_r, r_e);
      const Vec3 rel_t = vec_add(mat_vec(rel2_r, t_e), rel2_t);
      const Vec3 p = vec_add(mat_vec(rel_r, {x, y, z}), rel_t);

      store_xyz(base + fx->offset, fx->datatype, p.x);
      store_xyz(base + fy->offset, fy->datatype, p.y);
      store_xyz(base + fz->offset, fz->datatype, p.z);
      write_ref_time(base + time_field->offset, time_field->datatype, relative, t_ref_ns);
      ++out.points_deskewed;
    }
  }
  out.cloud = std::move(input);
  return out;
}

}  // namespace bagwiz::core::pointcloud
