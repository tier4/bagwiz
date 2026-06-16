// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_cloud_reader.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <type_traits>

namespace bagwiz::core::pointcloud
{

namespace
{

constexpr std::uint8_t kUint8 = 1;
constexpr std::uint8_t kUint16 = 2;
constexpr std::uint8_t kFloat32 = 7;

template <typename T>
T byte_swap(T value)
{
  static_assert(std::is_trivially_copyable_v<T>, "byte_swap requires trivially-copyable type");
  std::array<std::byte, sizeof(T)> in{};
  std::memcpy(in.data(), &value, sizeof(T));
  std::array<std::byte, sizeof(T)> out{};
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    out[i] = in[sizeof(T) - 1 - i];
  }
  T result{};
  std::memcpy(&result, out.data(), sizeof(T));
  return result;
}

template <typename T>
T load_pod(const std::byte * src, bool big_endian)
{
  static_assert(std::is_trivially_copyable_v<T>, "load_pod requires trivially-copyable type");
  T value{};
  std::memcpy(&value, src, sizeof(T));
  if constexpr (sizeof(T) == 1) {
    return value;
  }
  constexpr bool host_le = std::endian::native == std::endian::little;
  const bool need_swap = big_endian == host_le;
  return need_swap ? byte_swap(value) : value;
}

}  // namespace

std::optional<float> read_float_field(
  const PointCloudView & view, std::size_t point_index, std::size_t offset)
{
  if (view.point_step == 0) {
    return std::nullopt;
  }
  const std::size_t base = point_index * static_cast<std::size_t>(view.point_step);
  if (base + offset + sizeof(float) > view.data.size()) {
    return std::nullopt;
  }
  return load_pod<float>(view.data.data() + base + offset, view.is_bigendian);
}

std::optional<float> read_intensity(const PointCloudView & view, std::size_t point_index)
{
  if (!view.intensity_offset.has_value()) {
    return std::nullopt;
  }
  if (view.point_step == 0) {
    return std::nullopt;
  }
  const std::size_t base = point_index * static_cast<std::size_t>(view.point_step);
  const std::size_t offset = *view.intensity_offset;
  if (base + offset >= view.data.size()) {
    return std::nullopt;
  }

  switch (view.intensity_datatype) {
    case kUint8: {
      if (base + offset + sizeof(std::uint8_t) > view.data.size()) {
        return std::nullopt;
      }
      const auto raw = load_pod<std::uint8_t>(view.data.data() + base + offset, view.is_bigendian);
      return static_cast<float>(raw) / 255.0f;
    }
    case kUint16: {
      if (base + offset + sizeof(std::uint16_t) > view.data.size()) {
        return std::nullopt;
      }
      const auto raw = load_pod<std::uint16_t>(view.data.data() + base + offset, view.is_bigendian);
      return static_cast<float>(raw) / 65535.0f;
    }
    case kFloat32: {
      return read_float_field(view, point_index, offset);
    }
    default:
      return std::nullopt;
  }
}

bool is_valid_point(const PointCloudView & view, std::size_t point_index)
{
  if (!view.x_offset.has_value() || !view.y_offset.has_value() || !view.z_offset.has_value()) {
    return false;
  }
  const auto x = read_float_field(view, point_index, *view.x_offset);
  const auto y = read_float_field(view, point_index, *view.y_offset);
  const auto z = read_float_field(view, point_index, *view.z_offset);
  return x.has_value() && y.has_value() && z.has_value() && std::isfinite(*x) &&
         std::isfinite(*y) && std::isfinite(*z);
}

PointCloudResult extract_point_cloud(std::span<const std::byte> payload)
{
  PointCloudResult result;
  try {
    cdr_walker::CdrReader reader(payload);

    (void)reader.read_i32();   // header.stamp.sec
    (void)reader.read_u32();   // header.stamp.nanosec
    PointCloudView view;
    view.frame_id = reader.read_string();

    view.height = reader.read_u32();
    view.width = reader.read_u32();

    const std::uint32_t field_count = reader.read_sequence_length();
    std::optional<std::size_t> x_offset;
    std::optional<std::size_t> y_offset;
    std::optional<std::size_t> z_offset;
    std::optional<std::size_t> intensity_offset;
    std::uint8_t intensity_datatype = 0;

    for (std::uint32_t i = 0; i < field_count; ++i) {
      const std::string name = reader.read_string();
      const std::uint32_t offset = reader.read_u32();
      const std::uint8_t datatype = reader.read_u8();
      const std::uint32_t count = reader.read_u32();

      if (name == "x") {
        if (datatype != kFloat32) {
          result.error = "field 'x' must be FLOAT32 (datatype 7), got " + std::to_string(datatype);
          return result;
        }
        if (count != 1) {
          result.error = "field 'x' must have count == 1, got " + std::to_string(count);
          return result;
        }
        x_offset = offset;
      } else if (name == "y") {
        if (datatype != kFloat32) {
          result.error = "field 'y' must be FLOAT32 (datatype 7), got " + std::to_string(datatype);
          return result;
        }
        if (count != 1) {
          result.error = "field 'y' must have count == 1, got " + std::to_string(count);
          return result;
        }
        y_offset = offset;
      } else if (name == "z") {
        if (datatype != kFloat32) {
          result.error = "field 'z' must be FLOAT32 (datatype 7), got " + std::to_string(datatype);
          return result;
        }
        if (count != 1) {
          result.error = "field 'z' must have count == 1, got " + std::to_string(count);
          return result;
        }
        z_offset = offset;
      } else if (name == "intensity") {
        intensity_offset = offset;
        intensity_datatype = datatype;
      }
    }

    view.is_bigendian = reader.read_bool();
    view.point_step = reader.read_u32();
    view.row_step = reader.read_u32();

    const std::uint32_t data_length = reader.read_sequence_length();
    view.data = reader.read_bytes(data_length);
    view.is_dense = reader.read_bool();

    if (!x_offset || !y_offset || !z_offset) {
      result.error = "missing required point fields";
      return result;
    }

    if (view.point_step == 0 && view.width > 0) {
      result.error = "point_step is zero but width is non-zero";
      return result;
    }

    const std::size_t expected_data_size =
      static_cast<std::size_t>(view.row_step) * view.height;
    if (view.data.size() != expected_data_size) {
      result.error = "data size does not match row_step * height";
      return result;
    }

    const auto field_fits = [&](std::size_t offset, std::size_t size) {
      return offset + size <= static_cast<std::size_t>(view.point_step);
    };
    if (!field_fits(*x_offset, sizeof(float)) ||
        !field_fits(*y_offset, sizeof(float)) ||
        !field_fits(*z_offset, sizeof(float)))
    {
      result.error = "x/y/z offset exceeds point_step";
      return result;
    }

    if (intensity_offset.has_value()) {
      std::size_t intensity_size = 0;
      if (intensity_datatype == kUint8) {
        intensity_size = sizeof(std::uint8_t);
      } else if (intensity_datatype == kUint16) {
        intensity_size = sizeof(std::uint16_t);
      } else if (intensity_datatype == kFloat32) {
        intensity_size = sizeof(float);
      } else {
        result.error =
          "field 'intensity' has unsupported datatype " + std::to_string(intensity_datatype);
        return result;
      }
      if (!field_fits(*intensity_offset, intensity_size)) {
        result.error = "intensity offset exceeds point_step";
        return result;
      }
    }

    view.x_offset = x_offset;
    view.y_offset = y_offset;
    view.z_offset = z_offset;
    view.intensity_offset = intensity_offset;
    view.intensity_datatype = intensity_datatype;
    result.view = std::move(view);
  } catch (const std::exception & e) {
    result.view.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/PointCloud2 payload: ") + e.what();
  }

  return result;
}

}  // namespace bagwiz::core::pointcloud
