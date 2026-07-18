// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/overlay.hpp"

#include "bagwiz/core/pointcloud/color_mapper.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

std::string overlay_projected_points(
  const image::PackedRaster & src, const std::vector<ProjectedPoint> & projected,
  double property_min, double property_max, ColorScheme scheme, std::uint32_t point_size,
  float alpha, image::PackedRaster & out)
{
  if (src.empty()) {
    return "empty source raster";
  }
  if (&out != &src) {
    out = src;
  }

  const int width_i = static_cast<int>(src.width);
  const int height_i = static_cast<int>(src.height);
  const std::uint32_t step = src.width * 3U;

  // out already holds a copy of src, so we can draw directly into its buffer.
  cv::Mat canvas(height_i, width_i, CV_8UC3, const_cast<std::byte *>(out.bgr.data()), step);

  std::vector<ProjectedPoint> projected_sorted(projected.begin(), projected.end());
  std::sort(projected_sorted.begin(), projected_sorted.end(), [](const auto & a, const auto & b) {
    return a.depth < b.depth;
  });

  // Draw each point as a filled square whose side is point_size pixels, so the
  // rendered size grows one pixel per unit and its on-screen extent equals
  // point_size (the documented meaning of --point-size). Deriving a circle
  // radius as point_size / 2 instead quantized odd sizes onto the even size
  // below them (2 and 3 both radius 1, 4 and 5 both radius 2), so in the walk
  // preview the size only appeared to change on even values. `half` is the
  // top-left offset that keeps the square centered on the projected pixel.
  const int side = std::max(1, static_cast<int>(point_size));
  const int half = side / 2;

  auto in_bounds = [width_i, height_i, side](const ProjectedPoint & p) -> bool {
    return p.u >= -side && p.u < width_i + side && p.v >= -side && p.v < height_i + side;
  };

  auto draw_point = [half, side](
                      cv::Mat & target, const ProjectedPoint & p, const cv::Scalar & bgr) {
    cv::rectangle(target, cv::Rect(p.u - half, p.v - half, side, side), bgr, cv::FILLED);
  };

  ColorMapper mapper(scheme);

  if (alpha >= 0.999f) {
    for (const auto & p : projected_sorted) {
      if (!in_bounds(p)) {
        continue;
      }
      const auto color = mapper.map(p.value, property_min, property_max);
      const cv::Scalar bgr(color[0], color[1], color[2]);
      draw_point(canvas, p, bgr);
    }
  } else {
    // Alpha-blend the points onto the image. The overlay starts as a copy of the
    // image (not black) and gets opaque points drawn on it, so pixels with no
    // point are identical in both addWeighted operands and survive unchanged;
    // only pixels a point covers blend toward the point color. A black overlay
    // would instead scale every point-free pixel by (1 - alpha), darkening the
    // whole frame as alpha drops.
    cv::Mat overlay = canvas.clone();
    for (const auto & p : projected_sorted) {
      if (!in_bounds(p)) {
        continue;
      }
      const auto color = mapper.map(p.value, property_min, property_max);
      const cv::Scalar bgr(color[0], color[1], color[2]);
      draw_point(overlay, p, bgr);
    }
    cv::Mat blended;
    cv::addWeighted(canvas, 1.0 - alpha, overlay, alpha, 0.0, blended);
    blended.copyTo(canvas);
  }

  return {};
}

}  // namespace bagwiz::core::pointcloud
