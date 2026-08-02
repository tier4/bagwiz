// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/visual_frontend.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/image/camera_info.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
namespace image = bagwiz::core::image;

image::CameraInfo make_pinhole(std::uint32_t w = 640, std::uint32_t h = 480)
{
  image::CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {500, 0, 320, 0, 500, 240, 0, 0, 1};
  return info;
}

image::CameraInfo make_rational_polynomial(std::uint32_t w = 640, std::uint32_t h = 480)
{
  image::CameraInfo info = make_pinhole(w, h);
  info.distortion_model = "rational_polynomial";
  info.d = {-0.1, 0.02, 0, 0, 0, -0.05, 0.01, 0};
  return info;
}

std::vector<std::byte> black(std::uint32_t w, std::uint32_t h)
{
  return std::vector<std::byte>(static_cast<std::size_t>(w) * h * 3, std::byte{0});
}

// Colored 5x5 squares on black at the given centers (tracking-friendly
// corners). The color is given as (b, g, r) to match the packed BGR24 raster.
std::vector<std::byte> render_dots_bgr(
  std::uint32_t w, std::uint32_t h, const std::vector<std::array<int, 2>> & centers,
  std::array<std::uint8_t, 3> bgr)
{
  auto img = black(w, h);
  for (const auto & c : centers) {
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        const int x = c[0] + dx, y = c[1] + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(w) || y >= static_cast<int>(h)) continue;
        auto * px = &img[(static_cast<std::size_t>(y) * w + x) * 3];
        px[0] = std::byte{bgr[0]};
        px[1] = std::byte{bgr[1]};
        px[2] = std::byte{bgr[2]};
      }
    }
  }
  return img;
}

// White 5x5 squares on black at the given centers (tracking-friendly corners).
std::vector<std::byte> render_dots(
  std::uint32_t w, std::uint32_t h, const std::vector<std::array<int, 2>> & centers)
{
  return render_dots_bgr(w, h, centers, {255, 255, 255});
}

// track_id lookup by value in a VisualObservation vector; nullptr when absent.
const slam::VisualObservation * find_by_id(
  const std::vector<slam::VisualObservation> & obs, std::uint64_t id)
{
  const auto it =
    std::find_if(obs.begin(), obs.end(), [id](const auto & o) { return o.track_id == id; });
  return it == obs.end() ? nullptr : &*it;
}

// Grayscale analogue of render_dots(): 5x5 white (255) squares on black (0) at
// the given centers, packed single-channel with stride == w.
std::vector<std::uint8_t> render_dots_gray(
  std::uint32_t w, std::uint32_t h, const std::vector<std::array<int, 2>> & centers)
{
  std::vector<std::uint8_t> img(static_cast<std::size_t>(w) * h, 0);
  for (const auto & c : centers) {
    for (int dy = -2; dy <= 2; ++dy) {
      for (int dx = -2; dx <= 2; ++dx) {
        const int x = c[0] + dx, y = c[1] + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(w) || y >= static_cast<int>(h)) continue;
        img[static_cast<std::size_t>(y) * w + x] = 255;
      }
    }
  }
  return img;
}

TEST(VisualFrontend, BlackFrameYieldsNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
}

TEST(VisualFrontend, TwoConsecutiveBlackFramesStillYieldNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
  EXPECT_TRUE(fe.track(100'000'000, frame, 640, 480).empty());
}

TEST(VisualFrontend, MismatchedRasterSizeYieldsNoTracks)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  // frame is sized for 640x480; claiming 320x240 makes width*height*3 not
  // match frame.size().
  EXPECT_TRUE(fe.track(0, frame, 320, 240).empty());
}

TEST(VisualFrontend, IsMoveConstructible)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  slam::VisualFrontend moved(std::move(fe));
  const auto frame = black(640, 480);
  EXPECT_TRUE(moved.track(0, frame, 640, 480).empty());
}

TEST(VisualFrontend, IsMoveAssignable)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  slam::VisualFrontend other(cfg);
  other = std::move(fe);
  const auto frame = black(640, 480);
  EXPECT_TRUE(other.track(0, frame, 640, 480).empty());
}

// The first call to track() ever seen by an instance only seeds the detector
// (no prior frame to flow from), so it always returns no observations; every
// later call reports KLT-tracked survivors plus any freshly detected corners.
// Tests below prime with one throwaway call, then a "settle" call re-tracking
// the same frame (zero motion) to obtain a real baseline observation set
// before introducing the change under test.

TEST(VisualFrontend, TracksFollowTranslation)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;  // native: no scale error in the displacement check
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  std::vector<std::array<int, 2>> shifted;
  for (const auto & c : centers) shifted.push_back({c[0] + 8, c[1]});

  const auto frame1 = render_dots(640, 480, centers);
  const auto frame2 = render_dots(640, 480, shifted);

  EXPECT_TRUE(fe.track(0, frame1, 640, 480).empty());
  const auto obs1 = fe.track(1, frame1, 640, 480);
  const auto obs2 = fe.track(2, frame2, 640, 480);

  ASSERT_GE(obs2.size(), 10U);

  const double fx = cfg.camera.k[0];
  int matched = 0;
  for (const auto & o1 : obs1) {
    const auto * o2 = find_by_id(obs2, o1.track_id);
    if (o2 == nullptr) continue;
    ++matched;
    EXPECT_NEAR((o2->x - o1.x) * fx, 8.0, 1.0) << "track_id=" << o1.track_id;
  }
  EXPECT_GE(matched, 10);
}

// rgb must be the delivered frame's nearest pixel at the track position. The
// test recomputes the sampling position from the reported normalized
// coordinates (exact inverse for the zero-distortion pinhole at native
// tracking scale) and compares against the frame it fed in, so the assertion
// holds wherever the detector happens to place corners.
TEST(VisualFrontend, ObservationsCarryRgbSampledAtTrackPosition)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;  // native scale: u = x*fx + cx round-trips exactly
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  const std::array<std::uint8_t, 3> dot_bgr{32, 64, 200};
  const auto frame = render_dots_bgr(640, 480, centers, dot_bgr);

  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
  const auto obs = fe.track(1, frame, 640, 480);
  ASSERT_GE(obs.size(), 10U);

  const double fx = cfg.camera.k[0], cx = cfg.camera.k[2];
  const double fy = cfg.camera.k[4], cy = cfg.camera.k[5];
  int colored = 0;
  for (const auto & o : obs) {
    const auto ix = std::clamp<std::int64_t>(std::llround(o.x * fx + cx), 0, 639);
    const auto iy = std::clamp<std::int64_t>(std::llround(o.y * fy + cy), 0, 479);
    const auto * px = &frame[(static_cast<std::size_t>(iy) * 640 + ix) * 3];
    EXPECT_EQ(o.rgb[0], std::to_integer<std::uint8_t>(px[2])) << "track_id=" << o.track_id;
    EXPECT_EQ(o.rgb[1], std::to_integer<std::uint8_t>(px[1])) << "track_id=" << o.track_id;
    EXPECT_EQ(o.rgb[2], std::to_integer<std::uint8_t>(px[0])) << "track_id=" << o.track_id;
    if (o.rgb != std::array<std::uint8_t, 3>{0, 0, 0}) {
      ++colored;
    }
  }
  // Corners of colored squares sample colored pixels; at least one must.
  EXPECT_GE(colored, 1);
}

TEST(VisualFrontend, TrackIdsAreStableAndNewDetectionsGetFreshIds)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  slam::VisualFrontend fe(cfg);

  const std::vector<std::array<int, 2>> original = {{100, 150}, {250, 150}, {400, 150},
                                                    {100, 300}, {250, 300}, {400, 300}};
  std::vector<std::array<int, 2>> with_extra = original;
  const std::vector<std::array<int, 2>> extra = {{100, 450}, {250, 450}, {400, 450}, {550, 450}};
  with_extra.insert(with_extra.end(), extra.begin(), extra.end());

  const auto frame_orig = render_dots(640, 480, original);
  const auto frame_extra = render_dots(640, 480, with_extra);

  EXPECT_TRUE(fe.track(0, frame_orig, 640, 480).empty());
  const auto obs1 = fe.track(1, frame_orig, 640, 480);
  ASSERT_GE(obs1.size(), 5U);
  const auto obs2 = fe.track(2, frame_extra, 640, 480);

  int persisted = 0;
  for (const auto & o1 : obs1) {
    if (find_by_id(obs2, o1.track_id) != nullptr) ++persisted;
  }
  EXPECT_GE(persisted, 5);

  int fresh = 0;
  for (const auto & o2 : obs2) {
    if (find_by_id(obs1, o2.track_id) == nullptr) ++fresh;
  }
  EXPECT_GE(fresh, 2);
}

TEST(VisualFrontend, LostFeaturesAreDropped)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  slam::VisualFrontend fe(cfg);

  const std::vector<std::array<int, 2>> top_row = {{150, 150}, {300, 150}, {450, 150}};
  const std::vector<std::array<int, 2>> bottom_row = {{150, 350}, {300, 350}, {450, 350}};
  std::vector<std::array<int, 2>> full = top_row;
  full.insert(full.end(), bottom_row.begin(), bottom_row.end());

  const auto frame_full = render_dots(640, 480, full);
  const auto frame_bottom_only = render_dots(640, 480, bottom_row);

  EXPECT_TRUE(fe.track(0, frame_full, 640, 480).empty());
  const auto obs1 = fe.track(1, frame_full, 640, 480);
  ASSERT_GE(obs1.size(), 5U);
  const auto obs2 = fe.track(2, frame_bottom_only, 640, 480);

  const double fy = cfg.camera.k[4];
  const double cy = cfg.camera.k[5];
  for (const auto & o1 : obs1) {
    const double v = o1.y * fy + cy;  // recover the tracking-scale-independent pixel row
    const bool is_top = v < 250.0;
    const auto * o2 = find_by_id(obs2, o1.track_id);
    if (is_top) {
      EXPECT_EQ(o2, nullptr) << "top-row track_id=" << o1.track_id << " should have been dropped";
    } else {
      EXPECT_NE(o2, nullptr) << "bottom-row track_id=" << o1.track_id << " should have survived";
    }
  }
}

TEST(VisualFrontend, NormalizedCoordsMatchPinhole)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;  // native: no scale error
  slam::VisualFrontend fe(cfg);

  const auto frame = render_dots(640, 480, {{400, 300}});

  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
  const auto obs = fe.track(1, frame, 640, 480);

  ASSERT_EQ(obs.size(), 1U);
  EXPECT_NEAR(obs[0].x, (400.0 - 320.0) / 500.0, 0.005);
  EXPECT_NEAR(obs[0].y, (300.0 - 240.0) / 500.0, 0.005);
  EXPECT_EQ(obs[0].camera_id, cfg.camera_id);
  EXPECT_EQ(obs[0].stamp_ns, 1);
}

// Two independent frontends see the exact same raster stream, one configured
// with zero distortion (so its emitted coords are the detector's raw
// normalized coords: undistort_normalized is the identity map when d is all
// zero) and one with the real rational_polynomial coefficients. Detection is
// deterministic given identical pixels, so the two frontends' tracks line up
// 1:1; matching by nearest coordinate (rather than track_id) keeps the test
// honest about that alignment instead of assuming id parity across separate
// instances. The rational frontend's observation must equal
// undistort_normalized applied to the zero-distortion frontend's observation,
// to the same precision as the production code path (same function, same
// inputs) — this guards the wiring, not the distortion math itself.
TEST(VisualFrontend, RationalPolynomialUndistortsObservations)
{
  slam::VisualFrontendConfig zero_cfg;
  zero_cfg.camera = make_pinhole();  // plumb_bob, d = {0,0,0,0,0}
  zero_cfg.tracking_width = 640;     // native: no scale-induced detection jitter
  slam::VisualFrontend zero_fe(zero_cfg);

  slam::VisualFrontendConfig rational_cfg;
  rational_cfg.camera = make_rational_polynomial();
  rational_cfg.tracking_width = 640;
  slam::VisualFrontend rational_fe(rational_cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  const auto frame = render_dots(640, 480, centers);

  EXPECT_TRUE(zero_fe.track(0, frame, 640, 480).empty());
  EXPECT_TRUE(rational_fe.track(0, frame, 640, 480).empty());
  const auto raw_obs = zero_fe.track(1, frame, 640, 480);
  const auto rational_obs = rational_fe.track(1, frame, 640, 480);

  ASSERT_GE(raw_obs.size(), 10U);
  ASSERT_GE(rational_obs.size(), 10U);

  const auto model = image::select_distortion_model(rational_cfg.camera.distortion_model);
  int matched = 0;
  for (const auto & raw : raw_obs) {
    const slam::VisualObservation * nearest = nullptr;
    double nearest_dist = std::numeric_limits<double>::max();
    for (const auto & ro : rational_obs) {
      const double dist = std::hypot(ro.x - raw.x, ro.y - raw.y);
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest = &ro;
      }
    }
    ASSERT_NE(nearest, nullptr);

    const image::NormalizedPoint expected =
      image::undistort_normalized(raw.x, raw.y, model, rational_cfg.camera.d);
    EXPECT_NEAR(nearest->x, expected.x, 1e-9);
    EXPECT_NEAR(nearest->y, expected.y, 1e-9);
    ++matched;
  }
  EXPECT_GE(matched, 10);
}

TEST(VisualFrontend, DimensionMismatchReturnsEmpty)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;  // native: no scale error in the displacement check
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  std::vector<std::array<int, 2>> shifted;
  for (const auto & c : centers) shifted.push_back({c[0] + 8, c[1]});

  const auto frame1 = render_dots(640, 480, centers);
  const auto frame2 = render_dots(640, 480, shifted);
  // Sized for 320x240 but claimed as 640x480: width*height*3 does not match
  // the buffer, so this must be rejected without touching internal state.
  const auto undersized_frame = black(320, 240);

  EXPECT_TRUE(fe.track(0, frame1, 640, 480).empty());
  const auto obs1 = fe.track(1, frame1, 640, 480);
  ASSERT_GE(obs1.size(), 10U);

  EXPECT_TRUE(fe.track(2, undersized_frame, 640, 480).empty());

  // prev_pyramid must still hold frame1's optical-flow pyramid: if the rejected
  // call had clobbered it, tracking against frame2 below would restart from
  // garbage instead of following the translation.
  const auto obs2 = fe.track(3, frame2, 640, 480);
  ASSERT_GE(obs2.size(), 10U);

  const double fx = cfg.camera.k[0];
  int matched = 0;
  for (const auto & o1 : obs1) {
    const auto * o2 = find_by_id(obs2, o1.track_id);
    if (o2 == nullptr) continue;
    ++matched;
    EXPECT_NEAR((o2->x - o1.x) * fx, 8.0, 1.0) << "track_id=" << o1.track_id;
  }
  EXPECT_GE(matched, 10);
}

TEST(VisualFrontend, DownscaleKeepsPixelAccuracy)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 320;  // 2x downscale from the 640-wide input frames
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  std::vector<std::array<int, 2>> shifted;
  for (const auto & c : centers) shifted.push_back({c[0] + 8, c[1]});

  const auto frame1 = render_dots(640, 480, centers);
  const auto frame2 = render_dots(640, 480, shifted);

  EXPECT_TRUE(fe.track(0, frame1, 640, 480).empty());
  const auto obs1 = fe.track(1, frame1, 640, 480);
  const auto obs2 = fe.track(2, frame2, 640, 480);

  ASSERT_GE(obs2.size(), 10U);

  const double fx = cfg.camera.k[0];
  int matched = 0;
  for (const auto & o1 : obs1) {
    const auto * o2 = find_by_id(obs2, o1.track_id);
    if (o2 == nullptr) continue;
    ++matched;
    EXPECT_NEAR((o2->x - o1.x) * fx, 8.0, 1.5) << "track_id=" << o1.track_id;
  }
  EXPECT_GE(matched, 10);
}

// A decoded/republished stream can be a different resolution than its
// CameraInfo declares (e.g. a downscaled relay), so the frontend must rescale
// config.camera's intrinsics to the frame size it actually receives instead of
// applying the declared-resolution fx/cx/fy/cy verbatim. Camera A declares
// 1280x960 with fx=1000, cx=640, cy=480 but is fed 640x480 frames (half res);
// camera B declares 640x480 with fx=fy=500, cx=320, cy=240 (make_pinhole()) and
// is fed the identical 640x480 frames. Optically these are the same camera at
// half the sensor resolution, so a correctly rescaling frontend must emit the
// same normalized coordinates for both, to full precision.
TEST(VisualFrontend, RescalesIntrinsicsToDecodedResolution)
{
  slam::VisualFrontendConfig declared_cfg;
  declared_cfg.camera.width = 1280;
  declared_cfg.camera.height = 960;
  declared_cfg.camera.distortion_model = "plumb_bob";
  declared_cfg.camera.d = {0, 0, 0, 0, 0};
  declared_cfg.camera.k = {1000, 0, 640, 0, 1000, 480, 0, 0, 1};
  declared_cfg.tracking_width = 640;  // native to the delivered 640x480 frames
  slam::VisualFrontend declared_fe(declared_cfg);

  slam::VisualFrontendConfig native_cfg;
  native_cfg.camera = make_pinhole();  // 640x480, fx=fy=500, cx=320, cy=240
  native_cfg.tracking_width = 640;
  slam::VisualFrontend native_fe(native_cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  const auto frame = render_dots(640, 480, centers);

  EXPECT_TRUE(declared_fe.track(0, frame, 640, 480).empty());
  EXPECT_TRUE(native_fe.track(0, frame, 640, 480).empty());
  const auto declared_obs = declared_fe.track(1, frame, 640, 480);
  const auto native_obs = native_fe.track(1, frame, 640, 480);

  ASSERT_GE(declared_obs.size(), 10U);
  ASSERT_GE(native_obs.size(), 10U);

  // Two independent instances tracking identical frames are deterministic but
  // not guaranteed to assign matching track ids, so match by nearest
  // coordinate (same approach as RationalPolynomialUndistortsObservations).
  int matched = 0;
  for (const auto & d : declared_obs) {
    const slam::VisualObservation * nearest = nullptr;
    double nearest_dist = std::numeric_limits<double>::max();
    for (const auto & n : native_obs) {
      const double dist = std::hypot(n.x - d.x, n.y - d.y);
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest = &n;
      }
    }
    ASSERT_NE(nearest, nullptr);
    EXPECT_NEAR(nearest->x, d.x, 1e-9);
    EXPECT_NEAR(nearest->y, d.y, 1e-9);
    ++matched;
  }
  EXPECT_GE(matched, 10);
}

TEST(VisualFrontend, StatsAccumulatePerStage)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  const auto frame = render_dots(640, 480, centers);
  EXPECT_TRUE(fe.track(0, frame, 640, 480).empty());
  const auto obs = fe.track(1, frame, 640, 480);
  ASSERT_FALSE(obs.empty());

  const auto & s = fe.stats();
  EXPECT_EQ(s.frames, 2);
  EXPECT_GT(s.gray_ns, 0);
  EXPECT_GT(s.resize_ns, 0);
  EXPECT_GT(s.klt_forward_ns, 0);  // second frame flowed
  EXPECT_GT(s.klt_backward_ns, 0);
  EXPECT_GE(s.detect_calls, 1);
  EXPECT_GT(s.detect_ns, 0);
  EXPECT_GT(s.emit_ns, 0);
}

TEST(VisualFrontend, StatsIgnoreRejectedFrames)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  slam::VisualFrontend fe(cfg);
  const auto frame = black(640, 480);
  EXPECT_TRUE(fe.track(0, frame, 320, 240).empty());  // size mismatch: rejected
  EXPECT_EQ(fe.stats().frames, 0);
}

TEST(VisualFrontend, RepeatedRunsAreIdentical)
{
  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  std::vector<std::array<int, 2>> shifted;
  for (const auto & c : centers) shifted.push_back({c[0] + 8, c[1] + 3});
  const auto frame1 = render_dots(640, 480, centers);
  const auto frame2 = render_dots(640, 480, shifted);

  const auto run = [&] {
    slam::VisualFrontendConfig cfg;
    cfg.camera = make_pinhole();
    cfg.tracking_width = 640;
    slam::VisualFrontend fe(cfg);
    std::vector<slam::VisualObservation> all;
    for (const auto obs :
         {fe.track(0, frame1, 640, 480), fe.track(1, frame1, 640, 480),
          fe.track(2, frame2, 640, 480)}) {
      all.insert(all.end(), obs.begin(), obs.end());
    }
    return all;
  };
  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].track_id, b[i].track_id);
    EXPECT_EQ(a[i].x, b[i].x);  // exact: same code on same input
    EXPECT_EQ(a[i].y, b[i].y);
  }
}

TEST(VisualFrontend, GrayEntryTracksTranslationAndSamplesRgb)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> centers;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      centers.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  std::vector<std::array<int, 2>> shifted;
  for (const auto & c : centers) shifted.push_back({c[0] + 8, c[1]});
  const auto g1 = render_dots_gray(640, 480, centers);
  const auto g2 = render_dots_gray(640, 480, shifted);
  const slam::GrayView v1{g1.data(), 640, 480, 640};
  const slam::GrayView v2{g2.data(), 640, 480, 640};
  const auto sampler = [](std::uint32_t, std::uint32_t) {
    return std::array<std::uint8_t, 3>{7, 8, 9};
  };

  EXPECT_TRUE(fe.track(0, v1, sampler).empty());
  const auto obs1 = fe.track(1, v1, sampler);
  const auto obs2 = fe.track(2, v2, sampler);
  ASSERT_GE(obs2.size(), 10U);
  EXPECT_EQ(obs2.front().rgb, (std::array<std::uint8_t, 3>{7, 8, 9}));

  const double fx = cfg.camera.k[0];
  int matched = 0;
  for (const auto & o1 : obs1) {
    const auto * o2 = find_by_id(obs2, o1.track_id);
    if (o2 == nullptr) continue;
    ++matched;
    EXPECT_NEAR((o2->x - o1.x) * fx, 8.0, 1.0);  // +8 px translation in x
    EXPECT_NEAR((o2->y - o1.y) * fx, 0.0, 1.0);
  }
  EXPECT_GE(matched, 10);
}

TEST(VisualFrontend, GrayEntryMatchesBgrEntryOnEquivalentInput)
{
  // The same dots as BGR white squares and as gray 255 squares must produce
  // identical observations (BGR->gray of pure white is 255).
  std::vector<std::array<int, 2>> centers = {{100, 100}, {220, 100}, {340, 220}, {100, 340}};
  const auto bgr1 = render_dots(640, 480, centers);
  const auto gray1 = render_dots_gray(640, 480, centers);

  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  slam::VisualFrontend bgr_fe(cfg);
  slam::VisualFrontend gray_fe(cfg);
  const slam::GrayView gv{gray1.data(), 640, 480, 640};
  const auto sampler = [](std::uint32_t, std::uint32_t) {
    return std::array<std::uint8_t, 3>{255, 255, 255};
  };

  (void)bgr_fe.track(0, bgr1, 640, 480);
  (void)gray_fe.track(0, gv, sampler);
  const auto a = bgr_fe.track(1, bgr1, 640, 480);
  const auto b = gray_fe.track(1, gv, sampler);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].x, b[i].x);
    EXPECT_EQ(a[i].y, b[i].y);
  }
}

TEST(VisualFrontend, DetectionSkippedWhileTracksAboveRefillFloor)
{
  slam::VisualFrontendConfig cfg;
  cfg.camera = make_pinhole();
  cfg.tracking_width = 640;
  cfg.max_features = 8;  // refill floor = 7 with kRefillRatio 0.85
  slam::VisualFrontend fe(cfg);

  std::vector<std::array<int, 2>> many;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      many.push_back({100 + col * 120, 100 + row * 120});
    }
  }
  const auto frame_many = render_dots(640, 480, many);
  (void)fe.track(0, frame_many, 640, 480);  // seed: detection fills to 8
  const auto settled = fe.track(1, frame_many, 640, 480);
  ASSERT_EQ(settled.size(), 8U);
  const auto detect_calls_settled = fe.stats().detect_calls;

  // Same frame again: 8 live tracks >= floor 6, so detection must NOT run and
  // no new track ids may appear.
  const auto again = fe.track(2, frame_many, 640, 480);
  EXPECT_EQ(fe.stats().detect_calls, detect_calls_settled);
  for (const auto & obs : again) {
    EXPECT_NE(find_by_id(settled, obs.track_id), nullptr);
  }

  // Drop most dots: survivors fall below the floor, detection refills.
  const auto frame_few = render_dots(640, 480, {many[0], many[1]});
  (void)fe.track(3, frame_few, 640, 480);
  EXPECT_GT(fe.stats().detect_calls, detect_calls_settled);
}

}  // namespace
