// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_image_pairer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;

slam::ScanImagePairer::PendingImage make_image(std::size_t cam, std::int64_t stamp_ns)
{
  slam::ScanImagePairer::PendingImage image;
  image.cam = cam;
  image.stamp_ns = stamp_ns;
  image.type = "sensor_msgs/msg/Image";
  image.payload = {std::byte{0x01}, std::byte{0x02}};
  return image;
}

std::vector<std::array<float, 3>> make_points(float base)
{
  return {{base, 0.0F, 0.0F}, {base + 1.0F, 0.0F, 0.0F}};
}

TEST(ScanImagePairer, ImageWaitsForABracketingScan)
{
  slam::ScanImagePairer pairer;
  pairer.push_image(make_image(0, 100));
  EXPECT_FALSE(pairer.has_decidable());

  // A scan older than the pending image does not decide it: a closer scan may
  // still arrive (scans arrive in stamp order).
  pairer.push_scan(40, make_points(0.0F));
  EXPECT_FALSE(pairer.has_decidable());

  pairer.push_scan(130, make_points(10.0F));
  ASSERT_TRUE(pairer.has_decidable());
  const auto decision = pairer.decide_front();
  EXPECT_EQ(decision.image.stamp_ns, 100);
  ASSERT_EQ(decision.dynamic_points.size(), 2U);
  EXPECT_FLOAT_EQ(decision.dynamic_points[0][0], 10.0F);  // the nearest scan is 130
  EXPECT_FALSE(pairer.has_decidable());
}

TEST(ScanImagePairer, NearestScanWinsAcrossSlots)
{
  slam::ScanImagePairer pairer;
  pairer.push_scan(0, make_points(1.0F));
  pairer.push_scan(200, make_points(2.0F));
  pairer.push_image(make_image(0, 90));
  ASSERT_TRUE(pairer.has_decidable());
  const auto decision = pairer.decide_front();
  ASSERT_EQ(decision.dynamic_points.size(), 2U);
  EXPECT_FLOAT_EQ(decision.dynamic_points[0][0], 1.0F);  // |0-90| = 90 < |200-90| = 110
}

TEST(ScanImagePairer, TieKeepsTheEarlierSlot)
{
  slam::ScanImagePairer pairer;
  pairer.push_scan(50, make_points(1.0F));
  pairer.push_scan(150, make_points(2.0F));
  pairer.push_image(make_image(0, 100));  // exactly between the two scans
  ASSERT_TRUE(pairer.has_decidable());
  const auto decision = pairer.decide_front();
  ASSERT_EQ(decision.dynamic_points.size(), 2U);
  EXPECT_FLOAT_EQ(decision.dynamic_points[0][0], 1.0F);
}

TEST(ScanImagePairer, ScanBeyondThePairWindowYieldsNoDynamicPoints)
{
  slam::ScanImagePairer pairer;
  pairer.push_scan(200'000'100, make_points(1.0F));
  pairer.push_image(make_image(0, 100));
  ASSERT_TRUE(pairer.has_decidable());
  EXPECT_TRUE(pairer.decide_front().dynamic_points.empty());
}

TEST(ScanImagePairer, PairWindowBoundaryIsInclusive)
{
  slam::ScanImagePairer pairer;
  pairer.push_scan(100 + slam::kScanPairWindowNs, make_points(1.0F));
  pairer.push_image(make_image(0, 100));
  ASSERT_TRUE(pairer.has_decidable());
  EXPECT_EQ(pairer.decide_front().dynamic_points.size(), 2U);

  slam::ScanImagePairer tight(10);
  tight.push_scan(111, make_points(1.0F));
  tight.push_image(make_image(0, 100));
  ASSERT_TRUE(tight.has_decidable());
  EXPECT_TRUE(tight.decide_front().dynamic_points.empty());
}

TEST(ScanImagePairer, FrontImageBlocksLaterImages)
{
  slam::ScanImagePairer pairer;
  pairer.push_image(make_image(0, 1'000'000'000));
  pairer.push_image(make_image(1, 100));
  pairer.push_scan(150, make_points(1.0F));
  // The second image is bracketed by scan 150 but the undecidable front image
  // keeps the whole queue waiting.
  EXPECT_FALSE(pairer.has_decidable());

  pairer.push_scan(1'200'000'000, make_points(2.0F));
  ASSERT_TRUE(pairer.has_decidable());
  const auto first = pairer.decide_front();
  EXPECT_EQ(first.image.stamp_ns, 1'000'000'000);
  // The nearest scan (1.2 s) is 200 ms away: outside the pair window.
  EXPECT_TRUE(first.dynamic_points.empty());

  ASSERT_TRUE(pairer.has_decidable());
  const auto second = pairer.decide_front();
  EXPECT_EQ(second.image.stamp_ns, 100);
  EXPECT_EQ(second.image.cam, 1U);
  ASSERT_EQ(second.dynamic_points.size(), 2U);
  EXPECT_FLOAT_EQ(second.dynamic_points[0][0], 1.0F);
  EXPECT_FALSE(pairer.has_decidable());
}

TEST(ScanImagePairer, KeepsOnlyTheLatestFewScans)
{
  slam::ScanImagePairer pairer;
  for (std::int64_t stamp = 10; stamp <= 50; stamp += 10) {
    pairer.push_scan(stamp, make_points(static_cast<float>(stamp)));
  }
  // Five scans were pushed; only the latest kScanSlotHistorySize survive, so
  // the scan at 10 — otherwise the nearest to the image — is gone.
  pairer.push_image(make_image(0, 12));
  ASSERT_TRUE(pairer.has_decidable());
  const auto decision = pairer.decide_front();
  ASSERT_EQ(decision.dynamic_points.size(), 2U);
  EXPECT_FLOAT_EQ(decision.dynamic_points[0][0], 20.0F);
}

TEST(ScanImagePairer, FinishMakesEveryPendingImageDecidable)
{
  slam::ScanImagePairer pairer;
  pairer.push_image(make_image(0, 100));
  pairer.push_image(make_image(2, 200));
  EXPECT_FALSE(pairer.has_decidable());

  pairer.finish();
  ASSERT_TRUE(pairer.has_decidable());
  EXPECT_EQ(pairer.decide_front().image.stamp_ns, 100);  // FIFO drain
  ASSERT_TRUE(pairer.has_decidable());
  const auto last = pairer.decide_front();
  EXPECT_EQ(last.image.stamp_ns, 200);
  EXPECT_TRUE(last.dynamic_points.empty());  // no scan ever arrived
  EXPECT_FALSE(pairer.has_decidable());
}

TEST(ScanImagePairer, ImageFieldsAreCarriedThrough)
{
  slam::ScanImagePairer pairer;
  pairer.push_image(make_image(3, 42));
  pairer.finish();
  const auto decision = pairer.decide_front();
  EXPECT_EQ(decision.image.cam, 3U);
  EXPECT_EQ(decision.image.stamp_ns, 42);
  EXPECT_EQ(decision.image.type, "sensor_msgs/msg/Image");
  ASSERT_EQ(decision.image.payload.size(), 2U);
  EXPECT_EQ(decision.image.payload[0], std::byte{0x01});
  EXPECT_EQ(decision.image.payload[1], std::byte{0x02});
}

}  // namespace
