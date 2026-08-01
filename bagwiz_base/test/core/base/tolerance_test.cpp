// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/tolerance.hpp"

#include <gtest/gtest.h>

namespace
{
namespace tol = bagwiz::core::base::tolerance;

// The classes are not independent numbers: the rotation and unit-vector
// classes are derived from the translation class ("1 mm at 1 m"), so an edit
// that moves one without the others breaks the stated rationale. These
// assertions exist to make such an edit fail loudly rather than silently
// widen one axis of the contract.
TEST(Tolerance, RotationMatchesTheTranslationClass)
{
  EXPECT_DOUBLE_EQ(tol::kRotationRadians, tol::kPointMeters);
}

TEST(Tolerance, UnitVectorMatchesTheRotationClass)
{
  EXPECT_DOUBLE_EQ(tol::kUnitVectorComponent, tol::kRotationRadians);
}

// Quantized channels tolerate exactly the rounding step and nothing more: a
// deviation of 2 means the aggregation differs, not the quantization.
TEST(Tolerance, QuantizedClassesAreOneLeastSignificantBit)
{
  EXPECT_EQ(tol::kColorChannelLsb, 1);
  EXPECT_EQ(tol::kQuantizedWeightLsb, 1);
}

// A sanity bound rather than a design pin: a relative count tolerance is only
// meaningful strictly between 0 and 1, but nothing about the contract fixes
// where in that range it sits.
TEST(Tolerance, CountClassIsAUsableRelativeFraction)
{
  EXPECT_GT(tol::kCountRelative, 0.0);
  EXPECT_LT(tol::kCountRelative, 1.0);
}

}  // namespace
