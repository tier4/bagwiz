// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__TOLERANCE_HPP_
#define BAGWIZ__CORE__BASE__TOLERANCE_HPP_

// The per-quantity tolerances bagwiz's numeric output is held to. bagwiz does
// not promise bit-exact agreement between the CPU and CUDA backends, or
// between different `-j, --threads` values; it promises agreement within the
// class below matching the quantity. See AGENTS.md "Numerical
// Reproducibility" for which class applies to what, and for the three
// properties that stay strict (bag message order, output-file byte identity,
// discrete decisions).
//
// These are contract values, not tuning knobs. Each is derived from the
// physical or encoding resolution of the quantity, so widening one to make a
// failing comparison pass hides the divergence instead of explaining it.
namespace bagwiz::core::base::tolerance
{

// Point coordinates and translations, in meters, absolute. One to two orders
// of magnitude below LiDAR range resolution and the map voxel size: a
// millimeter is not a distinguishable position in the exported map.
inline constexpr double kPointMeters = 1e-3;

// Rotations, in radians, absolute. ~0.057 deg, which is 1 mm of arc at 1 m —
// deliberately the same magnitude as kPointMeters so a pose's rotation and
// translation are held to a consistent standard.
inline constexpr double kRotationRadians = 1e-3;

// Components of a normal or any other unit vector, absolute. ~0.06 deg,
// matching kRotationRadians for the same reason.
inline constexpr double kUnitVectorComponent = 1e-3;

// General real scalars — ratios, scores, residuals — relative. This is
// "equal as floating point, allowing for a different summation order", not a
// physical tolerance: any genuine algorithmic difference exceeds it.
inline constexpr double kScalarRelative = 1e-6;

// A color channel already quantized to uint8. One least-significant bit is
// exactly the sRGB quantization rounding step; a deviation of 2 or more means
// the aggregation differs, not the rounding.
inline constexpr int kColorChannelLsb = 1;

// A weight already quantized to uint8, for the same reason as
// kColorChannelLsb.
inline constexpr int kQuantizedWeightLsb = 1;

// Counts and point totals, relative, for end-to-end comparisons over real bag
// data only. Unit tests over synthetic input must compare counts exactly: a
// count that moves with the thread count there means the input sits on a
// tolerance boundary, which is a defect in the test input, not drift to be
// absorbed. Reserved for future real-bag end-to-end comparisons; no consumer
// today.
inline constexpr double kCountRelative = 1e-3;

}  // namespace bagwiz::core::base::tolerance

#endif  // BAGWIZ__CORE__BASE__TOLERANCE_HPP_
