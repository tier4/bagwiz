// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__COLOR_SCHEME_HPP_
#define BAGWIZ__CORE__POINTCLOUD__COLOR_SCHEME_HPP_

namespace bagwiz::core::pointcloud
{

enum class ColorScheme {
  kViridis,  // perceptually uniform blue-green-yellow
  kTurbo,    // high-saturation rainbow-like map
  kJet,      // common blue-cyan-yellow-red map
  kPlasma,   // perceptually uniform purple-orange
  kInferno,  // perceptually uniform black-orange-white
  kMagma,    // perceptually uniform black-purple-white
  kRainbow,  // full-spectrum rainbow map
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__COLOR_SCHEME_HPP_
