// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__SEQUENTIAL_BACKEND_HPP_
#define BAGWIZ__CORE__PIPELINE__SEQUENTIAL_BACKEND_HPP_

#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <string_view>

namespace bagwiz::core::pipeline
{

// The reference Backend: a single-threaded, zero-copy "read -> route -> write"
// loop. Output is byte-identical to the historical bag_copy loop, so it doubles
// as the differential oracle the threaded backends are validated against, and
// it is the zero-cost default when no parallelism is wanted. Each stage is
// timed with StageProfiler (dormant unless BAGWIZ_PROFILE is set).
class SequentialBackend : public Backend
{
public:
  RewriteCounts run(
    io::BagReader & reader, io::BagWriter & writer, const Processor & processor,
    std::string_view profile_label) override;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__SEQUENTIAL_BACKEND_HPP_
