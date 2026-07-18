// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/rewrite_backend.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <string_view>

namespace bagwiz::core::pipeline
{

RewriteCounts run_pipeline(
  io::BagReader & reader, io::BagWriter & writer, const Processor & processor, Backend & backend,
  std::string_view profile_label)
{
  return backend.run(reader, writer, processor, profile_label);
}

}  // namespace bagwiz::core::pipeline
