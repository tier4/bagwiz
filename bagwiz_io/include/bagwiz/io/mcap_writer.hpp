// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__MCAP_WRITER_HPP_
#define BAGWIZ__IO__MCAP_WRITER_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <memory>

namespace bagwiz::io::detail
{

// Create a writer for a single .mcap file (no metadata.yaml).
std::unique_ptr<BagWriter> create_mcap_file(
  const std::filesystem::path & path, const CreateOptions & options);

// Create a writer for a directory bag: a single .mcap shard inside `dir`
// plus a metadata.yaml written on close().
std::unique_ptr<BagWriter> create_mcap_directory(
  const std::filesystem::path & dir, const CreateOptions & options);

}  // namespace bagwiz::io::detail

#endif  // BAGWIZ__IO__MCAP_WRITER_HPP_
