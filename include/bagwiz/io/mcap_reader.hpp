// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__MCAP_READER_HPP_
#define BAGWIZ__IO__MCAP_READER_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <filesystem>
#include <memory>

namespace bagwiz::io::detail
{

// Open a single .mcap file as a BagReader.
std::unique_ptr<BagReader> open_mcap_file(const std::filesystem::path & path);

// Open a directory of .mcap shards. `metadata` must describe the layout
// (storage_identifier and relative_file_paths at minimum) — the caller is
// responsible for sourcing it from metadata.yaml or MetadataComputer.
std::unique_ptr<BagReader> open_mcap_directory(const std::filesystem::path & dir, BagMetadata md);

}  // namespace bagwiz::io::detail

#endif  // BAGWIZ__IO__MCAP_READER_HPP_
