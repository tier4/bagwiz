// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGCLI__IO__MCAP_READER_HPP_
#define BAGCLI__IO__MCAP_READER_HPP_

#include "bagcli/io/bag_io.hpp"

#include <filesystem>
#include <memory>

namespace bagcli::io::detail
{

// Open a single .mcap file as a BagReader.
std::unique_ptr<BagReader> open_mcap_file(const std::filesystem::path & path);

// Open a directory containing metadata.yaml + one or more .mcap shards.
std::unique_ptr<BagReader> open_mcap_directory(const std::filesystem::path & dir);

}  // namespace bagcli::io::detail

#endif  // BAGCLI__IO__MCAP_READER_HPP_
