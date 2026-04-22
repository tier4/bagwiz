// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGCLI__IO__METADATA_YAML_HPP_
#define BAGCLI__IO__METADATA_YAML_HPP_

#include "bagcli/io/bag_io.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace bagcli::io
{

// Minimal view of a rosbag2 metadata.yaml file, carrying just the fields
// bagcli needs to open a directory bag without re-scanning.
struct BagMetadata
{
  std::string storage_identifier;                          // "mcap" or "sqlite3"
  std::vector<std::filesystem::path> relative_file_paths;  // in play order
  std::vector<TopicInfo> topics;                           // may be empty
};

// Parse `<dir>/metadata.yaml`. Throws on IO or schema errors.
BagMetadata load_metadata_yaml(const std::filesystem::path & yaml_path);

}  // namespace bagcli::io

#endif  // BAGCLI__IO__METADATA_YAML_HPP_
