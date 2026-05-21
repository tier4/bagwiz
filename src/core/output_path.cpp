// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/output_path.hpp"

#include <filesystem>
#include <string>
#include <system_error>

namespace bagwiz::core
{

PrepareOutputResult prepare_output_path(const std::filesystem::path & path, bool overwrite)
{
  PrepareOutputResult result;

  // symlink_status (rather than status) so we treat a dangling symlink as
  // "something is there" — overwriting it should replace the symlink, not
  // chase it. The error_code overload distinguishes "the path is not there"
  // (status.type() == not_found) from a genuine stat failure such as
  // permission denied on the parent (status.type() == none).
  std::error_code probe_ec;
  const auto st = std::filesystem::symlink_status(path, probe_ec);
  if (st.type() == std::filesystem::file_type::not_found) {
    result.ok = true;
    return result;
  }
  if (st.type() == std::filesystem::file_type::none) {
    result.error = "could not stat output path '" + path.string() + "': " + probe_ec.message();
    return result;
  }

  if (!overwrite) {
    result.error =
      "output path '" + path.string() + "' already exists; pass --overwrite to replace it";
    return result;
  }

  // overwrite=true: nuke the existing entry so the writer starts from a
  // clean slot. remove_all handles both single files and directory trees,
  // and returns the count without throwing when given a std::error_code.
  std::error_code rm_ec;
  std::filesystem::remove_all(path, rm_ec);
  if (rm_ec) {
    result.error = "could not remove existing output path '" + path.string() +
                   "' for --overwrite: " + rm_ec.message();
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace bagwiz::core
