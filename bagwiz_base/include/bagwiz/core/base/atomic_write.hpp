// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__ATOMIC_WRITE_HPP_
#define BAGWIZ__CORE__BASE__ATOMIC_WRITE_HPP_

#include <filesystem>
#include <string>

namespace bagwiz::core
{

// Write `contents` to `path` via a sibling temporary + rename, so a failure
// partway through cannot leave a half-written file behind. The file-level
// sibling of write_bag_inplace() (core/bag_inplace.hpp), which gives bag paths
// the same guarantee.
//
// Returns true on success. On failure returns false, removes the temporary, and
// sets `error` to a message phrased so the caller can log it verbatim without
// prepending extra context. Never throws.
[[nodiscard]] bool write_file_atomically(
  const std::filesystem::path & path, const std::string & contents, std::string & error);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__ATOMIC_WRITE_HPP_
