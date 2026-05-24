// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__COMPLETION_HPP_
#define BAGWIZ__COMMANDS__COMPLETION_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

bool is_completion_request(int argc, char * const * argv);
int run_completion_request(int argc, char * const * argv);

std::vector<std::string> supported_shells();
std::optional<std::string> completion_script_for(const std::string_view & shell);

// Returns the canonical install path for `shell`'s completion script using the
// current process environment (HOME, XDG_DATA_HOME, XDG_CONFIG_HOME). Returns
// std::nullopt for an unknown shell or when no usable HOME is available.
std::optional<std::filesystem::path> default_install_path_for(const std::string_view & shell);

// Writes the completion script for `shell` to `target`, creating parent
// directories as needed. Refuses to overwrite an existing file unless
// `force` is true. Returns true on success.
bool install_completion_script(
  const std::string_view & shell, const std::filesystem::path & target, bool force);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__COMPLETION_HPP_
