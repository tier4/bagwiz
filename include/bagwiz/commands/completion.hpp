// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__COMPLETION_HPP_
#define BAGWIZ__COMMANDS__COMPLETION_HPP_

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

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__COMPLETION_HPP_
