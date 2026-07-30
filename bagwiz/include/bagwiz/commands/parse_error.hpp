// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PARSE_ERROR_HPP_
#define BAGWIZ__COMMANDS__PARSE_ERROR_HPP_

#include "CLI/CLI.hpp"

#include <string>

namespace bagwiz::commands
{

// Rewrite CLI11 parse-error messages so options that have a short form are
// reported with both forms (e.g. "-i/--input is required") instead of only
// the long form ("--input is required"). The rewrite scans the error message
// for option-name tokens and replaces each with the option's primary display
// name, looked up across the App tree.
std::string rewrite_parse_error(const CLI::App & app, const CLI::Error & error);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PARSE_ERROR_HPP_
