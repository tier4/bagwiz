// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__JOKE__JOKES_HPP_
#define BAGWIZ__CORE__JOKE__JOKES_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::joke
{

// Parse and validate a joke list from JSON text shaped as:
//   { "jokes": ["first joke", "second joke", ...] }
// JSON is parsed via yaml-cpp (JSON is a subset of YAML). Throws
// std::runtime_error when the text is unparseable, lacks a non-empty
// "jokes" string array, or contains an empty entry. Validating here keeps
// random_joke total over a guaranteed-non-empty list.
[[nodiscard]] std::vector<std::string> parse_jokes(std::string_view json_text);

// The jokes embedded into the binary at build time from
// src/core/joke/jokes.json.
// Self-contained: needs no runtime file or sourced ROS overlay. Throws
// std::runtime_error only if the embedded data is somehow malformed (a
// build-time error in practice).
[[nodiscard]] std::vector<std::string> load_jokes();

// Pick one joke uniformly at random from `jokes`. Seeds from a
// non-deterministic source per call. Returns "" if `jokes` is empty.
[[nodiscard]] std::string random_joke(const std::vector<std::string> & jokes);

}  // namespace bagwiz::core::joke

#endif  // BAGWIZ__CORE__JOKE__JOKES_HPP_
