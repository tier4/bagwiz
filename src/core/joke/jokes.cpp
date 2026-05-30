// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/joke/jokes.hpp"

#include "bagwiz/core/joke/jokes_data.hpp"  // generated: kEmbeddedJokesJson

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core::joke
{

namespace
{
constexpr const char * kJokesKey = "jokes";
}  // namespace

std::vector<std::string> parse_jokes(std::string_view json_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(std::string{json_text});
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("failed to parse joke data: ") + e.what());
  }

  const YAML::Node jokes_node = root[kJokesKey];
  if (!jokes_node || !jokes_node.IsSequence()) {
    throw std::runtime_error("joke data must contain a \"jokes\" array");
  }

  std::vector<std::string> jokes;
  jokes.reserve(jokes_node.size());
  for (const auto & entry : jokes_node) {
    auto joke = entry.as<std::string>();
    if (joke.empty()) {
      throw std::runtime_error("joke data contains an empty entry");
    }
    jokes.push_back(std::move(joke));
  }

  if (jokes.empty()) {
    throw std::runtime_error("joke data contains no jokes");
  }
  return jokes;
}

std::vector<std::string> load_jokes()
{
  return parse_jokes(kEmbeddedJokesJson);
}

std::string random_joke(const std::vector<std::string> & jokes)
{
  if (jokes.empty()) {
    return {};
  }
  std::random_device device;
  std::mt19937 generator(device());
  std::uniform_int_distribution<std::size_t> distribution(0, jokes.size() - 1);
  return jokes[distribution(generator)];
}

}  // namespace bagwiz::core::joke
