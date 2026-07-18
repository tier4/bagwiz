// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__TOPIC_ROUTER_HPP_
#define BAGWIZ__CORE__PIPELINE__TOPIC_ROUTER_HPP_

#include "bagwiz/core/pipeline/rewrite_backend.hpp"

#include <string>
#include <unordered_map>
#include <unordered_set>

// The two pure-copy Processors. Both keep payloads untouched and only decide
// the output topic name (or drop), so they are trivially thread-compatible
// (const lookups). Each holds a reference to a caller-owned selector container
// that must outlive the router.
namespace bagwiz::core::pipeline
{

// Drops messages whose topic is in `suppress`; forwards the rest unchanged.
// Backs `topic drop` (suppress = the dropped set) and `topic keep` (suppress =
// the complement of the kept set).
class SuppressRouter : public Processor
{
public:
  explicit SuppressRouter(const std::unordered_set<std::string> & suppress) : suppress_(suppress) {}

  [[nodiscard]] Emit route(const std::string & in_topic) const override;

private:
  const std::unordered_set<std::string> & suppress_;
};

// Forwards every message; a topic that is a key in `rename` is emitted under
// the mapped name, every other topic verbatim. Backs `topic rename`.
class RenameRouter : public Processor
{
public:
  explicit RenameRouter(const std::unordered_map<std::string, std::string> & rename)
  : rename_(rename)
  {
  }

  [[nodiscard]] Emit route(const std::string & in_topic) const override;

private:
  const std::unordered_map<std::string, std::string> & rename_;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__TOPIC_ROUTER_HPP_
