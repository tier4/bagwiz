// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/commands/command.hpp"

#include <utility>

namespace bagcli::commands
{

Registry & Registry::instance()
{
  static Registry r;
  return r;
}

void Registry::add(std::unique_ptr<Command> cmd)
{
  commands_.push_back(std::move(cmd));
}

}  // namespace bagcli::commands
