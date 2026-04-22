// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagcli/commands/command.hpp"
#include "bagcli/core/logging.hpp"

#include <gtest/gtest.h>

TEST(Smoke, LoggingInitIsIdempotent)
{
  bagcli::core::init_logging();
  bagcli::core::init_logging();
  SUCCEED();
}

TEST(Smoke, RegistryIsAccessible)
{
  auto & registry = bagcli::commands::Registry::instance();
  // Skeleton has no commands registered.
  EXPECT_EQ(registry.all().size(), 0U);
}
