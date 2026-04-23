// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"

#include <gtest/gtest.h>

TEST(Smoke, LoggingInitIsIdempotent)
{
  bagwiz::core::init_logging();
  bagwiz::core::init_logging();
  SUCCEED();
}

TEST(Smoke, RegistryIsAccessible)
{
  auto & registry = bagwiz::commands::Registry::instance();
  // Skeleton has no commands registered.
  EXPECT_EQ(registry.all().size(), 0U);
}
