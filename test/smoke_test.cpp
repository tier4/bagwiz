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

#include <cstdlib>
#include <string>

TEST(Smoke, LoggingInitIsIdempotent)
{
  bagwiz::core::init_logging();
  bagwiz::core::init_logging();
  SUCCEED();
}

// init_logging installs a console format without the leading "[{time}]"
// field, so diagnostic lines no longer carry a unix timestamp.
TEST(Smoke, LoggingInitDropsTimestampFromConsoleFormat)
{
  ::unsetenv("RCUTILS_CONSOLE_OUTPUT_FORMAT");

  bagwiz::core::init_logging();

  const char * fmt = std::getenv("RCUTILS_CONSOLE_OUTPUT_FORMAT");
  ASSERT_NE(fmt, nullptr);
  EXPECT_STREQ(fmt, "[{severity}] [{name}]: {message}");
  EXPECT_EQ(std::string(fmt).find("{time}"), std::string::npos);
}

// An explicit RCUTILS_CONSOLE_OUTPUT_FORMAT export wins: init_logging only
// supplies a default and must not clobber a user-chosen layout.
TEST(Smoke, LoggingInitRespectsExistingConsoleFormat)
{
  const std::string custom = "[{severity}] [{time}] CUSTOM {message}";
  ::setenv("RCUTILS_CONSOLE_OUTPUT_FORMAT", custom.c_str(), /*overwrite=*/1);

  bagwiz::core::init_logging();

  const char * fmt = std::getenv("RCUTILS_CONSOLE_OUTPUT_FORMAT");
  ASSERT_NE(fmt, nullptr);
  EXPECT_EQ(custom, fmt);

  // Leave the environment clean for any test that runs afterward.
  ::unsetenv("RCUTILS_CONSOLE_OUTPUT_FORMAT");
}

TEST(Smoke, RegistryIsAccessible)
{
  auto & registry = bagwiz::commands::Registry::instance();
  // Skeleton has no commands registered.
  EXPECT_EQ(registry.all().size(), 0U);
}
