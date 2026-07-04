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
#include <regex>
#include <string>

TEST(Smoke, LoggingInitIsIdempotent)
{
  bagwiz::core::init_logging();
  bagwiz::core::init_logging();
  SUCCEED();
}

// Every log level renders its timestamp as a human-readable local calendar
// datetime ("[YYYY-MM-DD HH:MM:SS.mmm TZ]") rather than a raw unix-epoch
// float, and the layout is identical across levels.
TEST(Smoke, LoggingRendersHumanReadableDateForEveryLevel)
{
  bagwiz::core::init_logging();

  testing::internal::CaptureStderr();
  // ERROR/WARN/INFO all sit at or above rcutils' default INFO threshold, so
  // each is emitted regardless of environment defaults.
  BAGWIZ_LOG_ERROR("bagwiz.test", "boom %d", 42);
  BAGWIZ_LOG_WARN("bagwiz.test", "careful");
  BAGWIZ_LOG_INFO("bagwiz.test", "hello");
  const std::string captured = testing::internal::GetCapturedStderr();

  // [LEVEL] [YYYY-MM-DD HH:MM:SS.mmm <TZ>] [bagwiz.test]: <message>
  // The timezone abbreviation is host-dependent (and may be absent), so it is
  // matched loosely as an optional trailing token before the closing bracket.
  const auto line_for = [&captured](const std::string & level) {
    const std::regex re(
      R"(\[)" + level +
      R"(\] \[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}( [^\]]+)?\] \[bagwiz\.test\]: )");
    return std::regex_search(captured, re);
  };

  EXPECT_TRUE(line_for("ERROR")) << captured;
  EXPECT_TRUE(line_for("WARN")) << captured;
  EXPECT_TRUE(line_for("INFO")) << captured;

  // The printf-style arguments are still expanded into the message.
  EXPECT_NE(captured.find("boom 42"), std::string::npos) << captured;
}

// The line is wrapped in rcutils' per-severity ANSI colour (ERROR/FATAL red,
// WARN yellow) when RCUTILS_COLORIZED_OUTPUT forces colour on, and emitted
// plain when it forces colour off -- independent of whether stderr is a
// terminal, matching rcutils' own colourization policy.
TEST(Smoke, LoggingColorizesPerSeverityWhenForced)
{
  bagwiz::core::init_logging();

  ::setenv("RCUTILS_COLORIZED_OUTPUT", "1", /*overwrite=*/1);
  testing::internal::CaptureStderr();
  BAGWIZ_LOG_ERROR("bagwiz.test", "red");
  BAGWIZ_LOG_WARN("bagwiz.test", "yellow");
  const std::string forced_on = testing::internal::GetCapturedStderr();

  // ERROR is wrapped in red ("\033[31m") and WARN in yellow ("\033[33m"), each
  // closed by the reset sequence ("\033[0m").
  EXPECT_NE(forced_on.find("\033[31m"), std::string::npos) << forced_on;
  EXPECT_NE(forced_on.find("\033[33m"), std::string::npos) << forced_on;
  EXPECT_NE(forced_on.find("\033[0m"), std::string::npos) << forced_on;

  ::setenv("RCUTILS_COLORIZED_OUTPUT", "0", /*overwrite=*/1);
  testing::internal::CaptureStderr();
  BAGWIZ_LOG_ERROR("bagwiz.test", "plain");
  const std::string forced_off = testing::internal::GetCapturedStderr();

  // No escape sequence at all when colour is forced off.
  EXPECT_EQ(forced_off.find('\033'), std::string::npos) << forced_off;

  // Leave the environment clean for any test that runs afterward.
  ::unsetenv("RCUTILS_COLORIZED_OUTPUT");
}

// BAGWIZ_LOG_LEVEL lowers rcutils' default logger threshold so DEBUG lines,
// which are suppressed at the default INFO level, become visible. The value is
// case-insensitive; an unset value leaves the default in place.
TEST(Smoke, LogLevelEnvControlsDebugVisibility)
{
  // Default threshold (env unset): a DEBUG line is suppressed.
  ::unsetenv("BAGWIZ_LOG_LEVEL");
  bagwiz::core::init_logging();
  testing::internal::CaptureStderr();
  BAGWIZ_LOG_DEBUG("bagwiz.test", "quiet-debug");
  EXPECT_EQ(testing::internal::GetCapturedStderr().find("quiet-debug"), std::string::npos);

  // BAGWIZ_LOG_LEVEL=debug (matched case-insensitively): the DEBUG line appears.
  ::setenv("BAGWIZ_LOG_LEVEL", "DeBuG", /*overwrite=*/1);
  bagwiz::core::init_logging();
  testing::internal::CaptureStderr();
  BAGWIZ_LOG_DEBUG("bagwiz.test", "loud-debug");
  EXPECT_NE(testing::internal::GetCapturedStderr().find("loud-debug"), std::string::npos);

  // Restore the default INFO threshold so tests running afterward are unaffected,
  // then leave the environment clean.
  ::setenv("BAGWIZ_LOG_LEVEL", "info", /*overwrite=*/1);
  bagwiz::core::init_logging();
  ::unsetenv("BAGWIZ_LOG_LEVEL");
}

TEST(Smoke, RegistryIsAccessible)
{
  auto & registry = bagwiz::commands::Registry::instance();
  // Skeleton has no commands registered.
  EXPECT_EQ(registry.all().size(), 0U);
}
