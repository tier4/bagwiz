// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/joke/jokes.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

TEST(JokesTest, ParsesEveryEntryFromJsonArray)
{
  const auto jokes =
    bagwiz::core::joke::parse_jokes(R"({"jokes": ["first joke", "second joke", "third joke"]})");

  ASSERT_EQ(jokes.size(), 3U);
  EXPECT_EQ(jokes[0], "first joke");
  EXPECT_EQ(jokes[1], "second joke");
  EXPECT_EQ(jokes[2], "third joke");
}

TEST(JokesTest, ThrowsWhenTextUnparseable)
{
  EXPECT_THROW((void)bagwiz::core::joke::parse_jokes("{ this is not valid"), std::runtime_error);
}

TEST(JokesTest, ThrowsWhenJokesKeyMissing)
{
  EXPECT_THROW((void)bagwiz::core::joke::parse_jokes(R"({"other": ["x"]})"), std::runtime_error);
}

TEST(JokesTest, ThrowsWhenArrayEmpty)
{
  EXPECT_THROW((void)bagwiz::core::joke::parse_jokes(R"({"jokes": []})"), std::runtime_error);
}

TEST(JokesTest, ThrowsWhenEntryEmpty)
{
  EXPECT_THROW(
    (void)bagwiz::core::joke::parse_jokes(R"({"jokes": ["ok", ""]})"), std::runtime_error);
}

TEST(JokesTest, LoadJokesReturnsTheEmbeddedList)
{
  // The list embedded from src/core/joke/jokes.json must be present and non-empty.
  const auto jokes = bagwiz::core::joke::load_jokes();
  EXPECT_FALSE(jokes.empty());
  for (const auto & joke : jokes) {
    EXPECT_FALSE(joke.empty());
  }
}

TEST(JokesTest, RandomJokeReturnsAMemberOfTheList)
{
  const std::vector<std::string> jokes{"a", "b", "c"};
  for (int i = 0; i < 50; ++i) {
    const auto joke = bagwiz::core::joke::random_joke(jokes);
    EXPECT_NE(std::find(jokes.begin(), jokes.end(), joke), jokes.end());
  }
}

TEST(JokesTest, RandomJokeReturnsEmptyForEmptyList)
{
  EXPECT_TRUE(bagwiz::core::joke::random_joke({}).empty());
}
