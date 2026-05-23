// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/completion.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

class HomeEnvGuard
{
public:
  explicit HomeEnvGuard(const std::filesystem::path & home)
  : old_home_(std::getenv("HOME") == nullptr ? "" : std::getenv("HOME")),
    had_home_(std::getenv("HOME") != nullptr)
  {
    setenv("HOME", home.c_str(), 1);
  }

  HomeEnvGuard(const HomeEnvGuard &) = delete;
  HomeEnvGuard & operator=(const HomeEnvGuard &) = delete;
  HomeEnvGuard(HomeEnvGuard &&) = delete;
  HomeEnvGuard & operator=(HomeEnvGuard &&) = delete;

  ~HomeEnvGuard()
  {
    if (had_home_) {
      setenv("HOME", old_home_.c_str(), 1);
    } else {
      unsetenv("HOME");
    }
  }

private:
  std::string old_home_;
  bool had_home_ = false;
};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::filesystem::path write_mcap_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/foo", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/bar", "std_msgs/msg/Int32"));

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());
  writer->write("/foo", 1'000'000'000, bytes);
  writer->write("/bar", 2'000'000'000, bytes);
  writer->close();
  return path;
}

std::string run_completion(std::vector<std::string> args)
{
  std::vector<char *> argv;
  argv.reserve(args.size());
  for (auto & arg : args) {
    argv.push_back(arg.data());
  }

  testing::internal::CaptureStdout();
  const int rc =
    bagwiz::commands::run_completion_request(static_cast<int>(argv.size()), argv.data());
  std::string out = testing::internal::GetCapturedStdout();
  EXPECT_EQ(rc, 0);
  return out;
}

class CompletionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_completion_test_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST_F(CompletionTest, WalkTopicCompletionExpandsCurrentUserHome)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "walk", "~/fixture.mcap"}),
    "/bar\n/foo\n");
}

TEST(SupportedShellsTest, ListsBashZshAndFish)
{
  const auto shells = bagwiz::commands::supported_shells();
  EXPECT_NE(std::find(shells.begin(), shells.end(), "bash"), shells.end());
  EXPECT_NE(std::find(shells.begin(), shells.end(), "zsh"), shells.end());
  EXPECT_NE(std::find(shells.begin(), shells.end(), "fish"), shells.end());
}

TEST(CompletionScriptTest, UnsupportedShellReturnsNullopt)
{
  EXPECT_FALSE(bagwiz::commands::completion_script_for("powershell").has_value());
  EXPECT_FALSE(bagwiz::commands::completion_script_for("").has_value());
}

TEST(CompletionScriptTest, BashScriptContainsExpectedMarkers)
{
  const auto script = bagwiz::commands::completion_script_for("bash");
  ASSERT_TRUE(script.has_value());
  EXPECT_NE(script->find("_bagwiz_completion"), std::string::npos);
  EXPECT_NE(script->find("complete -o default -F _bagwiz_completion bagwiz"), std::string::npos);
  EXPECT_NE(script->find("bagwiz __complete"), std::string::npos);
}

TEST(CompletionScriptTest, ZshScriptContainsExpectedMarkers)
{
  const auto script = bagwiz::commands::completion_script_for("zsh");
  ASSERT_TRUE(script.has_value());
  EXPECT_NE(script->find("#compdef bagwiz"), std::string::npos);
  EXPECT_NE(script->find("_bagwiz"), std::string::npos);
  EXPECT_NE(script->find("bagwiz __complete"), std::string::npos);
  EXPECT_NE(script->find("_describe"), std::string::npos);
  EXPECT_NE(script->find("_files"), std::string::npos);
}

TEST(CompletionScriptTest, FishScriptContainsExpectedMarkers)
{
  const auto script = bagwiz::commands::completion_script_for("fish");
  ASSERT_TRUE(script.has_value());
  EXPECT_NE(script->find("__bagwiz_complete"), std::string::npos);
  EXPECT_NE(script->find("complete -c bagwiz"), std::string::npos);
  EXPECT_NE(script->find("commandline -opc"), std::string::npos);
  EXPECT_NE(script->find("commandline -ct"), std::string::npos);
  EXPECT_NE(script->find("bagwiz __complete"), std::string::npos);
}

TEST(CompletionScriptTest, FishScriptFallsBackToFileCompletion)
{
  const auto script = bagwiz::commands::completion_script_for("fish");
  ASSERT_TRUE(script.has_value());
  // -F (force file completion) gated by a "no candidates" condition is the
  // canonical fish equivalent of bash's `complete -o default` fallback.
  EXPECT_NE(script->find("-F"), std::string::npos);
}
