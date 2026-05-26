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
#include <fstream>
#include <optional>
#include <span>
#include <sstream>
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

class EnvVarGuard
{
public:
  EnvVarGuard(std::string name, const std::optional<std::string> & value) : name_(std::move(name))
  {
    const char * const previous = std::getenv(name_.c_str());
    had_previous_ = previous != nullptr;
    if (had_previous_) {
      previous_value_ = previous;
    }
    if (value.has_value()) {
      setenv(name_.c_str(), value->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

  EnvVarGuard(const EnvVarGuard &) = delete;
  EnvVarGuard & operator=(const EnvVarGuard &) = delete;
  EnvVarGuard(EnvVarGuard &&) = delete;
  EnvVarGuard & operator=(EnvVarGuard &&) = delete;

  ~EnvVarGuard()
  {
    if (had_previous_) {
      setenv(name_.c_str(), previous_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::string previous_value_;
  bool had_previous_ = false;
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

TEST_F(CompletionTest, TrajDumpTopicCompletionListsBagTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/fixture.mcap"}),
    "/bar\n/foo\n");
}

TEST_F(CompletionTest, TrajDumpTopicCompletionRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/fixture.mcap", "/f"}),
    "/foo\n");
}

TEST_F(CompletionTest, TrajJoinTopicCompletionListsBagTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");
  const auto traj_arg = (tmp_dir_ / "in.tum").string();

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "traj", "join", "~/fixture.mcap", traj_arg}),
    "/bar\n/foo\n");
}

// The topic-binding table guards against flag interleave in any positional
// slot before the topic. Pre-refactor walk would have blindly called the
// reader on whatever sat at words[1]; post-refactor the binding rejects it
// before the io call. End-user output stays empty either way, but this
// test pins the new gate so a regression cannot silently re-enable a
// reader call on a flag-shaped path.
TEST_F(CompletionTest, WalkTopicCompletionSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(run_completion({"bagwiz", "__complete", "3", "bagwiz", "walk", "--unknown-flag"}), "");
}

TEST_F(CompletionTest, TrajDumpTopicCompletionSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  // A flag in the input slot must not cause the topic binding to call the
  // bag reader on a flag-shaped path; the binding's earlier-slot guard
  // bails out and produces no topic candidates.
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "--unknown-flag"}), "");
}

TEST_F(CompletionTest, TrajDumpFormatFlagValueCompletes)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  // The user is mid-flag for `--format`; topic completion should not
  // trigger, and the value-after-flag branch should return the sole
  // supported format.
  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "traj", "dump", "~/fixture.mcap", "--format"}),
    "tum\n");
}

// Typing `-` at the bagwiz top-level should list the implicit CLI11 help
// flags plus the `--version` flag wired up in main().
TEST(FlagCompletionTest, TopLevelDashListsHelpAndVersion)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "1", "bagwiz", "-"}), "--help\n--version\n-h\n");
}

// `ls` defines no user-level options, so `-` should at minimum surface
// the CLI11-injected help flags (previously: nothing).
TEST(FlagCompletionTest, LsDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "ls", "-"}), "--help\n-h\n");
}

// `walk` has only positional args; its `-` candidates collapse to help.
// Topic completion is gated off via the `-` prefix, so the binding does
// not call into the bag reader here.
TEST(FlagCompletionTest, WalkDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "walk", "-"}), "--help\n-h\n");
}

// The `complete` subcommand defines two flags of its own; with help merged
// in they sort as: --force, --help, --install, -h.
TEST(FlagCompletionTest, CompleteDashListsCompleteFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "complete", "-"}),
    "--force\n--help\n--install\n-h\n");
}

// `convert` has no parent-level flags; the `format` subcommand owns
// --overwrite/--storage/-s. Both contexts must respond to `-`.
TEST(FlagCompletionTest, ConvertParentDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "convert", "-"}), "--help\n-h\n");
}

TEST(FlagCompletionTest, ConvertFormatDashListsFormatFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "convert", "format", "-"}),
    "--help\n--overwrite\n--storage\n-h\n-s\n");
}

// Parent-level flag completion at the subcommand slot.
TEST(FlagCompletionTest, TrajParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "traj", "-"}), "--help\n-h\n");
}

TEST(FlagCompletionTest, TrajDumpDashListsDumpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "dump", "-"}),
    "--format\n--from\n--help\n--overwrite\n--to\n-f\n-h\n");
}

TEST(FlagCompletionTest, TrajJoinDashListsJoinFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "join", "-"}),
    "--force\n--format\n--from\n--help\n--msg-type\n--output\n--overwrite\n--to\n-f\n-h\n-o\n-t\n");
}

TEST(FlagCompletionTest, TfParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "tf", "-"}), "--help\n-h\n");
}

// `tf tree` previously fell through to `return {}` — pin the new behavior.
TEST(FlagCompletionTest, TfTreeDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "tree", "-"}), "--help\n-h\n");
}

TEST(FlagCompletionTest, TfWalkDashListsWalkFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "walk", "-"}),
    "--help\n--rot\n-h\n-r\n");
}

TEST(FlagCompletionTest, TfInjectStaticDashListsInjectFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "inject-static", "-"}),
    "--force\n--help\n--output\n--overwrite\n-h\n-o\n");
}

// Prefix narrowing still works once the flag candidate set is widened.
TEST(FlagCompletionTest, TrajDumpDoubleDashOPrefixSelectsOverwrite)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "dump", "--o"}),
    "--overwrite\n");
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

class InstallPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_install_path_test_" +
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

TEST_F(InstallPathTest, BashUsesXdgDataHomeWhenSet)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  const EnvVarGuard xdg_guard("XDG_DATA_HOME", std::string{tmp_dir_ / "xdg"});

  const auto path = bagwiz::commands::default_install_path_for("bash");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, tmp_dir_ / "xdg" / "bash-completion" / "completions" / "bagwiz");
}

TEST_F(InstallPathTest, BashFallsBackToHomeLocalShareWhenXdgUnset)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  const EnvVarGuard xdg_guard("XDG_DATA_HOME", std::nullopt);

  const auto path = bagwiz::commands::default_install_path_for("bash");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, tmp_dir_ / ".local" / "share" / "bash-completion" / "completions" / "bagwiz");
}

TEST_F(InstallPathTest, ZshUsesHomeZshCompletions)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  const auto path = bagwiz::commands::default_install_path_for("zsh");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, tmp_dir_ / ".zsh" / "completions" / "_bagwiz");
}

TEST_F(InstallPathTest, FishUsesXdgConfigHomeWhenSet)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  const EnvVarGuard xdg_guard("XDG_CONFIG_HOME", std::string{tmp_dir_ / "xdg-conf"});

  const auto path = bagwiz::commands::default_install_path_for("fish");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, tmp_dir_ / "xdg-conf" / "fish" / "completions" / "bagwiz.fish");
}

TEST_F(InstallPathTest, FishFallsBackToHomeConfigWhenXdgUnset)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  const EnvVarGuard xdg_guard("XDG_CONFIG_HOME", std::nullopt);

  const auto path = bagwiz::commands::default_install_path_for("fish");
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(*path, tmp_dir_ / ".config" / "fish" / "completions" / "bagwiz.fish");
}

TEST(InstallPathStandaloneTest, UnknownShellReturnsNullopt)
{
  EXPECT_FALSE(bagwiz::commands::default_install_path_for("powershell").has_value());
  EXPECT_FALSE(bagwiz::commands::default_install_path_for("").has_value());
}

namespace
{

std::string read_text_file(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

}  // namespace

class InstallScriptTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_install_script_test_" +
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

TEST_F(InstallScriptTest, WritesScriptAndCreatesParentDirectories)
{
  const auto target = tmp_dir_ / "deep" / "nested" / "path" / "bagwiz";

  ASSERT_TRUE(bagwiz::commands::install_completion_script("bash", target, false));

  ASSERT_TRUE(std::filesystem::exists(target));
  const auto expected = bagwiz::commands::completion_script_for("bash");
  ASSERT_TRUE(expected.has_value());
  EXPECT_EQ(read_text_file(target), *expected);
}

TEST_F(InstallScriptTest, RefusesToOverwriteExistingFileWithoutForce)
{
  const auto target = tmp_dir_ / "bagwiz";
  {
    std::ofstream stream(target);
    stream << "previous";
  }

  EXPECT_FALSE(bagwiz::commands::install_completion_script("bash", target, false));
  EXPECT_EQ(read_text_file(target), "previous");
}

TEST_F(InstallScriptTest, OverwritesExistingFileWhenForceIsTrue)
{
  const auto target = tmp_dir_ / "bagwiz";
  {
    std::ofstream stream(target);
    stream << "previous";
  }

  ASSERT_TRUE(bagwiz::commands::install_completion_script("zsh", target, true));

  const auto expected = bagwiz::commands::completion_script_for("zsh");
  ASSERT_TRUE(expected.has_value());
  EXPECT_EQ(read_text_file(target), *expected);
}

TEST_F(InstallScriptTest, UnknownShellFails)
{
  const auto target = tmp_dir_ / "bagwiz";
  EXPECT_FALSE(bagwiz::commands::install_completion_script("powershell", target, false));
  EXPECT_FALSE(std::filesystem::exists(target));
}
