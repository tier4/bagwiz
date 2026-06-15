// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/completion.hpp"

#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

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
#include <string_view>
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

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = 0;
  ts.header.stamp.nanosec = 0;
  ts.child_frame_id = child;
  ts.transform.rotation.w = 1.0;
  return ts;
}

// Self-describing MCAP carrying one tf2_msgs/msg/TFMessage on `/tf`.
// The single payload encodes three edges (map → odom → base_link plus
// base_link → lidar), giving the completion path four distinct frame
// ids to discover after deduplication.
std::filesystem::path write_tf_mcap_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  bagwiz::io::TopicInfo tf_topic;
  tf_topic.name = "/tf";
  tf_topic.type = "tf2_msgs/msg/TFMessage";
  tf_topic.serialization_format = "cdr";
  tf_topic.schema_encoding = "ros2msg";
  tf_topic.schema_text = bagwiz::core::kTfMessageWireSchema;

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(make_edge("map", "odom"));
  transforms.push_back(make_edge("odom", "base_link"));
  transforms.push_back(make_edge("base_link", "lidar"));
  const auto cdr = bagwiz::core::serialize_tf_message(transforms);

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(tf_topic);
  writer->write(
    tf_topic.name, /*timestamp_ns=*/1'000'000'000LL,
    std::span<const std::byte>(cdr.data(), cdr.size()));
  writer->close();
  return path;
}

// Self-describing MCAP carrying two tf2_msgs/msg/TFMessage topics (`/tf` and
// `/tf_static`) plus one non-TF topic (`/points`). Used to verify that
// `tf tree` topic completion lists only the TFMessage-typed topics and
// excludes everything else.
std::filesystem::path write_mixed_tf_mcap_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  bagwiz::io::TopicInfo tf_topic;
  tf_topic.name = "/tf";
  tf_topic.type = "tf2_msgs/msg/TFMessage";
  tf_topic.serialization_format = "cdr";
  tf_topic.schema_encoding = "ros2msg";
  tf_topic.schema_text = bagwiz::core::kTfMessageWireSchema;

  bagwiz::io::TopicInfo tf_static_topic = tf_topic;
  tf_static_topic.name = "/tf_static";

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(make_edge("map", "odom"));
  transforms.push_back(make_edge("odom", "base_link"));
  const auto cdr = bagwiz::core::serialize_tf_message(transforms);
  const auto tf_bytes = std::span<const std::byte>(cdr.data(), cdr.size());

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto other_bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(tf_topic);
  writer->declare_topic(tf_static_topic);
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->write("/tf", 1'000'000'000LL, tf_bytes);
  writer->write("/tf_static", 1'000'000'000LL, tf_bytes);
  writer->write("/points", 2'000'000'000LL, other_bytes);
  writer->close();
  return path;
}

// MCAP carrying one topic of each message type `traj dump` supports (/odom,
// /pose, /pwc, /tf) plus two unsupported topics (/img, /points). Used to
// verify that `traj dump` <topic> completion offers only the supported types.
// Topic metadata alone drives completion, so the payloads are arbitrary bytes.
std::filesystem::path write_traj_dump_mixed_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/tf", "tf2_msgs/msg/TFMessage"));
  writer->declare_topic(make_topic("/pose", "geometry_msgs/msg/PoseStamped"));
  writer->declare_topic(make_topic("/pwc", "geometry_msgs/msg/PoseWithCovarianceStamped"));
  writer->declare_topic(make_topic("/odom", "nav_msgs/msg/Odometry"));
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/img", "sensor_msgs/msg/Image"));
  writer->write("/tf", 1'000'000'000, bytes);
  writer->write("/pose", 2'000'000'000, bytes);
  writer->write("/pwc", 3'000'000'000, bytes);
  writer->write("/odom", 4'000'000'000, bytes);
  writer->write("/points", 5'000'000'000, bytes);
  writer->write("/img", 6'000'000'000, bytes);
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

// `traj dump <bag> <TAB>` (the <topic> slot) lists only topics whose type is
// one traj dump can process — /odom, /pose, /pwc, /tf here — excluding the
// unsupported /img and /points, sorted.
TEST_F(CompletionTest, TrajDumpTopicCompletionListsOnlySupportedTypes)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_traj_dump_mixed_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/fixture.mcap"}),
    "/odom\n/pose\n/pwc\n/tf\n");
}

// A typed prefix narrows the <topic> candidates within the supported set.
TEST_F(CompletionTest, TrajDumpTopicCompletionRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_traj_dump_mixed_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/fixture.mcap", "/p"}),
    "/pose\n/pwc\n");
}

// A prefix that matches only an unsupported topic (/points) yields nothing:
// the type filter excludes it even though the name matches.
TEST_F(CompletionTest, TrajDumpTopicCompletionExcludesUnsupportedTypeOnPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_traj_dump_mixed_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/fixture.mcap", "/poi"}),
    "");
}

// A bag whose topics are all unsupported types yields no <topic> candidates, so
// the shell's default file completion takes over.
TEST_F(CompletionTest, TrajDumpTopicCompletionEmptyWhenNoSupportedTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "unsupported.mcap");  // String + Int32 only

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "traj", "dump", "~/unsupported.mcap"}),
    "");
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

// `--from <TAB>` after a TF-bearing bag must list every distinct frame
// id reachable from the bag's /tf message(s), sorted and deduplicated.
TEST_F(CompletionTest, TrajDumpFromFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "~/tf.mcap", "/tf", "out.tum",
       "--from"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `--to <TAB>` shares the same value source as `--from`. Pin both so
// that a future divergence cannot silently regress one branch.
TEST_F(CompletionTest, TrajDumpToFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "~/tf.mcap", "/tf", "out.tum",
       "--to"}),
    "base_link\nlidar\nmap\nodom\n");
}

// Typed prefix narrows the candidates to matching frame ids. Validates
// the prefix filter that `complete_frame_id_value` applies, including
// the case where a partial prefix matches no frames (returns empty).
TEST_F(CompletionTest, TrajDumpFromFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "~/tf.mcap", "/tf", "out.tum",
       "--from", "ba"}),
    "base_link\n");
}

// `traj join` reuses the same `complete_traj_frame_id` helper since
// it puts the bag at the same positional slot. Verify it actually
// surfaces frame ids too — guards against a regression where someone
// later restricts the helper to just `traj dump`.
TEST_F(CompletionTest, TrajJoinFromFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "traj", "join", "~/tf.mcap", "in.tum", "/tf",
       "--from"}),
    "base_link\nlidar\nmap\nodom\n");
}

// When the bag opens successfully but carries no tf2_msgs/msg/TFMessage
// topic, completion has nothing to suggest and returns empty — completion
// simply offers nothing rather than surfacing a placeholder candidate.
TEST_F(CompletionTest, TrajDumpFromFlagEmptyWhenBagHasNoTf)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "no_tf.mcap");  // String + Int32, no TF

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "~/no_tf.mcap", "/tf", "out.tum",
       "--from"}),
    "");
}

// An input path that fails to open must not surface the sentinel — that would
// mislead the user into believing the bag exists but is empty of TF.
// The contract is "silent fall-through so the shell's file completion
// takes over", matching how `complete_topics` handles bad inputs.
TEST_F(CompletionTest, TrajDumpFromFlagEmptyForMissingBag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "~/missing.mcap", "/tf", "out.tum",
       "--from"}),
    "");
}

// A flag in the bag slot must not be passed to io::open_read by the
// frame-id completer. Pins the same defensive gate that the topic
// binding applies for `walk` / `traj`.
TEST_F(CompletionTest, TrajDumpFromFlagSuppressedWhenBagSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "--unknown-flag", "/tf", "out.tum",
       "--from"}),
    "");
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
// in they sort as: --help, --install, --overwrite, -h.
TEST(FlagCompletionTest, CompleteDashListsCompleteFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "complete", "-"}),
    "--help\n--install\n--overwrite\n-h\n");
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

// `bagwiz convert <TAB>` lists both subcommands, sorted.
TEST(FlagCompletionTest, ConvertSubcommandListsFormatAndMsgtype)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "convert", ""}), "format\nmsgtype\n");
}

// `bagwiz convert msgtype <TAB>` lists its single action verb.
TEST(FlagCompletionTest, ConvertMsgtypeSubcommandListsGeo)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "convert", "msgtype", ""}), "geo\n");
}

// `bagwiz convert msgtype geo -` lists the geo flags, sorted, with help merged.
TEST(FlagCompletionTest, ConvertMsgtypeGeoDashListsGeoFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "convert", "msgtype", "geo", "-"}),
    "--crs\n--dst\n--frame-id\n--help\n--origin\n--output\n--overwrite\n--src\n--topic\n-h\n-o\n");
}

// `--src` completes from the source snake_case choice set (no bag access).
TEST(FlagCompletionTest, ConvertMsgtypeGeoSrcFlagListsChoices)
{
  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "convert", "msgtype", "geo", "in.mcap", "--src"}),
    "nav_sat_fix\n");
}

// `--dst` completes from the target snake_case choice set, sorted.
TEST(FlagCompletionTest, ConvertMsgtypeGeoDstFlagListsChoices)
{
  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "convert", "msgtype", "geo", "in.mcap", "--dst"}),
    "pose_stamped\npose_with_covariance_stamped\n");
}

// `--crs` completes the coordinate-system choices, sorted.
TEST(FlagCompletionTest, ConvertMsgtypeGeoCrsFlagListsChoices)
{
  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "convert", "msgtype", "geo", "in.mcap", "--crs"}),
    "enu\nutm\n");
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

// `bagwiz tf <TAB>` lists all subcommands, sorted.
TEST(FlagCompletionTest, TfSubcommandListsStaticTreeAndWalk)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "tf", ""}), "static\ntree\nwalk\n");
}

// `tf static <TAB>` lists the command group's actions, sorted.
TEST(FlagCompletionTest, TfStaticSubcommandListsCalcAndCp)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "static", ""}), "calc\ncp\n");
}

// `tf static -` is the command-group slot; `--json` lives under `calc`, so only
// the implicit help flags appear here.
TEST(FlagCompletionTest, TfStaticGroupDashListsHelpFlagsOnly)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "static", "-"}), "--help\n-h\n");
}

// `tf static calc -` surfaces the action's own --json flag alongside the
// implicit help flags.
TEST(FlagCompletionTest, TfStaticCalcDashListsStaticFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "static", "calc", "-"}),
    "--help\n--json\n-h\n");
}

// `tf static cp -` surfaces the copy action's flags (--output/-o, --overwrite)
// alongside the implicit help flags, sorted. <src>/<dst> are bag paths, so they
// carry no bagwiz candidates and fall through to the shell's file completion.
TEST(FlagCompletionTest, TfStaticCpDashListsCpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "static", "cp", "-"}),
    "--help\n--output\n--overwrite\n-h\n-o\n");
}

// `tf walk -` has no user flags, so only the implicit help flags appear.
TEST(FlagCompletionTest, TfWalkDashListsHelpFlagsOnly)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "walk", "-"}), "--help\n-h\n");
}

// `tf static calc <bag> <TAB>` (the <from> slot) lists only frame ids from the
// bag's static TF (*tf_static) topics, since `tf static calc` resolves the
// static tree. The mixed fixture's /tf_static carries map→odom→base_link.
TEST_F(CompletionTest, TfStaticCalcFromSlotListsStaticFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "tf", "static", "calc", "~/mixed.mcap"}),
    "base_link\nmap\nodom\n");
}

// `tf static calc <bag> <from> <TAB>` (the <to> slot) shares the same
// static-only frame-id source as the <from> slot.
TEST_F(CompletionTest, TfStaticCalcToSlotListsStaticFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "tf", "static", "calc", "~/mixed.mcap", "map"}),
    "base_link\nmap\nodom\n");
}

// A typed prefix narrows the static <from> frame-id candidates.
TEST_F(CompletionTest, TfStaticCalcFromSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "tf", "static", "calc", "~/mixed.mcap", "ba"}),
    "base_link\n");
}

// A bag with only a dynamic /tf topic (no *tf_static) has no static frames, so
// `tf static calc` completion returns empty rather than listing the dynamic
// frames — confirming the static-only restriction. (`tf walk` on the same bag
// does list those frames; see TfWalkFromSlotListsFrameIds.)
TEST_F(CompletionTest, TfStaticCalcFromSlotExcludesDynamicOnlyFrames)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");  // /tf only, no /tf_static

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "tf", "static", "calc", "~/tf.mcap"}),
    "");
}

// `tf walk <bag> <TAB>` (the <from> slot) lists the bag's TF frame ids. Unlike
// `tf static calc`, `tf walk`'s frame-id slots sit one word earlier (it has no
// `calc` action verb) and draw from all TF topics, not just the static ones.
TEST_F(CompletionTest, TfWalkFromSlotListsFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "walk", "~/tf.mcap"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `tf walk <bag> <from> <TAB>` (the <to> slot) shares the same frame-id source.
TEST_F(CompletionTest, TfWalkToSlotListsFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "tf", "walk", "~/tf.mcap", "map"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `tf tree <bag> <TAB>` (the <topic> slot) lists only the bag's
// tf2_msgs/msg/TFMessage topics — `/tf` and `/tf_static` here — excluding the
// non-TF `/points` topic, sorted.
TEST_F(CompletionTest, TfTreeTopicSlotListsOnlyTfMessageTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "~/mixed.mcap"}),
    "/tf\n/tf_static\n");
}

// A typed prefix narrows the <topic> candidates to matching TF topics.
TEST_F(CompletionTest, TfTreeTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "~/mixed.mcap", "/tf_"}),
    "/tf_static\n");
}

// A bag with no tf2_msgs/msg/TFMessage topic yields no <topic> candidates, so
// the shell's default file completion takes over (matches walk/traj behavior).
TEST_F(CompletionTest, TfTreeTopicSlotEmptyWhenBagHasNoTf)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "no_tf.mcap");  // String + Int32, no TF

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "~/no_tf.mcap"}), "");
}

// A flag in the input slot must not cause the tf-tree topic binding to call the
// bag reader on a flag-shaped path; the binding's earlier-slot guard bails out
// and produces no topic candidates.
TEST_F(CompletionTest, TfTreeTopicSlotSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "--unknown-flag"}), "");
}

// Completing the <input> slot itself (cursor on word 2, before the <topic> word)
// must not trigger tf-tree topic completion; the cursor-position guard bails so
// the shell's file completion handles the bag path.
TEST_F(CompletionTest, TfTreeInputSlotDoesNotListTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "tree", "~/mixed.mcap"}), "");
}

// A bag path that does not exist yields no <topic> candidates: the reader throws
// and complete_tf_message_topics swallows it, so the shell's file completion
// takes over instead of surfacing a misleading empty TF result.
TEST_F(CompletionTest, TfTreeTopicSlotEmptyForMissingBag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "~/missing.mcap"}), "");
}

// `tf tree` takes one-or-more topics, so the SECOND topic slot (and beyond) must
// also complete TF topics — the variadic binding fires at every positional slot
// from the first topic onward.
TEST_F(CompletionTest, TfTreeSecondTopicSlotListsTfMessageTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "tf", "tree", "~/mixed.mcap", "/tf"}),
    "/tf\n/tf_static\n");
}

// A typed prefix narrows the candidates at a later topic slot too.
TEST_F(CompletionTest, TfTreeSecondTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "tf", "tree", "~/mixed.mcap", "/tf", "/tf_st"}),
    "/tf_static\n");
}

// Prefix narrowing still works once the flag candidate set is widened.
TEST(FlagCompletionTest, TrajDumpDoubleDashOPrefixSelectsOverwrite)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "dump", "--o"}),
    "--overwrite\n");
}

// `topic drop <bag> <TAB>` (the first <topics> slot) lists every topic in the
// bag — drop can target any topic, so no type filter applies.
TEST_F(CompletionTest, TopicDropTopicSlotListsAllTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "topic", "drop", "~/fixture.mcap"}),
    "/bar\n/foo\n");
}

// `topic drop` takes one-or-more selectors, so the SECOND slot (and beyond)
// also completes topics — the variadic binding fires at every positional slot
// from the first topic onward.
TEST_F(CompletionTest, TopicDropSecondTopicSlotListsTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "topic", "drop", "~/fixture.mcap", "/foo"}),
    "/bar\n/foo\n");
}

// A typed prefix narrows the selector candidates.
TEST_F(CompletionTest, TopicDropTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "4", "bagwiz", "topic", "drop", "~/fixture.mcap", "/f"}),
    "/foo\n");
}

// A flag in the input slot must not cause the topic binding to call the bag
// reader on a flag-shaped path; the binding's earlier-slot guard bails out.
TEST_F(CompletionTest, TopicDropTopicSlotSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "topic", "drop", "--unknown-flag"}), "");
}

// `bagwiz topic <TAB>` lists the command group's action verbs, sorted.
TEST(FlagCompletionTest, TopicSubcommandListsDropAndKeep)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "topic", ""}), "drop\nkeep\n");
}

// `bagwiz topic -` is the command-group slot; only the implicit help flags
// appear (drop's own flags live one slot deeper).
TEST(FlagCompletionTest, TopicParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "topic", "-"}), "--help\n-h\n");
}

// `topic drop -` surfaces the action's flags (--output/-o, --overwrite) plus
// the implicit help flags, sorted.
TEST(FlagCompletionTest, TopicDropDashListsDropFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "topic", "drop", "-"}),
    "--help\n--output\n--overwrite\n-h\n-o\n");
}

// `topic keep <bag> <TAB>` (the first <topics> slot) lists every topic in the
// bag — keep can target any topic, so no type filter applies.
TEST_F(CompletionTest, TopicKeepTopicSlotListsAllTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "topic", "keep", "~/fixture.mcap"}),
    "/bar\n/foo\n");
}

// `topic keep` takes one-or-more selectors, so the SECOND slot (and beyond)
// also completes topics — the variadic binding fires at every positional slot
// from the first topic onward.
TEST_F(CompletionTest, TopicKeepSecondTopicSlotListsTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "topic", "keep", "~/fixture.mcap", "/foo"}),
    "/bar\n/foo\n");
}

// A typed prefix narrows the selector candidates.
TEST_F(CompletionTest, TopicKeepTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "4", "bagwiz", "topic", "keep", "~/fixture.mcap", "/f"}),
    "/foo\n");
}

// `topic keep -` surfaces the action's flags (--output/-o, --overwrite) plus
// the implicit help flags, sorted.
TEST(FlagCompletionTest, TopicKeepDashListsKeepFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "topic", "keep", "-"}),
    "--help\n--output\n--overwrite\n-h\n-o\n");
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

TEST(ActivateCommandTest, SourcesTargetForEveryShell)
{
  const std::filesystem::path target{"/tmp/bagwiz/completions/bagwiz"};
  for (const std::string_view shell : {"bash", "zsh", "fish"}) {
    const auto command = bagwiz::commands::activate_command_for(shell, target);
    ASSERT_TRUE(command.has_value()) << "shell: " << shell;
    EXPECT_EQ(*command, "source " + target.string()) << "shell: " << shell;
  }
}

TEST(ActivateCommandTest, UnknownShellReturnsNullopt)
{
  const std::filesystem::path target{"/tmp/bagwiz/completions/bagwiz"};
  EXPECT_FALSE(bagwiz::commands::activate_command_for("powershell", target).has_value());
  EXPECT_FALSE(bagwiz::commands::activate_command_for("", target).has_value());
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

TEST_F(InstallScriptTest, RefusesToOverwriteExistingFileWithoutOverwriteFlag)
{
  const auto target = tmp_dir_ / "bagwiz";
  {
    std::ofstream stream(target);
    stream << "previous";
  }

  EXPECT_FALSE(bagwiz::commands::install_completion_script("bash", target, false));
  EXPECT_EQ(read_text_file(target), "previous");
}

TEST_F(InstallScriptTest, OverwritesExistingFileWhenOverwriteIsTrue)
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
