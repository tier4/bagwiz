// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/completion.hpp"

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <zstd.h>

#include <array>
#include <cstddef>
#include <cstdint>
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

// MCAP carrying one raw image topic (/image), one compressed image topic
// (/image/compressed), and one non-image topic (/points). Used to verify
// `generate video` <image_topic> completion offers both image types it operates on
// (sensor_msgs/msg/Image and sensor_msgs/msg/CompressedImage) while excluding
// every non-image topic. Topic metadata alone drives completion, so the
// payloads are arbitrary bytes.
std::filesystem::path write_image_topics_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/image", "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic("/image/compressed", "sensor_msgs/msg/CompressedImage"));
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->write("/image", 1'000'000'000, bytes);
  writer->write("/image/compressed", 2'000'000'000, bytes);
  writer->write("/points", 3'000'000'000, bytes);
  writer->close();
  return path;
}

// MCAP carrying an image topic (/cam/image_raw/compressed) and its sibling
// CameraInfo topic (/cam/camera_info), plus an unrelated topic (/points). Used to
// verify `--cam-info` completion offers only the CameraInfo topic.
std::filesystem::path write_camera_info_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/cam/image_raw/compressed", "sensor_msgs/msg/CompressedImage"));
  writer->declare_topic(make_topic("/cam/camera_info", "sensor_msgs/msg/CameraInfo"));
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->write("/cam/image_raw/compressed", 1'000'000'000, bytes);
  writer->write("/cam/camera_info", 2'000'000'000, bytes);
  writer->write("/points", 3'000'000'000, bytes);
  writer->close();
  return path;
}

// MCAP carrying one PointCloud2 topic (/points) and one non-PointCloud2 topic
// (/image). Used to verify that `map slam` completes the <pcd_topic> slot with
// only sensor_msgs/msg/PointCloud2 topics.
std::filesystem::path write_pointcloud2_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/image", "sensor_msgs/msg/Image"));
  writer->write("/points", 1'000'000'000, bytes);
  writer->write("/image", 2'000'000'000, bytes);
  writer->close();
  return path;
}

// Write an uncompressed sqlite3 directory bag declaring the given (name, type)
// topics with one arbitrary-payload message each. Topic metadata alone drives
// completion, so the payloads are placeholders.
std::filesystem::path write_sqlite3_dir_bag(
  const std::filesystem::path & dir,
  const std::vector<std::pair<std::string, std::string>> & topics)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::Directory;

  constexpr std::array<std::byte, 4> kPayload{
    std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  const auto bytes = std::span<const std::byte>(kPayload.data(), kPayload.size());

  auto writer = bagwiz::io::open_write(dir, options);
  std::int64_t timestamp_ns = 1'000'000'000;
  for (const auto & [name, type] : topics) {
    writer->declare_topic(make_topic(name, type));
    writer->write(name, timestamp_ns, bytes);
    timestamp_ns += 1'000'000'000;
  }
  writer->close();
  return dir;
}

// Rewrite an uncompressed sqlite3 directory bag in place as a rosbag2 FILE-mode
// zstd envelope: compress the single `.db3` shard to `.db3.zstd`, drop the plain
// shard, and flip metadata.yaml's compression fields + relative_file_paths.
// Returns the bare `.db3.zstd` shard path (used for the single-file case).
std::filesystem::path make_file_mode_zstd(const std::filesystem::path & dir)
{
  std::filesystem::path shard;
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().extension() == ".db3") {
      shard = entry.path();
      break;
    }
  }

  std::ifstream in(shard, std::ios::binary | std::ios::ate);
  const auto size = static_cast<std::size_t>(in.tellg());
  in.seekg(0);
  std::vector<char> raw(size);
  in.read(raw.data(), static_cast<std::streamsize>(size));

  std::vector<char> compressed(ZSTD_compressBound(size));
  const std::size_t written =
    ZSTD_compress(compressed.data(), compressed.size(), raw.data(), size, /*level=*/3);
  compressed.resize(written);

  const std::filesystem::path zstd_path(shard.string() + ".zstd");
  {
    std::ofstream out(zstd_path, std::ios::binary);
    out.write(compressed.data(), static_cast<std::streamsize>(compressed.size()));
  }
  std::filesystem::remove(shard);

  YAML::Node root = YAML::LoadFile((dir / "metadata.yaml").string());
  YAML::Node info = root["rosbag2_bagfile_information"];
  info["compression_mode"] = "FILE";
  info["compression_format"] = "zstd";
  info["relative_file_paths"] = std::vector<std::string>{zstd_path.filename().string()};
  std::ofstream meta(dir / "metadata.yaml");
  meta << root;

  return zstd_path;
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
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "walk", "-i", "~/fixture.mcap", "-t"}),
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
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "traj", "dump", "-i", "~/fixture.mcap", "-t"}),
    "/odom\n/pose\n/pwc\n/tf\n");
}

// A typed prefix narrows the <topic> candidates within the supported set.
TEST_F(CompletionTest, TrajDumpTopicCompletionRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_traj_dump_mixed_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "traj", "dump", "-i", "~/fixture.mcap", "-t", "/p"}),
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
      {"bagwiz", "__complete", "6", "bagwiz", "traj", "dump", "-i", "~/fixture.mcap", "-t",
       "/poi"}),
    "");
}

// A bag whose topics are all unsupported types yields no <topic> candidates, so
// the shell's default file completion takes over.
TEST_F(CompletionTest, TrajDumpTopicCompletionEmptyWhenNoSupportedTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "unsupported.mcap");  // String + Int32 only

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "traj", "dump", "-i", "~/unsupported.mcap", "-t"}),
    "");
}

// `traj join -t <TAB>` names a new topic to embed the trajectory under, not a
// topic that already exists in the bag, so no bag-topic candidates are offered.
TEST_F(CompletionTest, TrajJoinTopicCompletionListsBagTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");
  const auto traj_arg = (tmp_dir_ / "in.tum").string();

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "traj", "join", "-i", "~/fixture.mcap", "--traj",
       traj_arg, "-t"}),
    "");
}

// With the flag-only surface, topic completion requires an explicit -i/--input.
// Completing a -t/--topic value when no -i is given must stay empty so the
// shell's file completion takes over instead of trying to open a non-existent bag.
TEST_F(CompletionTest, WalkTopicCompletionSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(run_completion({"bagwiz", "__complete", "3", "bagwiz", "walk", "-t"}), "");
}

TEST_F(CompletionTest, TrajDumpTopicCompletionSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  // With the flag-only surface, a flag value in the -i/--input slot must not
  // be treated as a bag path. Topic completion for -t/--topic must bail out
  // and produce no candidates.
  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "traj", "dump", "-i", "--unknown-flag", "-t"}),
    "");
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
      {"bagwiz", "__complete", "7", "bagwiz", "traj", "dump", "-i", "~/fixture.mcap", "--format"}),
    "tum\n");
}

// `--ref <TAB>` after a TF-bearing bag must list every distinct frame
// id reachable from the bag's /tf message(s), sorted and deduplicated.
TEST_F(CompletionTest, TrajDumpRefFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", "~/tf.mcap", "-t", "/tf", "-o",
       "out.tum", "--ref"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `--of <TAB>` shares the same value source as `--ref`. Pin both so
// that a future divergence cannot silently regress one branch.
TEST_F(CompletionTest, TrajDumpOfFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", "~/tf.mcap", "-t", "/tf", "-o",
       "out.tum", "--of"}),
    "base_link\nlidar\nmap\nodom\n");
}

// Typed prefix narrows the candidates to matching frame ids. Validates
// the prefix filter that `complete_frame_id_value` applies, including
// the case where a partial prefix matches no frames (returns empty).
TEST_F(CompletionTest, TrajDumpRefFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", "~/tf.mcap", "-t", "/tf", "-o",
       "out.tum", "--ref", "ba"}),
    "base_link\n");
}

// `traj join` reuses the same `complete_traj_frame_id` helper since
// it puts the bag at the same positional slot. Verify it actually
// surfaces frame ids too — guards against a regression where someone
// later restricts the helper to just `traj dump`.
TEST_F(CompletionTest, TrajJoinRefFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "join", "-i", "~/tf.mcap", "--traj",
       "in.tum", "-t", "/tf", "--ref"}),
    "base_link\nlidar\nmap\nodom\n");
}

// When the bag opens successfully but carries no tf2_msgs/msg/TFMessage
// topic, completion has nothing to suggest and returns empty — completion
// simply offers nothing rather than surfacing a placeholder candidate.
TEST_F(CompletionTest, TrajDumpRefFlagEmptyWhenBagHasNoTf)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "no_tf.mcap");  // String + Int32, no TF

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", "~/no_tf.mcap", "-t", "/tf",
       "-o", "out.tum", "--ref"}),
    "");
}

// An input path that fails to open must not surface the sentinel — that would
// mislead the user into believing the bag exists but is empty of TF.
// The contract is "silent fall-through so the shell's file completion
// takes over", matching how `complete_topics` handles bad inputs.
TEST_F(CompletionTest, TrajDumpRefFlagEmptyForMissingBag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", "~/missing.mcap", "-t", "/tf",
       "-o", "out.tum", "--ref"}),
    "");
}

// A flag in the bag slot must not be passed to io::open_read by the
// frame-id completer. Pins the same defensive gate that the topic
// binding applies for `walk` / `traj`.
TEST_F(CompletionTest, TrajDumpRefFlagSuppressedWhenBagSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "9", "bagwiz", "traj", "dump", "--unknown-flag", "-t", "/tf", "-o",
       "out.tum", "--ref"}),
    "");
}

// Typing `-` at the bagwiz top-level should list the implicit CLI11 help
// flags plus the `--version` flag wired up in main().
TEST(FlagCompletionTest, TopLevelDashListsHelpAndVersion)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "1", "bagwiz", "-"}), "--help\n--version\n-h\n");
}

// `ls` surfaces its own `-l`/`--long` flag plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, LsDashListsLsFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "ls", "-"}),
    "--help\n--input\n--long\n-h\n-i\n-l\n");
}

// `trim` surfaces its window and output flags plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, TrimDashListsTrimFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "trim", "-"}),
    "--align\n--both\n--duration\n--end\n--help\n--input\n--output\n--overwrite\n--stamp\n--"
    "start\n-h\n-i\n-o\n-w\n");
}

// The value of `--stamp` completes to its two clock choices.
TEST(FlagCompletionTest, TrimStampValueCompletes)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "trim", "--stamp"}), "header\nrecv\n");
}

// `walk` surfaces its own `--cam-info` flag plus the implicit help flags, sorted.
// Topic completion is gated off via the `-` prefix, so the binding does
// not call into the bag reader here.
TEST(FlagCompletionTest, WalkDashListsWalkFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "walk", "-"}),
    "--cam-info\n--help\n--input\n--topic\n-h\n-i\n-t\n");
}

// The `complete` subcommand defines two flags of its own; with help merged
// in they sort as: --help, --install, --overwrite, -h, -w.
TEST(FlagCompletionTest, CompleteDashListsCompleteFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "complete", "-"}),
    "--help\n--install\n--overwrite\n--shell\n-h\n-w\n");
}

// `convert` has no parent-level flags; the `format` subcommand owns
// -w/--overwrite/--storage. Both contexts must respond to `-`.
TEST(FlagCompletionTest, ConvertParentDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "convert", "-"}), "--help\n-h\n");
}

TEST(FlagCompletionTest, ConvertFormatDashListsFormatFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "convert", "format", "-"}),
    "--help\n--input\n--output\n--overwrite\n--storage\n-h\n-i\n-o\n-w\n");
}

// `bagwiz convert <TAB>` lists its only subcommand.
TEST(FlagCompletionTest, ConvertSubcommandListsFormat)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "convert", ""}), "format\n");
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
    "--format\n--help\n--input\n--of\n--output\n--overwrite\n--ref\n--topic\n-f\n-h\n-i\n-o\n-t\n-"
    "w\n");
}

TEST(FlagCompletionTest, TrajJoinDashListsJoinFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "join", "-"}),
    "--force\n--format\n--help\n--input\n--msg-type\n--of\n--output\n--overwrite\n--ref\n--topic\n-"
    "-traj\n-h\n-i\n-m\n-o\n-t\n-w\n");
}

TEST(FlagCompletionTest, TfParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "tf", "-"}), "--help\n-h\n");
}

// `tf tree -` surfaces its own --topics/-t flag plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, TfTreeDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "tf", "tree", "-"}),
    "--help\n--input\n--topics\n-h\n-i\n-t\n");
}

// `bagwiz tf <TAB>` lists all subcommands, sorted.
TEST(FlagCompletionTest, TfSubcommandListsStaticAndTree)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "tf", ""}), "static\ntree\n");
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

// `tf static calc -` surfaces the action's own --json/--of/--ref flags alongside
// the implicit help flags, sorted.
TEST(FlagCompletionTest, TfStaticCalcDashListsStaticFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "static", "calc", "-"}),
    "--help\n--input\n--json\n--of\n--ref\n-h\n-i\n");
}

// `tf static cp -` surfaces the copy action's flags (--output/-o, -w/--overwrite)
// alongside the implicit help flags, sorted. <src>/<dst> are bag paths, so they
// carry no bagwiz candidates and fall through to the shell's file completion.
TEST(FlagCompletionTest, TfStaticCpDashListsCpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "static", "cp", "-"}),
    "--dst\n--help\n--output\n--overwrite\n--src\n-h\n-o\n-w\n");
}

// `tf static calc <bag> --of <TAB>` lists only frame ids from the bag's static
// TF (*tf_static) topics, since `tf static calc` resolves the static tree. The
// mixed fixture's /tf_static carries map→odom→base_link.
TEST_F(CompletionTest, TfStaticCalcOfFlagListsStaticFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "tf", "static", "calc", "-i", "~/mixed.mcap",
       "--of"}),
    "base_link\nmap\nodom\n");
}

// `--ref <TAB>` shares the same static-only frame-id source as `--of`.
TEST_F(CompletionTest, TfStaticCalcRefFlagListsStaticFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "tf", "static", "calc", "-i", "~/mixed.mcap",
       "--ref"}),
    "base_link\nmap\nodom\n");
}

// A typed prefix narrows the static --of frame-id candidates.
TEST_F(CompletionTest, TfStaticCalcOfFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "tf", "static", "calc", "-i", "~/mixed.mcap", "--of",
       "ba"}),
    "base_link\n");
}

// A bag with only a dynamic /tf topic (no *tf_static) has no static frames, so
// `tf static calc --of <TAB>` returns empty rather than listing the dynamic
// frames — confirming the static-only restriction. This is the one test in the
// suite that can tell "static filtering applied" apart from "no filtering at
// all": the fixture carries base_link/lidar/map/odom on /tf alone, so an
// unfiltered lookup would surface them. Every other static fixture
// (write_mixed_tf_mcap_fixture) carries identical frames on /tf and /tf_static
// and so cannot distinguish the two.
TEST_F(CompletionTest, TfStaticCalcOfFlagExcludesDynamicOnlyFrames)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");  // /tf only, no /tf_static

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "tf", "static", "calc", "-i", "~/tf.mcap", "--of"}),
    "");
}

// `tf tree <input> -t <TAB>` offers only the bag's TFMessage topics -- not the
// other topics the same fixture carries.
TEST_F(CompletionTest, TfTreeTopicsFlagListsOnlyTfMessageTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "tf", "tree", "-i", "~/mixed.mcap", "-t"}),
    "/tf\n/tf_static\n");
}

// A typed prefix narrows the candidates within the TF set. --topics is variadic,
// so this also stands in for the second-value-slot case the old
// TfTreeSecondTopicSlotRespectsPrefix covered.
TEST_F(CompletionTest, TfTreeTopicsFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "tf", "tree", "-i", "~/mixed.mcap", "-t", "/tf_"}),
    "/tf_static\n");
}

// A bag with no tf2_msgs/msg/TFMessage topic yields no candidates, so the
// shell's default file completion takes over (matches walk/traj behavior).
TEST_F(CompletionTest, TfTreeTopicsFlagEmptyWhenBagHasNoTf)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "no_tf.mcap");  // String + Int32, no TF

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "tf", "tree", "-i", "~/no_tf.mcap", "-t"}),
    "");
}

// Completing the <input> slot itself (cursor on word 2, before -t) must not
// trigger tf-tree topic completion; the cursor-position guard bails so the
// shell's file completion handles the bag path. This matters more now, not
// less: with topics gone from the positionals, a bug that completed topics at
// <input> would have nothing else to catch it.
TEST_F(CompletionTest, TfTreeInputSlotDoesNotListTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "4", "bagwiz", "tf", "tree", "-i", "~/mixed.mcap"}),
    "");
}

// THE DISCRIMINATOR. Under the old positional binding (topic_word=3, variadic)
// this slot offered the TF topics; under the flag binding it must offer nothing.
TEST_F(CompletionTest, TfTreeBareSlotAfterInputOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "tf", "tree", "-i", "~/mixed.mcap"}),
    "");
}

// A bag path that does not exist yields no candidates: the reader throws and
// complete_topics swallows it, so the shell's file completion takes over
// instead of surfacing a misleading empty TF result.
TEST_F(CompletionTest, TfTreeTopicsFlagEmptyForMissingBag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "tf", "tree", "-i", "~/missing.mcap", "-t"}),
    "");
}

// --topics is variadic, so a second and later value slot completes too.
TEST_F(CompletionTest, TfTreeTopicsFlagCompletesEveryValueSlot)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mixed_tf_mcap_fixture(tmp_dir_ / "mixed.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "tf", "tree", "-i", "~/mixed.mcap", "-t", "/tf"}),
    "/tf\n/tf_static\n");
}

// Prefix narrowing still works once the flag candidate set is widened.
// `--o` now matches both `--of` and `--overwrite`, sorted lexicographically.
TEST(FlagCompletionTest, TrajDumpDoubleDashOPrefixSelectsOfAndOverwrite)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "traj", "dump", "--o"}),
    "--of\n--output\n--overwrite\n");
}

// `topic drop <input> -t <TAB>` offers every topic in the bag -- drop takes
// selectors, so unlike cam-info's bindings it has no type filter.
TEST_F(CompletionTest, TopicDropTopicsFlagListsAllTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "topic", "drop", "-i", "~/fixture.mcap", "-t"}),
    "/bar\n/foo\n");
}

// The long form completes identically.
TEST_F(CompletionTest, TopicDropLongTopicsFlagCompletes)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "topic", "drop", "-i", "~/fixture.mcap", "--topics"}),
    "/bar\n/foo\n");
}

// --topics is variadic, so a second and later value slot completes too.
TEST_F(CompletionTest, TopicDropTopicsFlagCompletesEveryValueSlot)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "topic", "drop", "-i", "~/fixture.mcap", "-t",
       "/foo"}),
    "/bar\n/foo\n");
}

// A typed prefix narrows the candidates.
TEST_F(CompletionTest, TopicDropTopicsFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "topic", "drop", "-i", "~/fixture.mcap", "-t", "/f"}),
    "/foo\n");
}

// THE DISCRIMINATOR. Under the old positional binding (topic_word=3, variadic)
// this slot offered every topic; under the flag binding it must offer nothing.
// Without this, every test above passes under BOTH bindings -- they only prove
// "some binding exists", not that the positional slot is gone.
TEST_F(CompletionTest, TopicDropBareSlotAfterInputOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "topic", "drop", "-i", "~/fixture.mcap"}),
    "");
}

// `bagwiz topic <TAB>` lists the command group's action verbs, sorted.
TEST(FlagCompletionTest, TopicSubcommandListsDropKeepAndRename)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "topic", ""}), "drop\nkeep\nrename\n");
}

// `bagwiz map <TAB>` lists the command group's action verbs (slam, viewer),
// sorted.
TEST(FlagCompletionTest, MapSubcommandListsSlamAndViewer)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "map", ""}), "slam\nviewer\n");
}

// A partial verb narrows the candidates.
TEST(FlagCompletionTest, MapSubcommandPrefixNarrowsToViewer)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "map", "v"}), "viewer\n");
}

// `map viewer <TAB>`: the <map> positional is a path, left to shell file
// completion, so no candidates are emitted.
TEST(FlagCompletionTest, MapViewerMapSlotDefersToShell)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "4", "bagwiz", "map", "viewer", "-m", ""}), "");
}

// `map viewer -` surfaces only the implicit help flags (viewer has no other
// flags).
TEST(FlagCompletionTest, MapViewerDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "map", "viewer", "-"}),
    "--help\n--map\n-h\n-m\n");
}

// `map slam -` surfaces the slam action's flags plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, MapSlamDashListsSlamFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "map", "slam", "-"}),
    "--backend\n--cam\n--cam-info\n--cam-keyframe-blur\n--cam-min-dist\n--dynamic-dp\n--dynamic-"
    "ds\n--dynamic-res\n--fill-min-inliers\n--frame\n--gnss\n--help\n--imu\n--input\n--input-"
    "res\n--max-range\n--min-range\n--no-color-propagate\n--no-cooldown-fill\n--no-progress\n--no-"
    "warmup-fill\n--outlier-k\n--outlier-r\n--output\n--overwrite\n--pcd\n--remove-dynamic\n--"
    "remove-outliers\n--submap-keyframes\n--threads\n--viewer\n-h\n-i\n-j\n-o\n-w\n");
}

// `map slam --backend <TAB>` lists the three backend modes.
TEST(FlagCompletionTest, MapSlamBackendListsModes)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "map", "slam", "--backend"}),
    "auto\ncpu\ncuda\n");
}

// `map slam <input> <pcd_topic>` completes the topic slot from the bag's
// PointCloud2 topics, excluding other types.
TEST_F(CompletionTest, MapSlamTopicCompletionListsOnlyPointCloud2)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "map", "slam", "-i", "~/fixture.mcap", "--pcd"}),
    "/points\n");
}

// `pcd concat -` surfaces concat's flags plus the implicit help flags, sorted.
// Guards the flag list against drift and proves the --stamp-offset value
// completion below did not disturb the `-` branch.
TEST(FlagCompletionTest, PcdConcatDashListsConcatFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "pcd", "concat", "-"}),
    "--drop-inputs\n--force\n--frame\n--help\n--input\n--output\n--overwrite\n--pcd\n--stamp-"
    "offset\n--threads\n--tolerance\n--topic\n-h\n-i\n-j\n-o\n-t\n-w\n");
}

// `pcd concat <bag> <out> --stamp-offset <TAB>` completes the <topic> half of the
// topic=value argument from the bag's PointCloud2 topics (like --pcd),
// emitting a trailing '=' so the shell leaves the cursor ready for the value.
TEST_F(CompletionTest, PcdConcatStampOffsetCompletesPointCloud2TopicsWithEquals)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o", "/out",
       "--stamp-offset"}),
    "/points=\n");
}

// A typed prefix narrows the --stamp-offset topic candidates.
TEST_F(CompletionTest, PcdConcatStampOffsetRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o", "/out",
       "--stamp-offset", "/po"}),
    "/points=\n");
}

// Once the value word already contains '=', the topic is chosen and the value (a
// duration) has nothing to suggest.
TEST_F(CompletionTest, PcdConcatStampOffsetOffersNothingAfterEquals)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o", "/out",
       "--stamp-offset", "/points=50"}),
    "");
}

// A bag that fails to open yields no --stamp-offset candidates; completion is
// best-effort and falls through to the shell's file completion.
TEST_F(CompletionTest, PcdConcatStampOffsetUnknownBagYieldsNoCandidates)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "concat", "-i", "~/missing.mcap", "-o", "/out",
       "--stamp-offset"}),
    "");
}

// One --stamp-offset occurrence consumes several <topic>=<value> values, so the
// <topic> half completes for the second value in the same run too.
TEST_F(CompletionTest, PcdConcatStampOffsetCompletesSecondValueInSameRun)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "9", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o", "/out",
       "--stamp-offset", "/points=50"}),
    "/points=\n");
}

// A typed prefix narrows the second value's topic candidates as well.
TEST_F(CompletionTest, PcdConcatStampOffsetSecondValueRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o",
       "/out", "--stamp-offset", "/points=50", "/po"}),
    "/points=\n");
}

// The run keeps completing for a third value after two complete ones.
TEST_F(CompletionTest, PcdConcatStampOffsetCompletesThirdValueInSameRun)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o",
       "/out", "--stamp-offset", "/points=50", "/points2=100"}),
    "/points=\n");
}

// Bash splits a typed value at COMP_WORDBREAKS' '=', so the same second-value
// completion must work when the first value arrives as three separate words.
TEST_F(CompletionTest, PcdConcatStampOffsetCompletesAfterBashSplitValue)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "11", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o",
       "/out", "--stamp-offset", "/points", "=", "50"}),
    "/points=\n");
}

// With bash's '=' splitting, the cursor right after '=' sits on the <value>
// half (a duration), which has nothing to suggest — the walk-back must not
// offer topics there.
TEST_F(CompletionTest, PcdConcatStampOffsetOffersNothingOnBashSplitValueHalf)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "pcd", "concat", "-i", "~/fixture.mcap", "-o",
       "/out", "--stamp-offset", "/points", "="}),
    "");
}

// `pcd <TAB>` lists the command group's two subcommands.
TEST(FlagCompletionTest, PcdSubcommandListsConcatAndUndistort)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "pcd", ""}), "concat\nundistort\n");
}

// `pcd undistort -` surfaces undistort's flags plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, PcdUndistortDashListsUndistortFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "pcd", "undistort", "-"}),
    "--help\n--input\n--of\n--output\n--overwrite\n--pcd\n--pose\n--ref\n--threads\n-h\n-i\n-j\n-"
    "o\n-w\n");
}

// `pcd undistort <bag> <pose_topic> --pcd <TAB>` completes PointCloud2 topics
// from the bag named at the <input> slot, mirroring concat's --pcd.
TEST_F(CompletionTest, PcdUndistortPcdCompletesPointCloud2Topics)
{
  const HomeEnvGuard home_guard(tmp_dir_);
  write_pointcloud2_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "undistort", "-i", "~/fixture.mcap", "--pose",
       "/pose", "--pcd", ""}),
    "/points\n");
}

// `--ref <TAB>` after a TF-bearing bag lists every distinct frame id
// reachable from the bag's /tf message(s), sorted and deduplicated — pcd
// undistort reuses the same complete_frame_id_arg helper as traj dump/join.
TEST_F(CompletionTest, PcdUndistortRefFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "undistort", "-i", "~/tf.mcap", "--pose",
       "/pose", "--ref"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `--of <TAB>` shares the same value source as `--ref`. Pin both so that a
// future divergence cannot silently regress one branch.
TEST_F(CompletionTest, PcdUndistortOfFlagListsBagFrameIds)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_tf_mcap_fixture(tmp_dir_ / "tf.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "pcd", "undistort", "-i", "~/tf.mcap", "--pose",
       "/pose", "--of"}),
    "base_link\nlidar\nmap\nodom\n");
}

// `bagwiz topic -` is the command-group slot; only the implicit help flags
// appear (drop's own flags live one slot deeper).
TEST(FlagCompletionTest, TopicParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "topic", "-"}), "--help\n-h\n");
}

// `topic drop -` surfaces the action's flags (--output/-o, --overwrite,
// --topics/-t) plus the implicit help flags, sorted.
TEST(FlagCompletionTest, TopicDropDashListsDropFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "topic", "drop", "-"}),
    "--help\n--input\n--output\n--overwrite\n--topics\n-h\n-i\n-o\n-t\n-w\n");
}

// `keep` binds the same way as `drop`.
TEST_F(CompletionTest, TopicKeepTopicsFlagListsAllTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "topic", "keep", "-i", "~/fixture.mcap", "-t"}),
    "/bar\n/foo\n");
}

TEST_F(CompletionTest, TopicKeepTopicsFlagCompletesEveryValueSlot)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "topic", "keep", "-i", "~/fixture.mcap", "-t",
       "/foo"}),
    "/bar\n/foo\n");
}

// THE DISCRIMINATOR. Under the old positional binding (topic_word=3, variadic)
// this slot offered every topic; under the flag binding it must offer nothing.
// Without this, every test above passes under BOTH bindings -- they only prove
// "some binding exists", not that the positional slot is gone.
TEST_F(CompletionTest, TopicKeepBareSlotAfterInputOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "5", "bagwiz", "topic", "keep", "-i", "~/fixture.mcap"}),
    "");
}

// `topic rename <bag> <TAB>` (the <src_topic> slot) lists every topic in the
// bag — any existing topic can be the rename source.
TEST_F(CompletionTest, TopicRenameSrcSlotListsAllTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "topic", "rename", "-i", "~/fixture.mcap", "-s"}),
    "/bar\n/foo\n");
}

// `topic rename <bag> <src> <TAB>` is the <dst_topic> slot — a brand-new name,
// so nothing is suggested (the binding is non-variadic and fires only at the
// <src_topic> slot).
TEST_F(CompletionTest, TopicRenameDstSlotOffersNothing)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "fixture.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "topic", "rename", "-i", "~/fixture.mcap", "-s",
       "/foo", "-d"}),
    "");
}

// `topic rename -` surfaces the action's flags (--output/-o, --overwrite) plus
// the implicit help flags, sorted.
TEST(FlagCompletionTest, TopicRenameDashListsRenameFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "topic", "rename", "-"}),
    "--dst-topic\n--help\n--input\n--output\n--overwrite\n--src-topic\n-d\n-h\n-i\n-o\n-s\n-w\n");
}

// `topic keep -` surfaces the action's flags (--output/-o, --overwrite,
// --topics/-t) plus the implicit help flags, sorted.
TEST(FlagCompletionTest, TopicKeepDashListsKeepFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "topic", "keep", "-"}),
    "--help\n--input\n--output\n--overwrite\n--topics\n-h\n-i\n-o\n-t\n-w\n");
}

// `bagwiz generate <TAB>` lists the command group's single subcommand.
TEST(FlagCompletionTest, GenerateSubcommandListsVideo)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "generate", ""}), "video\n");
}

// `bagwiz generate -` is the command-group slot; only the implicit help flags
// appear (video's own flags live one slot deeper).
TEST(FlagCompletionTest, GenerateParentDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "generate", "-"}), "--help\n-h\n");
}

// `generate video -` surfaces the action's flags plus the implicit help flags,
// sorted.
TEST(FlagCompletionTest, GenerateVideoDashListsVideoFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "generate", "video", "-"}),
    "--alpha\n--cam-info\n--field\n--help\n--input\n--max\n--min\n--output\n--overwrite\n--pcd\n--"
    "point-size\n--resize\n--scheme\n--topic\n--undistort\n-h\n-i\n-o\n-t\n-w\n");
}

// `--field <TAB>` offers the valid point-cloud field choices, sorted.
TEST(FlagCompletionTest, GenerateVideoFieldFlagListsChoices)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "generate", "video", "--field"}),
    "distance\nintensity\nx\ny\nz\n");
}

// `--scheme <TAB>` offers the valid color scheme choices, sorted.
TEST(FlagCompletionTest, GenerateVideoSchemeFlagListsChoices)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "generate", "video", "--scheme"}),
    "inferno\njet\nmagma\nplasma\nrainbow\nturbo\nviridis\n");
}

// `generate video <bag> <TAB>` (the <image_topic> slot) lists only the bag's image
// topics — /image (Image) and /image/compressed (CompressedImage) here —
// excluding the non-image /points, sorted.
TEST_F(CompletionTest, GenerateVideoTopicSlotListsOnlyImageTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "generate", "video", "-i", "~/images.mcap", "-t"}),
    "/image\n/image/compressed\n");
}

// A typed prefix narrows the <image_topic> candidates within the image-type set.
TEST_F(CompletionTest, GenerateVideoTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "generate", "video", "-i", "~/images.mcap", "-t",
       "/image/"}),
    "/image/compressed\n");
}

// A prefix that matches only a non-image topic (/points) yields nothing: the
// type filter excludes it even though the name matches.
TEST_F(CompletionTest, GenerateVideoTopicSlotExcludesUnsupportedTypeOnPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "generate", "video", "-i", "~/images.mcap", "-t",
       "/p"}),
    "");
}

// `--cam-info <TAB>` after a bag offers only sensor_msgs/msg/CameraInfo
// topics, excluding image and non-CameraInfo topics.
TEST_F(CompletionTest, GenerateVideoCameraInfoFlagListsOnlyCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "generate", "video", "-i", "~/cameras.mcap", "-t",
       "/cam/image_raw/compressed", "-o", "out.avi", "--cam-info"}),
    "/cam/camera_info\n");
}

// A typed prefix narrows the `--cam-info` candidates within the CameraInfo set.
TEST_F(CompletionTest, GenerateVideoCameraInfoFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "generate", "video", "-i", "~/cameras.mcap", "-t",
       "/cam/image_raw/compressed", "-o", "out.avi", "--cam-info", "/cam/c"}),
    "/cam/camera_info\n");
}

// A bag with no CameraInfo topic yields no `--cam-info` candidates.
TEST_F(CompletionTest, GenerateVideoCameraInfoFlagEmptyWhenNoCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");  // no CameraInfo

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "generate", "video", "-i", "~/images.mcap", "-t",
       "/image", "-o", "out.avi", "--cam-info"}),
    "");
}

// A bag with no image topic yields no <image_topic> candidates, so the shell's default
// file completion takes over (matches walk/traj/tf behavior).
TEST_F(CompletionTest, GenerateVideoTopicSlotEmptyWhenNoImageTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_mcap_fixture(tmp_dir_ / "no_image.mcap");  // String + Int32, no image

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "generate", "video", "-i", "~/no_image.mcap", "-t"}),
    "");
}

// A flag in the input slot must not cause the topic binding to call the bag
// reader on a flag-shaped path; the binding's earlier-slot guard bails out.
TEST_F(CompletionTest, GenerateVideoTopicSlotSuppressedWhenInputSlotIsFlag)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");

  EXPECT_EQ(run_completion({"bagwiz", "__complete", "4", "bagwiz", "generate", "video", "-t"}), "");
}

// `walk <input> <topic> --cam-info <TAB>` offers only the bag's
// sensor_msgs/msg/CameraInfo topics, mirroring `generate video --cam-info`.
TEST_F(CompletionTest, WalkCameraInfoFlagListsOnlyCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "walk", "-i", "~/cameras.mcap", "-t",
       "/cam/image_raw/compressed", "--cam-info"}),
    "/cam/camera_info\n");
}

// A typed prefix narrows the walk `--cam-info` candidates within the CameraInfo set.
TEST_F(CompletionTest, WalkCameraInfoFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "walk", "-i", "~/cameras.mcap", "-t",
       "/cam/image_raw/compressed", "--cam-info", "/cam/c"}),
    "/cam/camera_info\n");
}

// `cam-info recompute-p <input> --topics <TAB>` offers only the bag's CameraInfo
// topics -- not the CompressedImage or PointCloud2 topics the same fixture
// carries. recompute-p takes its topics via --topics rather than as positionals.
TEST_F(CompletionTest, CamInfoRecomputePTopicsFlagListsOnlyCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
       "--topics"}),
    "/cam/camera_info\n");
}

// The short form completes identically.
TEST_F(CompletionTest, CamInfoRecomputePShortTopicsFlagCompletes)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
       "-t"}),
    "/cam/camera_info\n");
}

// --topics is variadic, so a second and later value slot completes too. This is
// what a plain "previous word is the flag" check would miss.
TEST_F(CompletionTest, CamInfoRecomputePTopicsFlagCompletesEveryValueSlot)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
       "--topics", "/cam/camera_info"}),
    "/cam/camera_info\n");
}

// A typed prefix narrows the candidates within the CameraInfo set.
TEST_F(CompletionTest, CamInfoRecomputePTopicsFlagRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
       "--topics", "/cam/c"}),
    "/cam/camera_info\n");
}

// A different flag's value slot must not borrow --topics' candidates.
TEST_F(CompletionTest, CamInfoRecomputePOtherFlagValueOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
       "--alpha"}),
    "");
}

// recompute-p surfaces its own flags, not replace's (--frame-id is replace-only).
TEST_F(CompletionTest, CamInfoRecomputePListsItsOwnFlags)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  const auto out = run_completion(
    {"bagwiz", "__complete", "5", "bagwiz", "cam-info", "recompute-p", "-i", "~/cameras.mcap",
     "-"});
  EXPECT_NE(out.find("--topics"), std::string::npos) << out;
  EXPECT_NE(out.find("--alpha"), std::string::npos) << out;
  EXPECT_EQ(out.find("--frame-id"), std::string::npos) << out;
}

// A bag with no CameraInfo topic yields no walk `--cam-info` candidates, so the
// shell's default file completion takes over.
TEST_F(CompletionTest, WalkCameraInfoFlagEmptyWhenNoCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_image_topics_fixture(tmp_dir_ / "images.mcap");  // no CameraInfo

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "walk", "-i", "~/images.mcap", "-t", "/image",
       "--cam-info"}),
    "");
}

// `bagwiz cam-info <TAB>` lists its subcommands. Candidates come back sorted.
TEST(FlagCompletionTest, CamInfoSubcommandListsActions)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "cam-info", ""}),
    "dump\nrecompute-p\nreplace\n");
}

// `bagwiz cam-info -` lists just the implicit help flags (the group itself defines
// no flags of its own).
TEST(FlagCompletionTest, CamInfoParentDashListsHelpFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "2", "bagwiz", "cam-info", "-"}), "--help\n-h\n");
}

// `bagwiz cam-info replace -` lists replace's flags, sorted, with help merged.
TEST(FlagCompletionTest, CamInfoReplaceDashListsReplaceFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "cam-info", "replace", "-"}),
    "--frame-id\n--help\n--input\n--output\n--overwrite\n--topics\n--yaml\n-h\n-i\n-o\n-t\n-w\n");
}

// `cam-info replace <input> <yaml> -t <TAB>` offers only the bag's
// CameraInfo topics -- not the CompressedImage or PointCloud2 topics the same
// fixture carries.
TEST_F(CompletionTest, CamInfoReplaceTopicsFlagListsOnlyCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "cam-info", "replace", "-i", "~/cameras.mcap",
       "--yaml", "calib.yaml", "-t"}),
    "/cam/camera_info\n");
}

// The long form completes identically.
TEST_F(CompletionTest, CamInfoReplaceLongTopicsFlagCompletes)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "8", "bagwiz", "cam-info", "replace", "-i", "~/cameras.mcap",
       "--yaml", "calib.yaml", "--topics"}),
    "/cam/camera_info\n");
}

// --topics is variadic, so a second and later value slot completes too.
TEST_F(CompletionTest, CamInfoReplaceTopicsFlagCompletesEveryValueSlot)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "9", "bagwiz", "cam-info", "replace", "-i", "~/cameras.mcap",
       "--yaml", "calib.yaml", "-t", "/cam/camera_info"}),
    "/cam/camera_info\n");
}

// The <yaml> slot is a path, not a topic list: it must not offer topics
// now that they are no longer positional there.
TEST_F(CompletionTest, CamInfoReplaceYamlSlotOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "replace", "-i", "~/cameras.mcap",
       "--yaml"}),
    "");
}

// THE DISCRIMINATOR. Under the old positional binding (topic_word=4, variadic)
// this slot offered the CameraInfo topics; under the flag binding there is no
// bare topic slot after the --yaml value, so completion offers nothing.
TEST_F(CompletionTest, CamInfoReplaceBareSlotAfterYamlOffersNoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "7", "bagwiz", "cam-info", "replace", "-i", "~/cameras.mcap",
       "--yaml", "calib.yaml"}),
    "");
}

// `cam-info dump <input> <TAB>` offers only the bag's CameraInfo topics -- not
// the CompressedImage or PointCloud2 topics the same fixture carries. <topic> is
// at word 3, one left of replace's, because dump has no <calib_yaml>.
TEST_F(CompletionTest, CamInfoDumpTopicSlotListsOnlyCameraInfoTopics)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "dump", "-i", "~/cameras.mcap", "-t"}),
    "/cam/camera_info\n");
}

// A typed prefix narrows the candidates within the CameraInfo set.
TEST_F(CompletionTest, CamInfoDumpTopicSlotRespectsPrefix)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "6", "bagwiz", "cam-info", "dump", "-i", "~/cameras.mcap", "-t",
       "/cam/c"}),
    "/cam/camera_info\n");
}

// Unlike replace's, dump's binding is non-variadic: a camera_calibration YAML
// holds one calibration. With a non-topic flag following `-t`, the cursor leaves
// the topic value slot and nothing is suggested.
TEST_F(CompletionTest, CamInfoDumpTopicSlotIsNotVariadic)
{
  const HomeEnvGuard home_guard(tmp_dir_);

  write_camera_info_fixture(tmp_dir_ / "cameras.mcap");

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "9", "bagwiz", "cam-info", "dump", "-i", "~/cameras.mcap", "-t",
       "/cam/camera_info", "-o", "out"}),
    "");
}

// `bagwiz check <TAB>` lists its single subcommand.
TEST(FlagCompletionTest, CheckSubcommandListsBroken)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "check", ""}), "broken\n");
}

// `bagwiz check -` lists just the implicit help flags (the group itself defines no
// flags of its own).
TEST(FlagCompletionTest, CheckParentDashListsHelpFlags)
{
  EXPECT_EQ(run_completion({"bagwiz", "__complete", "2", "bagwiz", "check", "-"}), "--help\n-h\n");
}

// `bagwiz check broken -` lists broken's flags, sorted, with help merged.
TEST(FlagCompletionTest, CheckBrokenDashListsBrokenFlags)
{
  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "3", "bagwiz", "check", "broken", "-"}),
    "--deep\n--help\n--input\n--rm\n-h\n-i\n");
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

// `--stamp-offset` emits `<topic>=` candidates; the bash script must suppress the
// auto-inserted trailing space for those so the value can follow the '='.
TEST(CompletionScriptTest, BashScriptSuppressesSpaceForEqualsCandidates)
{
  const auto script = bagwiz::commands::completion_script_for("bash");
  ASSERT_TRUE(script.has_value());
  EXPECT_NE(script->find("compopt -o nospace"), std::string::npos);
}

// zsh equivalent: `<topic>=` candidates are added with an empty suffix so no
// trailing space is inserted after the '='.
TEST(CompletionScriptTest, ZshScriptSuppressesSpaceForEqualsCandidates)
{
  const auto script = bagwiz::commands::completion_script_for("zsh");
  ASSERT_TRUE(script.has_value());
  EXPECT_NE(script->find("compadd -S ''"), std::string::npos);
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

// A bare single-file `.db3.zstd` envelope has no metadata.yaml, so listing its
// topics forces a full decompress of the whole database — seconds of hang per
// TAB on a multi-GB bag. Topic completion must offer nothing instead. Without
// the guard this same bag would decompress and list /foo and /bar.
TEST_F(CompletionTest, WalkTopicCompletionSkipsBareSingleFileZstd)
{
  const auto bag = tmp_dir_ / "single";
  write_sqlite3_dir_bag(bag, {{"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}});
  const auto zstd_path = make_file_mode_zstd(bag);

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "walk", "-i", zstd_path.string(), "-t"}),
    "");
}

// A directory bag (even FILE-mode) serves its topic list from metadata.yaml
// without decompressing the envelope, so topic completion stays fast and must
// NOT be skipped. This uncompressed sqlite3 directory bag proves the fixture
// builds a real, listable bag.
TEST_F(CompletionTest, WalkTopicCompletionListsSqlite3DirectoryTopics)
{
  const auto bag = tmp_dir_ / "dir";
  write_sqlite3_dir_bag(bag, {{"/foo", "std_msgs/msg/String"}, {"/bar", "std_msgs/msg/Int32"}});

  EXPECT_EQ(
    run_completion({"bagwiz", "__complete", "5", "bagwiz", "walk", "-i", bag.string(), "-t"}),
    "/bar\n/foo\n");
}

// frame-id discovery iterates TF messages; on a FILE-mode zstd bag the first read
// decompresses the whole shard to a temp .db3. `traj dump --ref` must offer
// nothing instead of hanging, even though the bag declares a /tf topic.
TEST_F(CompletionTest, TrajDumpRefFlagSkipsFileModeZstd)
{
  const auto bag = tmp_dir_ / "tf_file_mode";
  write_sqlite3_dir_bag(
    bag, {{"/tf", "tf2_msgs/msg/TFMessage"}, {"/points", "sensor_msgs/msg/PointCloud2"}});
  make_file_mode_zstd(bag);

  EXPECT_EQ(
    run_completion(
      {"bagwiz", "__complete", "10", "bagwiz", "traj", "dump", "-i", bag.string(), "-t", "/tf",
       "-o", "out.tum", "--ref"}),
    "");
}
