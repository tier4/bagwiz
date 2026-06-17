// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pipeline/pipelined_backend.hpp"

#include "bag_equal.hpp"  // NOLINT(build/include_subdir)  sibling test header, resolves relative
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/bounded_message_queue.hpp"
#include "bagwiz/core/pipeline/owned_message.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/core/pipeline/sequential_backend.hpp"
#include "bagwiz/core/pipeline/topic_router.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

namespace pipeline = bagwiz::core::pipeline;
namespace io = bagwiz::io;
namespace fs = std::filesystem;

io::TopicInfo make_topic(std::string name, std::string type)
{
  io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

io::CreateOptions mcap_dir_opts()
{
  io::CreateOptions opts;
  opts.format = io::Format::Mcap;
  opts.layout = io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// Build an MCAP bag of `count` messages alternating across /foo and /bar, each
// carrying a `bytes`-long payload. `bytes` lets a test push payloads larger than
// a deliberately tiny queue cap to exercise the backpressure / oversized-admit
// paths.
fs::path build_input(const fs::path & dir, int count, std::size_t bytes)
{
  const auto path = dir / "input";
  auto writer = io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/foo", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/bar", "std_msgs/msg/Int32"));
  const std::vector<std::byte> payload(bytes, std::byte{0xAB});
  for (int i = 0; i < count; ++i) {
    const char * topic = (i % 2 == 0) ? "/foo" : "/bar";
    writer->write(
      topic, static_cast<std::int64_t>(i + 1) * 1'000'000'000LL,
      std::span<const std::byte>(payload.data(), payload.size()));
  }
  writer->close();
  return path;
}

using DeclareFn = std::function<void(io::BagWriter &, std::span<const io::TopicInfo>)>;

// Open `in`, declare the output topics via `declare`, run `proc` on `backend`,
// and finalize `out`. Returns the rewrite counts. The single place both the
// Sequential reference run and the Pipelined run flow through, so the only
// variable between them is the backend.
pipeline::RewriteCounts run_rewrite(
  pipeline::Backend & backend, const fs::path & in, const fs::path & out,
  const pipeline::Processor & proc, const DeclareFn & declare)
{
  auto reader = io::open_read(in);
  auto writer = io::open_write(out, mcap_dir_opts());
  declare(*writer, reader->topics());
  const auto counts = pipeline::run_pipeline(*reader, *writer, proc, backend, "");
  writer->close();
  return counts;
}

// Declare every input topic verbatim.
void declare_all(io::BagWriter & writer, std::span<const io::TopicInfo> topics)
{
  for (const auto & t : topics) {
    writer.declare_topic(t);
  }
}

// A BagWriter that throws on its Nth write(), to prove a writer-stage failure
// propagates out of PipelinedBackend::run() instead of hanging. Only the writer
// thread touches it, so the unsynchronized counter is safe.
class ThrowingWriter : public io::BagWriter
{
public:
  explicit ThrowingWriter(int throw_on_nth_write) : throw_on_(throw_on_nth_write) {}
  void declare_topic(const io::TopicInfo &) override {}
  void write(std::string_view, std::int64_t, std::span<const std::byte>) override
  {
    if (++writes_ >= throw_on_) {
      throw std::runtime_error("synthetic write failure");
    }
  }
  void close() override {}

private:
  int throw_on_;
  int writes_ = 0;
};

class PipelinedBackendTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = fs::temp_directory_path() /
               ("bagwiz_pipelined_backend_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                std::to_string(
                  reinterpret_cast<std::uintptr_t>(
                    this)));  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
  }

  fs::path tmp_dir_;
};

}  // namespace

// --- BackendKind override parsing -------------------------------------------

TEST(BackendSelectTest, ParseOverrideRecognizesNamesCaseInsensitively)
{
  EXPECT_FALSE(pipeline::parse_backend_override(nullptr).has_value());
  EXPECT_FALSE(pipeline::parse_backend_override("").has_value());
  EXPECT_FALSE(pipeline::parse_backend_override("garbage").has_value());
  EXPECT_EQ(pipeline::parse_backend_override("sequential"), pipeline::BackendKind::Sequential);
  EXPECT_EQ(pipeline::parse_backend_override("SEQ"), pipeline::BackendKind::Sequential);
  EXPECT_EQ(pipeline::parse_backend_override("Pipelined"), pipeline::BackendKind::Pipelined);
  EXPECT_EQ(pipeline::parse_backend_override("pipe"), pipeline::BackendKind::Pipelined);
}

// --- BoundedMessageQueue unit behavior --------------------------------------

pipeline::OwnedMessage make_owned(const std::string & topic, std::size_t bytes)
{
  pipeline::OwnedMessage msg;
  msg.out_topic = topic;
  msg.timestamp_ns = 1;
  msg.payload.assign(bytes, std::byte{0x7F});
  return msg;
}

TEST(BoundedMessageQueueTest, PreservesFifoOrderAndDrainsAfterClose)
{
  pipeline::BoundedMessageQueue queue;
  EXPECT_TRUE(queue.push(make_owned("/a", 1)));
  EXPECT_TRUE(queue.push(make_owned("/b", 1)));
  EXPECT_TRUE(queue.push(make_owned("/c", 1)));
  queue.close();

  pipeline::OwnedMessage out;
  ASSERT_TRUE(queue.pop(out));
  EXPECT_EQ(out.out_topic, "/a");
  ASSERT_TRUE(queue.pop(out));
  EXPECT_EQ(out.out_topic, "/b");
  ASSERT_TRUE(queue.pop(out));
  EXPECT_EQ(out.out_topic, "/c");
  EXPECT_FALSE(queue.pop(out));  // closed and drained
}

TEST(BoundedMessageQueueTest, AdmitsAMessageLargerThanTheCapWhenEmpty)
{
  pipeline::BoundedMessageQueue queue(4);            // 4-byte cap
  EXPECT_TRUE(queue.push(make_owned("/big", 100)));  // empty -> admitted despite cap
  pipeline::OwnedMessage out;
  ASSERT_TRUE(queue.pop(out));
  EXPECT_EQ(out.payload.size(), 100U);
}

// --- Differential: Pipelined output == Sequential output --------------------

TEST_F(PipelinedBackendTest, MatchesSequentialWhenSuppressing)
{
  const auto in = build_input(tmp_dir_, 3, 4);
  const auto seq_out = tmp_dir_ / "seq";
  const auto pipe_out = tmp_dir_ / "pipe";
  const std::unordered_set<std::string> suppress{"/foo"};
  pipeline::SuppressRouter router(suppress);
  const DeclareFn declare = [&](io::BagWriter & w, std::span<const io::TopicInfo> topics) {
    for (const auto & t : topics) {
      if (suppress.count(t.name) == 0) {
        w.declare_topic(t);
      }
    }
  };

  pipeline::SequentialBackend seq;
  pipeline::PipelinedBackend pipe;
  const auto seq_counts = run_rewrite(seq, in, seq_out, router, declare);
  const auto pipe_counts = run_rewrite(pipe, in, pipe_out, router, declare);

  bagwiz::test::expect_bags_equal(seq_out, pipe_out);
  EXPECT_EQ(seq_counts.copied, pipe_counts.copied);
  EXPECT_EQ(seq_counts.dropped, pipe_counts.dropped);
  EXPECT_EQ(seq_counts.renamed, pipe_counts.renamed);
  EXPECT_EQ(pipe_counts.copied, 1U);
  EXPECT_EQ(pipe_counts.dropped, 2U);
}

TEST_F(PipelinedBackendTest, MatchesSequentialWhenRenaming)
{
  const auto in = build_input(tmp_dir_, 3, 4);
  const auto seq_out = tmp_dir_ / "seq";
  const auto pipe_out = tmp_dir_ / "pipe";
  const std::unordered_map<std::string, std::string> rename{{"/foo", "/renamed"}};
  pipeline::RenameRouter router(rename);
  const DeclareFn declare = [&](io::BagWriter & w, std::span<const io::TopicInfo> topics) {
    for (const auto & t : topics) {
      if (t.name == "/foo") {
        auto renamed = t;
        renamed.name = "/renamed";
        w.declare_topic(renamed);
      } else {
        w.declare_topic(t);
      }
    }
  };

  pipeline::SequentialBackend seq;
  pipeline::PipelinedBackend pipe;
  const auto seq_counts = run_rewrite(seq, in, seq_out, router, declare);
  const auto pipe_counts = run_rewrite(pipe, in, pipe_out, router, declare);

  bagwiz::test::expect_bags_equal(seq_out, pipe_out);
  EXPECT_EQ(seq_counts.copied, pipe_counts.copied);
  EXPECT_EQ(seq_counts.renamed, pipe_counts.renamed);
  EXPECT_EQ(pipe_counts.copied, 3U);
  EXPECT_EQ(pipe_counts.renamed, 2U);
}

TEST_F(PipelinedBackendTest, FullPassthroughIsByteIdenticalToInput)
{
  const auto in = build_input(tmp_dir_, 5, 16);
  const auto pipe_out = tmp_dir_ / "pipe";
  const std::unordered_set<std::string> empty;
  pipeline::SuppressRouter router(empty);

  pipeline::PipelinedBackend pipe;
  const auto counts = run_rewrite(pipe, in, pipe_out, router, declare_all);

  EXPECT_EQ(counts.copied, 5U);
  EXPECT_EQ(counts.dropped, 0U);
  bagwiz::test::expect_bags_equal(in, pipe_out);
}

TEST_F(PipelinedBackendTest, HandlesEmptyBag)
{
  // A bag with declared topics but zero messages.
  const auto in = build_input(tmp_dir_, 0, 4);
  const auto pipe_out = tmp_dir_ / "pipe";
  const std::unordered_set<std::string> empty;
  pipeline::SuppressRouter router(empty);

  pipeline::PipelinedBackend pipe;
  const auto counts = run_rewrite(pipe, in, pipe_out, router, declare_all);

  EXPECT_EQ(counts.copied, 0U);
  EXPECT_EQ(counts.dropped, 0U);
  bagwiz::test::expect_bags_equal(in, pipe_out);
}

// Payloads (64 B) far exceed the queue cap (16 B), so every message is admitted
// one at a time and the reader blocks until the writer drains it: this is the
// backpressure path, and it must still produce Sequential-identical output.
TEST_F(PipelinedBackendTest, BackpressureWithTinyQueueStaysCorrect)
{
  const auto in = build_input(tmp_dir_, 8, 64);
  const auto seq_out = tmp_dir_ / "seq";
  const auto pipe_out = tmp_dir_ / "pipe";
  const std::unordered_set<std::string> empty;
  pipeline::SuppressRouter router(empty);

  pipeline::SequentialBackend seq;
  pipeline::PipelinedBackend pipe(16);  // 16-byte cap << 64-byte payloads
  const auto seq_counts = run_rewrite(seq, in, seq_out, router, declare_all);
  const auto pipe_counts = run_rewrite(pipe, in, pipe_out, router, declare_all);

  bagwiz::test::expect_bags_equal(seq_out, pipe_out);
  EXPECT_EQ(pipe_counts.copied, 8U);
  EXPECT_EQ(seq_counts.copied, pipe_counts.copied);
}

// A writer that fails mid-run must surface its exception from run() and never
// deadlock the reader (EXPECT_THROW returning at all proves no hang).
TEST_F(PipelinedBackendTest, WriterErrorPropagatesWithoutHanging)
{
  const auto in = build_input(tmp_dir_, 4, 8);
  auto reader = io::open_read(in);
  ThrowingWriter writer(2);  // throw on the 2nd write
  const std::unordered_set<std::string> empty;
  pipeline::SuppressRouter router(empty);
  pipeline::PipelinedBackend pipe;

  EXPECT_THROW(pipeline::run_pipeline(*reader, writer, router, pipe, ""), std::runtime_error);
}

namespace
{

// Appends a byte to every "/foo" message (kWrite) and forwards others verbatim
// (kPassthrough): exercises the seam's transform path with no geo dependency.
class AppendByteProcessor : public pipeline::Processor
{
public:
  [[nodiscard]] pipeline::Emit route(const std::string & in_topic) const override
  {
    return pipeline::Emit{true, in_topic};
  }
  [[nodiscard]] bool transforms() const override { return true; }
  [[nodiscard]] pipeline::TransformAction transform(
    const std::string & in_topic, std::span<const std::byte> in,
    std::vector<std::byte> & out) const override
  {
    if (in_topic == "/foo") {
      out.assign(in.begin(), in.end());
      out.push_back(std::byte{0xFF});
      return pipeline::TransformAction::kWrite;
    }
    return pipeline::TransformAction::kPassthrough;
  }
};

// Skips every "/bar" message (kSkip), forwards others verbatim (kPassthrough).
class SkipBarProcessor : public pipeline::Processor
{
public:
  [[nodiscard]] pipeline::Emit route(const std::string & in_topic) const override
  {
    return pipeline::Emit{true, in_topic};
  }
  [[nodiscard]] bool transforms() const override { return true; }
  [[nodiscard]] pipeline::TransformAction transform(
    const std::string & in_topic, std::span<const std::byte> /*in*/,
    std::vector<std::byte> & /*out*/) const override
  {
    return in_topic == "/bar" ? pipeline::TransformAction::kSkip
                              : pipeline::TransformAction::kPassthrough;
  }
};

}  // namespace

TEST_F(PipelinedBackendTest, TransformWriteAndPassthroughMatchSequential)
{
  const auto in = build_input(tmp_dir_, 3, 4);  // /foo, /bar, /foo
  const auto seq_out = tmp_dir_ / "seq";
  const auto pipe_out = tmp_dir_ / "pipe";
  AppendByteProcessor proc;

  pipeline::SequentialBackend seq;
  pipeline::PipelinedBackend pipe;
  const auto seq_counts = run_rewrite(seq, in, seq_out, proc, declare_all);
  const auto pipe_counts = run_rewrite(pipe, in, pipe_out, proc, declare_all);

  bagwiz::test::expect_bags_equal(seq_out, pipe_out);
  EXPECT_EQ(seq_counts.copied, pipe_counts.copied);
  EXPECT_EQ(seq_counts.transformed, pipe_counts.transformed);
  EXPECT_EQ(pipe_counts.copied, 3U);
  EXPECT_EQ(pipe_counts.transformed, 2U);  // two /foo messages rewritten
  EXPECT_EQ(pipe_counts.skipped, 0U);
}

TEST_F(PipelinedBackendTest, TransformSkipMatchesSequential)
{
  const auto in = build_input(tmp_dir_, 3, 8);  // /foo, /bar, /foo
  const auto seq_out = tmp_dir_ / "seq";
  const auto pipe_out = tmp_dir_ / "pipe";
  SkipBarProcessor proc;

  pipeline::SequentialBackend seq;
  pipeline::PipelinedBackend pipe;
  const auto seq_counts = run_rewrite(seq, in, seq_out, proc, declare_all);
  const auto pipe_counts = run_rewrite(pipe, in, pipe_out, proc, declare_all);

  bagwiz::test::expect_bags_equal(seq_out, pipe_out);
  EXPECT_EQ(seq_counts.copied, pipe_counts.copied);
  EXPECT_EQ(seq_counts.skipped, pipe_counts.skipped);
  EXPECT_EQ(pipe_counts.copied, 2U);   // two /foo forwarded
  EXPECT_EQ(pipe_counts.skipped, 1U);  // one /bar skipped
}
