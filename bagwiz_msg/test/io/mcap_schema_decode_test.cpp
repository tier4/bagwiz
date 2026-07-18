// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// End-to-end tests for the MCAP schema-driven decode pipeline.
//
// These exercise the full writer → reader → decoder → formatter flow
// against real .mcap files: write a bag with an embedded ros2msg
// schema, reopen it via bagwiz::io::open_read, hand the TopicInfo to
// core::decoder::open_decoder, and decode the message bytes.
//
// The unit tests in test/core/decoder/decoder_test.cpp construct
// TopicInfo manually and skip the IO layer — they prove the decoder
// works in isolation. These tests prove the IO layer plumbs
// schema_text correctly into the factory's selection logic, so the
// MCAP writer's embedded schema text round-trips into the right
// decoder backend at read time.

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/decoder/message_view.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace dec = bagwiz::core::decoder;
namespace cdr = bagwiz::core::cdr_walker;

namespace
{

// --- Test fixtures ----------------------------------------------------

// Build a CDR-LE payload for a `string data` schema with the given value.
// Layout: 4-byte encapsulation header + uint32 length (incl NUL) + bytes
// + NUL terminator. Hand-crafted so the test does not depend on a CDR
// writer the production code does not currently expose.
std::vector<std::byte> string_payload(const std::string & value)
{
  std::vector<std::byte> out;
  const auto push = [&out](std::uint8_t b) { out.push_back(std::byte{b}); };
  push(0x00);
  push(0x01);  // CDR_LE
  push(0x00);
  push(0x00);
  const std::uint32_t length = static_cast<std::uint32_t>(value.size()) + 1;
  push(static_cast<std::uint8_t>(length & 0xFFU));
  push(static_cast<std::uint8_t>((length >> 8) & 0xFFU));
  push(static_cast<std::uint8_t>((length >> 16) & 0xFFU));
  push(static_cast<std::uint8_t>((length >> 24) & 0xFFU));
  for (const char c : value) {
    push(static_cast<std::uint8_t>(c));
  }
  push(0x00);
  return out;
}

class McapSchemaDecodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_mcap_schema_decode_" +
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

// --- Helpers ----------------------------------------------------------

// Find a topic by name in the reader's topic list. Returns nullptr if
// not found; tests should ASSERT_NE to fail fast.
const bagwiz::io::TopicInfo * find_topic(
  const bagwiz::io::BagReader & reader, const std::string & name)
{
  for (const auto & t : reader.topics()) {
    if (t.name == name) {
      return &t;
    }
  }
  return nullptr;
}

// Write `topic` + one message into `path` via bagwiz's own writer.
// Returns the resulting path so tests can chain.
std::filesystem::path write_single_message_bag(
  const std::filesystem::path & path, const bagwiz::io::TopicInfo & topic,
  std::span<const std::byte> payload)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::SingleFile;
  opts.mcap_compression = "none";

  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(topic);
  writer->write(topic.name, 1'000'000'000LL, payload);
  writer->close();
  return path;
}

}  // namespace

// --- Tests ------------------------------------------------------------

TEST_F(McapSchemaDecodeTest, BagwizWrittenMcapDecodesViaSchemaPath)
{
  // The MCAP writer embeds the schema in the Schema record; the
  // decoder factory picks the schema-driven backend when schema_text
  // is non-empty. End-to-end proof that a bagwiz-written MCAP carries
  // enough information to decode its own messages without going
  // through introspection.
  //
  // We assert backend == "schema" specifically so a regression that
  // silently routed everything to introspection (e.g. an accidental
  // BAGWIZ_DECODER override leaking from another test) would fail
  // here rather than passing with worse-but-functional behaviour.
  bagwiz::io::TopicInfo topic;
  topic.name = "/foo";
  topic.type = "std_msgs/msg/String";
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = "string data\n";

  const auto payload = string_payload("hello world");
  const auto path = write_single_message_bag(tmp_dir_ / "out.mcap", topic, payload);

  auto reader = bagwiz::io::open_read(path);
  const auto * t = find_topic(*reader, "/foo");
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->schema_text, "string data\n");
  EXPECT_EQ(t->schema_encoding, "ros2msg");

  auto open = dec::open_decoder(*t);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "schema");

  bagwiz::io::RawMessage msg;
  ASSERT_TRUE(reader->next(msg));
  const auto decoded = open.decoder->decode(msg.payload);
  ASSERT_TRUE(decoded.ok()) << decoded.error;

  const auto * obj = std::get_if<cdr::Object>(&decoded.value->v);
  ASSERT_NE(obj, nullptr);
  dec::MessageView view{*obj};
  EXPECT_EQ(view.primitive<std::string>("data").value(), "hello world");
}

TEST_F(McapSchemaDecodeTest, CustomTypeWithoutTypesupportStillDecodes)
{
  // The central payoff of the refactor: a message type whose
  // typesupport `.so` is not installed anywhere on AMENT_PREFIX_PATH
  // still decodes from MCAP, because the schema text travels with the
  // bag and the schema-driven decoder doesn't dlopen anything.
  //
  // Use a deliberately-fake package name so introspection would fail
  // with a clear dlopen error. We then prove the schema path takes
  // over and produces correct output.
  bagwiz::io::TopicInfo topic;
  topic.name = "/custom";
  topic.type = "bagwiz_fake_pkg/msg/Greeting";
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = "string data\n";

  const auto payload = string_payload("custom-type works without source");
  const auto path = write_single_message_bag(tmp_dir_ / "custom.mcap", topic, payload);

  auto reader = bagwiz::io::open_read(path);
  const auto * t = find_topic(*reader, "/custom");
  ASSERT_NE(t, nullptr);

  auto open = dec::open_decoder(*t);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "schema") << "introspection fallback would fail";

  bagwiz::io::RawMessage msg;
  ASSERT_TRUE(reader->next(msg));
  const auto decoded = open.decoder->decode(msg.payload);
  ASSERT_TRUE(decoded.ok()) << decoded.error;

  const auto * obj = std::get_if<cdr::Object>(&decoded.value->v);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(std::get<std::string>(obj->fields[0].second.v), "custom-type works without source");
}

TEST_F(McapSchemaDecodeTest, DirectoryLayoutLazyPopulatesSchemasForDecoding)
{
  // Multi-shard / directory-layout bags don't put schemas in
  // metadata.yaml — they are stored only in the shard files
  // themselves. topics() before populate_schemas() returns empty
  // schema_text, so a naive
  // factory call would fall through to introspection. The shard
  // reader's opportunistic backfill (populate_schemas + the
  // backfill-on-next() path) must restore the schema before
  // open_decoder runs, otherwise the fake-type test above would be
  // silently bypassed in the directory case.
  bagwiz::io::TopicInfo topic;
  topic.name = "/dir_topic";
  topic.type = "bagwiz_fake_pkg/msg/Greeting";
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = "string data\n";

  const auto dir = tmp_dir_ / "dir_bag";
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  auto writer = bagwiz::io::open_write(dir, opts);
  writer->declare_topic(topic);
  writer->write(topic.name, 1'000'000'000LL, string_payload("from a directory bag"));
  writer->close();
  ASSERT_TRUE(std::filesystem::exists(dir / "metadata.yaml"));

  auto reader = bagwiz::io::open_read(dir);
  // First topics() call: metadata-only → schema fields are empty.
  const auto * pre = find_topic(*reader, "/dir_topic");
  ASSERT_NE(pre, nullptr);
  EXPECT_TRUE(pre->schema_text.empty()) << "metadata.yaml leaked schemas it shouldn't";

  // Lazy populate: opens shard 0 and backfills onto our owned topics.
  reader->populate_schemas();
  const auto * after = find_topic(*reader, "/dir_topic");
  ASSERT_NE(after, nullptr);
  EXPECT_EQ(after->schema_text, "string data\n");

  // Now the factory has enough info to pick the schema backend.
  auto open = dec::open_decoder(*after);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "schema");

  bagwiz::io::RawMessage msg;
  ASSERT_TRUE(reader->next(msg));
  const auto decoded = open.decoder->decode(msg.payload);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  const auto * obj = std::get_if<cdr::Object>(&decoded.value->v);
  ASSERT_NE(obj, nullptr);
  EXPECT_EQ(std::get<std::string>(obj->fields[0].second.v), "from a directory bag");
}

TEST_F(McapSchemaDecodeTest, EmptySchemaForcesIntrospectionFallback)
{
  // A bagwiz-written MCAP whose declare_topic() was given an empty
  // schema_text matches the legacy output from before MCAP self-
  // description was supported. The factory must transparently fall
  // back to introspection so the bag is still readable. We use
  // std_msgs/msg/String so introspection resolves (the package is on
  // the test-time AMENT_PREFIX_PATH).
  bagwiz::io::TopicInfo topic;
  topic.name = "/legacy";
  topic.type = "std_msgs/msg/String";
  topic.serialization_format = "cdr";
  // schema_encoding / schema_text intentionally left empty.

  const auto payload = string_payload("legacy bag");
  const auto path = write_single_message_bag(tmp_dir_ / "legacy.mcap", topic, payload);

  auto reader = bagwiz::io::open_read(path);
  const auto * t = find_topic(*reader, "/legacy");
  ASSERT_NE(t, nullptr);
  EXPECT_TRUE(t->schema_text.empty());

  auto open = dec::open_decoder(*t);
  ASSERT_TRUE(open.ok()) << open.error;
  EXPECT_EQ(open.decoder->backend(), "introspection");

  bagwiz::io::RawMessage msg;
  ASSERT_TRUE(reader->next(msg));
  const auto decoded = open.decoder->decode(msg.payload);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  const auto * obj = std::get_if<cdr::Object>(&decoded.value->v);
  ASSERT_NE(obj, nullptr);
  dec::MessageView view{*obj};
  EXPECT_EQ(view.primitive<std::string>("data").value(), "legacy bag");
}

TEST_F(McapSchemaDecodeTest, CrossBackendEquivalenceOnRealMcap)
{
  // Round-trip the same payload twice through the same MCAP fixture
  // but force the introspection backend on the second pass via
  // BAGWIZ_DECODER. The two decoded Value trees must be structurally
  // equal — the YAML formatter relies on this contract to produce
  // byte-identical output regardless of which backend decoded the
  // message.
  bagwiz::io::TopicInfo topic;
  topic.name = "/eq";
  topic.type = "std_msgs/msg/String";
  topic.serialization_format = "cdr";
  topic.schema_encoding = "ros2msg";
  topic.schema_text = "string data\n";

  const auto payload = string_payload("equivalence test");
  const auto path = write_single_message_bag(tmp_dir_ / "eq.mcap", topic, payload);

  // Schema path.
  ::unsetenv("BAGWIZ_DECODER");
  auto reader_a = bagwiz::io::open_read(path);
  const auto * t_a = find_topic(*reader_a, "/eq");
  ASSERT_NE(t_a, nullptr);
  auto open_a = dec::open_decoder(*t_a);
  ASSERT_TRUE(open_a.ok());
  ASSERT_EQ(open_a.decoder->backend(), "schema");
  bagwiz::io::RawMessage msg_a;
  ASSERT_TRUE(reader_a->next(msg_a));
  const auto decoded_a = open_a.decoder->decode(msg_a.payload);
  ASSERT_TRUE(decoded_a.ok()) << decoded_a.error;

  // Introspection path.
  ::setenv("BAGWIZ_DECODER", "introspection", 1);
  auto reader_b = bagwiz::io::open_read(path);
  const auto * t_b = find_topic(*reader_b, "/eq");
  ASSERT_NE(t_b, nullptr);
  auto open_b = dec::open_decoder(*t_b);
  ASSERT_TRUE(open_b.ok());
  ASSERT_EQ(open_b.decoder->backend(), "introspection");
  bagwiz::io::RawMessage msg_b;
  ASSERT_TRUE(reader_b->next(msg_b));
  const auto decoded_b = open_b.decoder->decode(msg_b.payload);
  ASSERT_TRUE(decoded_b.ok()) << decoded_b.error;
  ::unsetenv("BAGWIZ_DECODER");

  const auto * obj_a = std::get_if<cdr::Object>(&decoded_a.value->v);
  const auto * obj_b = std::get_if<cdr::Object>(&decoded_b.value->v);
  ASSERT_NE(obj_a, nullptr);
  ASSERT_NE(obj_b, nullptr);
  ASSERT_EQ(obj_a->fields.size(), obj_b->fields.size());
  for (std::size_t i = 0; i < obj_a->fields.size(); ++i) {
    EXPECT_EQ(obj_a->fields[i].first, obj_b->fields[i].first);
    EXPECT_EQ(
      std::get<std::string>(obj_a->fields[i].second.v),
      std::get<std::string>(obj_b->fields[i].second.v));
  }
}
