// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.convert";

// Resolve the target storage backend for a write. The CLI takes `--storage`
// optionally; when omitted, we fall back to inferring from the output path's
// extension (`.mcap` / `.db3`). If neither is conclusive — typically a
// directory output without an explicit flag — we surface a clear error
// instead of silently picking a default, since the user has not actually
// chosen one. `storage_flag` is the CLI string value (empty when the user
// did not pass `--storage`); the returned format is never `Format::Auto`.
io::Format resolve_target_storage(
  const std::string & storage_flag, const std::filesystem::path & output_path,
  std::string & error_out)
{
  if (!storage_flag.empty()) {
    return (storage_flag == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
  }
  const auto inferred = io::infer_format_from_extension(output_path);
  if (inferred != io::Format::Auto) {
    return inferred;
  }
  error_out = "cannot determine target storage from output path '" + output_path.string() +
              "'; pass --storage <mcap|sqlite3> or use a .mcap/.db3 extension";
  return io::Format::Auto;
}

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Ships `storage` (ROS 2 mcap <-> sqlite3 repack).
class ConvertCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "convert"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Convert between bag formats";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_storage(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kStorage:
        return run_storage();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kStorage };
  Subcommand selected_ = Subcommand::kNone;

  struct StorageArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;     // empty when --storage not passed; resolved at run time
    bool overwrite = false;  // replace any pre-existing output_path
  } storage_args_;

  // Inspect metadata.yaml of a directory-layout input to detect rosbag2's
  // generic compression layer (which we don't decompress). Single-file
  // inputs have no metadata.yaml; mcap chunk-level compression there is
  // handled transparently by libmcap.
  static int check_input_compression(const std::filesystem::path & input)
  {
    std::error_code ec;
    if (!std::filesystem::is_directory(input, ec)) {
      return 0;
    }
    const auto metadata_path = input / "metadata.yaml";
    if (!std::filesystem::exists(metadata_path)) {
      return 0;
    }
    io::BagMetadata md;
    try {
      md = io::load_metadata_yaml(metadata_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "Could not parse metadata.yaml (%s); proceeding without compression check",
        e.what());
      return 0;
    }
    // rosbag2 emits "NONE" or omits the field for non-compressed bags;
    // anything else is a hard fail.
    if (!md.compression_mode.empty() && md.compression_mode != "NONE") {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "input bag uses rosbag2-layer compression (compression_mode='%s', format='%s'); "
        "decompress with `ros2 bag convert` first",
        md.compression_mode.c_str(), md.compression_format.c_str());
      return 1;
    }
    return 0;
  }

  void configure_storage(CLI::App & app)
  {
    auto * sub =
      app.add_subcommand("storage", "Repack a ROS 2 rosbag into a different storage backend");
    sub->add_option("input", storage_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "output", storage_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub
      ->add_option(
        "-s,--storage", storage_args_.storage,
        "Target storage backend (default: inferred from output extension)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->add_flag(
      "--overwrite", storage_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->footer(
      "Messages are copied verbatim — only the storage backend changes; no\n"
      "deserialization or type conversion is performed.\n"
      "If --storage is omitted, the backend is inferred from the output path's\n"
      "extension (.mcap or .db3); other paths (e.g. a directory) require --storage.\n"
      "Inputs that use rosbag2-layer compression (compression_mode != NONE)\n"
      "are rejected; decompress with `ros2 bag convert` first.");
    sub->callback([this]() { selected_ = Subcommand::kStorage; });
  }

  int run_storage()
  {
    const auto & args = storage_args_;

    if (const int rc = check_input_compression(args.input_path); rc != 0) {
      return rc;
    }

    std::string err;
    const io::Format target_format = resolve_target_storage(args.storage, args.output_path, err);
    if (target_format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }

    if (const auto r = core::prepare_output_path(args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // Reject same-storage repack: it's almost always a user mistake (and a
    // plain copy is what they actually want). Detection is by magic bytes
    // (single-file inputs) or metadata.yaml (directory layouts) — never
    // by extension — so renamed files are still classified correctly.
    const auto source_format = io::detect_format(args.input_path);
    if (source_format == target_format) {
      const char * fmt_name = (target_format == io::Format::Sqlite3) ? "sqlite3" : "mcap";
      BAGWIZ_LOG_ERROR(kLogger, "input is already in '%s' storage; nothing to convert", fmt_name);
      return 1;
    }

    io::CreateOptions copts;
    copts.format = target_format;
    copts.layout = io::Layout::Auto;  // factory picks SingleFile if extension matches
    // Leave compression off so the output is predictable; callers can
    // recompress with `ros2 bag convert` if they want.
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    // Force schema bytes onto the topic list before declaring so MCAP
    // outputs preserve self-description across a repack (one-shot shard
    // open for multi-shard MCAP inputs; no-op for single-file MCAP and
    // SQLite3 where schemas are either already loaded or not embedded).
    reader->populate_schemas();

    // SQLite3 storage in Humble (and earlier) does not embed message
    // definitions, so reader->topics() comes back with empty
    // schema_text. Resolve each missing definition from
    // $AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg before declaring
    // the topic — otherwise the resulting MCAP loses self-description
    // and breaks strict downstream readers like rosbags-convert.
    std::size_t declared = 0;
    std::size_t resolved_defs = 0;
    std::size_t unresolved_defs = 0;
    for (const auto & t : reader->topics()) {
      io::TopicInfo augmented = t;
      if (augmented.schema_text.empty()) {
        auto resolved = core::resolve_message_definition(augmented.type);
        if (!resolved.text.empty()) {
          augmented.schema_text = std::move(resolved.text);
          augmented.schema_encoding = std::move(resolved.encoding);
          ++resolved_defs;
        } else {
          ++unresolved_defs;
          if (unresolved_defs <= 5) {
            BAGWIZ_LOG_WARN(
              kLogger,
              "no .msg on disk for type '%s' (topic '%s'); writing MCAP without "
              "self-description for this topic",
              augmented.type.c_str(), augmented.name.c_str());
          }
        }
      }
      try {
        writer->declare_topic(augmented);
        ++declared;
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(
          kLogger, "declare_topic failed for '%s': %s; skipping topic", t.name.c_str(), e.what());
      }
    }
    if (resolved_defs > 0) {
      BAGWIZ_LOG_INFO(
        kLogger, "resolved %zu missing message definition(s) from $AMENT_PREFIX_PATH",
        resolved_defs);
    }
    if (unresolved_defs > 5) {
      BAGWIZ_LOG_WARN(
        kLogger, "(plus %zu more topic(s) without resolvable .msg)", unresolved_defs - 5);
    }

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    uint64_t total_failed = 0;
    io::RawMessage msg;
    while (true) {
      try {
        if (!reader->next(msg)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "ros2 read error: %s", e.what());
        return 1;
      }
      ++total_in;

      if (msg.topic == nullptr) {
        continue;
      }

      try {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
        ++total_out;
      } catch (const std::exception & e) {
        ++total_failed;
        if (total_failed <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", msg.topic->name.c_str(),
            e.what());
        }
      }
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Repack done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, declared);
    if (total_failed > 0) {
      BAGWIZ_LOG_WARN(kLogger, "%" PRIu64 " message(s) failed to write", total_failed);
    }

    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(ConvertCommand)

}  // namespace bagwiz::commands
