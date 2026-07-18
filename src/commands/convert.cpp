// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/convert_msg_geo.hpp"
#include "bagwiz/core/bag_copy.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/msg_convert/geo_pose_convert.hpp"
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.convert";

// Resolve the target storage backend for a write. Precedence (first match
// wins):
//   1. Explicit `--storage <mcap|sqlite3>` from the CLI.
//   2. Output path's extension (`.mcap` → mcap, `.db3` → sqlite3) — only
//      applies to single-file outputs that carry one of those extensions.
//   3. Input bag's detected storage backend. This is the fallback for
//      directory-layout outputs (no extension to infer from) when the user
//      did not pass `--storage`: a pure layout change should not require
//      restating the storage backend.
// If none of the three resolves, `error_out` is set and `Format::Auto` is
// returned. `storage_flag` is the CLI string value (empty when the user did
// not pass `--storage`); `input_format` is the input's detected storage
// (`Format::Auto` when detection failed). The returned format is never
// `Format::Auto` on success.
io::Format resolve_target_storage(
  const std::string & storage_flag, const std::filesystem::path & output_path,
  io::Format input_format, std::string & error_out)
{
  if (!storage_flag.empty()) {
    return (storage_flag == "sqlite3") ? io::Format::Sqlite3 : io::Format::Mcap;
  }
  const auto inferred = io::infer_format_from_extension(output_path);
  if (inferred != io::Format::Auto) {
    return inferred;
  }
  if (input_format != io::Format::Auto) {
    return input_format;
  }
  error_out = "cannot determine target storage for output '" + output_path.string() +
              "': pass --storage <mcap|sqlite3>, use an output extension (.mcap or .db3), "
              "or supply an input whose storage backend can be auto-detected";
  return io::Format::Auto;
}

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Ships `format` (ROS 2 mcap <-> sqlite3 repack, plus file <-> directory
// layout transitions inferred from the output path) and `msg geo` (re-type
// NavSatFix topics into a geometry_msgs pose type).
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
    configure_format(app);
    configure_msg(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kFormat:
        return run_format();
      case Subcommand::kMsgGeo:
        return run_convert_msg_geo(msg_geo_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kFormat, kMsgGeo };
  Subcommand selected_ = Subcommand::kNone;

  struct FormatArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;     // empty when --storage not passed; resolved at run time
    bool overwrite = false;  // replace any pre-existing output_path
  } format_args_;

  ConvertMsgGeoArgs msg_geo_args_;

  void configure_format(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "format",
      "Repack a ROS 2 rosbag, converting between storage backends and/or "
      "file/directory layouts");
    sub->add_option("input", format_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "output", format_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub
      ->add_option(
        "-s,--storage", format_args_.storage,
        "Target storage backend (default: inferred from the output extension when it "
        "is .mcap or .db3; otherwise the input bag's storage backend is reused)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    sub->add_flag(
      "-w,--overwrite", format_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kFormat; });
  }

  // `msg` is a command group, not a leaf: its actions live under families
  // such as `msg geo` (position-related type conversions). Modeling it as a
  // group keeps room for further families (e.g. imu) without one flat command
  // accreting every type's options.
  void configure_msg(CLI::App & app)
  {
    auto * group = app.add_subcommand("msg", "Convert the message type of selected topics");
    group->require_subcommand(1);
    configure_msg_geo(*group);
  }

  void configure_msg_geo(CLI::App & group)
  {
    namespace mtc = bagwiz::core::msg_convert;
    auto * sub = group.add_subcommand(
      "geo",
      "Convert a geographic source (sensor_msgs/msg/NavSatFix) into a geometry_msgs pose type, "
      "projecting WGS84 lat/lon/alt into a Cartesian frame (ENU or UTM)");
    sub->add_option("input", msg_geo_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "--src", msg_geo_args_.src,
        "Source message type (snake_case). Required unless --topic is given; ignored when it is.")
      ->check(CLI::IsMember(mtc::from_snake_choices()));
    sub->add_option("--dst", msg_geo_args_.dst, "Target message type (snake_case). Required.")
      ->check(CLI::IsMember(mtc::to_snake_choices()));
    sub->add_option(
      "--topic", msg_geo_args_.topics,
      "Convert exactly these topic(s) instead of every topic matching --src. All named topics "
      "must share one message type.");
    sub
      ->add_option(
        "--crs", msg_geo_args_.crs,
        "Target Cartesian coordinate system: 'enu' (local tangent plane, needs an origin) or "
        "'utm' (easting/northing). Defaults to 'enu'.")
      ->check(CLI::IsMember({"enu", "utm"}))
      ->capture_default_str();
    sub->add_option(
      "--origin", msg_geo_args_.origin,
      "WGS84 datum as <lat>,<lon>,<alt>. Required for ENU unless it can be derived from the "
      "first NavSatFix; an optional offset for UTM.");
    sub->add_option(
      "--frame-id", msg_geo_args_.frame_id,
      "frame_id written onto the converted messages. Defaults to 'map' (enu) or 'utm' (utm).");
    sub->add_option(
      "-o,--output", msg_geo_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", msg_geo_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kMsgGeo; });
  }

  int run_format()
  {
    const auto & args = format_args_;

    // Detect the input's storage backend up-front so it can (a) feed
    // resolve_target_storage as the fallback for directory-layout outputs
    // without --storage, and (b) anchor the same-storage repack check
    // below. Magic-byte / metadata.yaml based — never extension based —
    // so renamed inputs still classify correctly.
    const auto source_format = io::detect_format(args.input_path);

    std::string err;
    const io::Format target_format =
      resolve_target_storage(args.storage, args.output_path, source_format, err);
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

    // Reject same-storage + same-layout repacks: a plain `cp` is what the
    // user actually wants. When the layouts differ (e.g. file → directory
    // on the same backend) the run is allowed since the output shape
    // genuinely changes. Input layout is read from the filesystem (input
    // is guaranteed to exist by CLI::ExistingPath); output layout is read
    // from the path's extension (no `.mcap`/`.db3` → directory layout).
    if (source_format == target_format) {
      std::error_code ec;
      const bool input_is_directory = std::filesystem::is_directory(args.input_path, ec);
      const bool output_is_directory =
        io::infer_format_from_extension(args.output_path) == io::Format::Auto;
      if (input_is_directory == output_is_directory) {
        const char * fmt_name = (target_format == io::Format::Sqlite3) ? "sqlite3" : "mcap";
        BAGWIZ_LOG_ERROR(
          kLogger,
          "input is already in '%s' storage with the same layout; nothing to convert "
          "(use `cp -r` for a verbatim copy)",
          fmt_name);
        return 1;
      }
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

    // convert is a pure passthrough copy (only the storage format changes), so it
    // runs through the shared rewrite seam with an empty suppress set on the
    // threaded backend. A read/write error now aborts the run (fail-fast) instead
    // of silently skipping messages, which could mask partial output corruption.
    core::BagCopyCounts counts;
    try {
      const std::unordered_set<std::string> none;
      counts = core::bag_copy_filtered(
        *reader, *writer, none, "convert", core::pipeline::BackendKind::Pipelined);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "convert read/write failed: %s", e.what());
      return 1;
    }

    try {
      writer->close();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "writer->close failed: %s", e.what());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Repack done: %" PRIu64 " message(s) written across %zu topic(s)", counts.copied,
      declared);

    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(ConvertCommand)

}  // namespace bagwiz::commands
