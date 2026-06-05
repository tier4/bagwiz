// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/convert_msgtype_geo.hpp"

#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/msgtype_convert/geo_pose_convert.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace mtc = bagwiz::core::msgtype_convert;

constexpr const char * kLogger = "bagwiz.cmd.convert.msgtype.geo";

// Parse a "lat,lon,alt" string into a GeoOrigin. Returns false (with `err`
// populated) on a malformed value.
bool parse_origin(const std::string & text, mtc::GeoOrigin & out, std::string & err)
{
  std::array<double, 3> values{};
  std::size_t field = 0;
  std::size_t start = 0;
  while (field < 3) {
    const std::size_t comma = text.find(',', start);
    const std::string token =
      text.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    try {
      std::size_t consumed = 0;
      values[field] = std::stod(token, &consumed);
      if (token.empty() || consumed != token.size()) {
        throw std::invalid_argument("trailing characters");
      }
    } catch (const std::exception &) {
      err = "could not parse --origin '" + text + "': expected <lat>,<lon>,<alt> (three numbers)";
      return false;
    }
    ++field;
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  if (field != 3 || text.find(',', start) != std::string::npos) {
    err = "could not parse --origin '" + text + "': expected exactly <lat>,<lon>,<alt>";
    return false;
  }
  out = mtc::GeoOrigin{values[0], values[1], values[2]};
  return true;
}

// Default world-frame name for a target CRS (REP-105 'map' for ENU; the
// robot_localization convention 'utm' for UTM).
std::string default_frame_id(mtc::GeoCrs crs)
{
  return crs == mtc::GeoCrs::kEnu ? "map" : "utm";
}

// Comma-joined list for human-readable messages; "(none)" when empty.
std::string join_csv(const std::vector<std::string> & names)
{
  if (names.empty()) {
    return "(none)";
  }
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      out += ", ";
    }
    out += names[i];
  }
  return out;
}

// Resolve which topics to convert and the single source ROS type they share.
// Implements the two selection modes (explicit --topic vs by --src type).
// Returns false (after logging) on any selection error.
bool select_topics(
  const ConvertMsgtypeGeoArgs & args, std::span<const io::TopicInfo> topics,
  std::vector<std::string> & selected_out, std::string & source_ros_type_out)
{
  if (!args.topics.empty()) {
    // --topic given: --dst required (checked by caller), --src ignored. Every
    // named topic must exist and all must share one message type.
    std::unordered_set<std::string> seen;
    std::vector<std::string> requested;
    for (const auto & t : args.topics) {
      if (t.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "Every --topic argument must be a non-empty topic name.");
        return false;
      }
      if (seen.insert(t).second) {
        requested.push_back(t);
      }
    }

    std::vector<std::string> unknown;
    std::string common_type;
    for (const auto & name : requested) {
      const io::TopicInfo * match = nullptr;
      for (const auto & t : topics) {
        if (t.name == name) {
          match = &t;
          break;
        }
      }
      if (match == nullptr) {
        unknown.push_back(name);
        continue;
      }
      if (common_type.empty()) {
        common_type = match->type;
      } else if (common_type != match->type) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--topic selection mixes message types ('%s' and '%s'); all selected topics must share "
          "one type.",
          common_type.c_str(), match->type.c_str());
        return false;
      }
    }
    if (!unknown.empty()) {
      std::vector<std::string> available;
      available.reserve(topics.size());
      for (const auto & t : topics) {
        available.push_back(t.name);
      }
      BAGWIZ_LOG_ERROR(kLogger, "No such topic(s) in the bag: %s", join_csv(unknown).c_str());
      BAGWIZ_LOG_ERROR(kLogger, "Available topics: %s", join_csv(available).c_str());
      return false;
    }
    selected_out = std::move(requested);
    source_ros_type_out = common_type;
    return true;
  }

  // No --topic: --src required. Select every topic whose type matches --src.
  if (args.src.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "--src is required when --topic is not given.");
    return false;
  }
  const auto from_ros = mtc::from_snake_to_ros_type(args.src);
  if (!from_ros.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Unknown --src '%s'. Supported: %s", args.src.c_str(),
      join_csv(mtc::from_snake_choices()).c_str());
    return false;
  }
  for (const auto & t : topics) {
    if (t.type == *from_ros) {
      selected_out.push_back(t.name);
    }
  }
  if (selected_out.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "No topic of type '%s' (--src %s) found in the bag.", from_ros->c_str(),
      args.src.c_str());
    return false;
  }
  source_ros_type_out = *from_ros;
  return true;
}

// Scan the bag for the first decodable NavSatFix on any selected topic and use
// its lat/lon/alt as the ENU origin. Returns std::nullopt when no such message
// can be decoded.
std::optional<mtc::GeoOrigin> scan_first_origin(
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & selected)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception &) {
    return std::nullopt;
  }
  reader->populate_schemas();

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
  io::ReadFilter filter;
  for (const auto & t : reader->topics()) {
    if (selected.count(t.name) == 0) {
      continue;
    }
    auto open = core::decoder::open_decoder(t);
    if (open.ok()) {
      decoders.emplace(t.name, std::move(open.decoder));
      filter.topics.push_back(t.name);
    }
  }
  if (decoders.empty()) {
    return std::nullopt;
  }
  reader->set_filter(filter);

  io::RawMessage raw;
  while (reader->next(raw)) {
    if (raw.topic == nullptr) {
      continue;
    }
    auto it = decoders.find(raw.topic->name);
    if (it == decoders.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      continue;
    }
    if (const auto sample = mtc::extract_nav_sat_fix(*decoded.value)) {
      return mtc::GeoOrigin{sample->latitude, sample->longitude, sample->altitude};
    }
  }
  return std::nullopt;
}

// Build the TopicInfo used to declare a converted topic: same name/QoS, but the
// target type and (resolved) schema. The type-description hash is cleared since
// the message type changed.
io::TopicInfo make_target_topic_info(
  const io::TopicInfo & source, const std::string & target_type,
  const core::ResolvedMessageDefinition & target_def)
{
  io::TopicInfo info = source;
  info.type = target_type;
  info.serialization_format = "cdr";
  info.schema_encoding = target_def.encoding;
  info.schema_text = target_def.text;
  info.type_description_hash.clear();
  return info;
}

// One full conversion pass: read `input_path`, declare every topic (converted
// topics re-typed to `target_type`), then stream-copy — converting selected
// topics' messages and forwarding the rest verbatim. The writer factory is
// parameterised so the in-place path can hand in a tmp location.
int execute_pass(
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & selected,
  const std::string & target_type, const core::ResolvedMessageDefinition & target_def,
  const mtc::GeoPoseConverter & converter,
  const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());

  // One decoder per selected topic (all share the source type, validated up
  // front). A decoder that cannot be opened aborts the run — we cannot convert
  // a topic we cannot decode.
  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoders;
  for (const auto & t : topics) {
    if (selected.count(t.name) == 0) {
      continue;
    }
    auto open = core::decoder::open_decoder(t);
    if (!open.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not open decoder for topic '%s' (type '%s'): %s", t.name.c_str(),
        t.type.c_str(), open.error.c_str());
      return 1;
    }
    decoders.emplace(t.name, std::move(open.decoder));
  }

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
    return 1;
  }

  for (const auto & t : topics) {
    const io::TopicInfo declared =
      selected.count(t.name) != 0 ? make_target_topic_info(t, target_type, target_def) : t;
    try {
      writer->declare_topic(declared);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  std::uint64_t total_in = 0;
  std::uint64_t converted = 0;
  std::uint64_t copied = 0;
  std::uint64_t failed = 0;
  io::RawMessage msg;
  while (true) {
    try {
      if (!reader->next(msg)) {
        break;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
      return 1;
    }
    ++total_in;
    if (msg.topic == nullptr) {
      continue;
    }

    auto dec_it = decoders.find(msg.topic->name);
    if (dec_it == decoders.end()) {
      // Not a converted topic: forward verbatim.
      try {
        writer->write(msg.topic->name, msg.timestamp_ns, msg.payload);
        ++copied;
      } catch (const std::exception & e) {
        ++failed;
        if (failed <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", msg.topic->name.c_str(),
            e.what());
        }
      }
      continue;
    }

    const auto decoded = dec_it->second->decode(msg.payload);
    if (!decoded.ok()) {
      ++failed;
      if (failed <= 3) {
        BAGWIZ_LOG_WARN(
          kLogger, "decode failed on '%s': %s; skipping message", msg.topic->name.c_str(),
          decoded.error.c_str());
      }
      continue;
    }
    const auto sample = mtc::extract_nav_sat_fix(*decoded.value);
    if (!sample.has_value()) {
      ++failed;
      if (failed <= 3) {
        BAGWIZ_LOG_WARN(
          kLogger, "message on '%s' is not a decodable NavSatFix; skipping",
          msg.topic->name.c_str());
      }
      continue;
    }
    try {
      const auto payload = converter.convert(*sample);
      writer->write(
        msg.topic->name, msg.timestamp_ns,
        std::span<const std::byte>(payload.data(), payload.size()));
      ++converted;
    } catch (const std::exception & e) {
      ++failed;
      if (failed <= 3) {
        BAGWIZ_LOG_WARN(
          kLogger, "convert/write failed on '%s': %s; skipping message", msg.topic->name.c_str(),
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
    kLogger,
    "convert msgtype geo: converted %" PRIu64 " message(s) to %s, copied %" PRIu64
    " other message(s) (of %" PRIu64 " read).",
    converted, target_type.c_str(), copied, total_in);
  if (failed > 0) {
    BAGWIZ_LOG_WARN(kLogger, "%" PRIu64 " message(s) failed and were skipped.", failed);
  }
  return 0;
}

}  // namespace

int run_convert_msgtype_geo(const ConvertMsgtypeGeoArgs & args)
{
  // 1. CRS and target type (the CLI's IsMember checks make these reachable only
  //    with valid tokens, but validate defensively).
  mtc::GeoCrs crs = mtc::GeoCrs::kEnu;
  if (args.crs == "enu") {
    crs = mtc::GeoCrs::kEnu;
  } else if (args.crs == "utm") {
    crs = mtc::GeoCrs::kUtm;
  } else {
    BAGWIZ_LOG_ERROR(kLogger, "--crs must be 'enu' or 'utm' (got '%s').", args.crs.c_str());
    return 1;
  }

  if (args.dst.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "--dst is required.");
    return 1;
  }
  const auto to_ros = mtc::to_snake_to_ros_type(args.dst);
  if (!to_ros.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Unknown --dst '%s'. Supported: %s", args.dst.c_str(),
      join_csv(mtc::to_snake_choices()).c_str());
    return 1;
  }

  // 2. Inspect the bag's topics to drive selection.
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  reader->populate_schemas();

  std::vector<std::string> selected;
  std::string source_ros_type;
  if (!select_topics(args, reader->topics(), selected, source_ros_type)) {
    return 1;
  }

  // 3. Whitelist check on the (source, target) type pair.
  const auto route = mtc::resolve_route(source_ros_type, *to_ros);
  if (!route.has_value()) {
    const auto source_snake = mtc::ros_type_to_snake(source_ros_type);
    BAGWIZ_LOG_ERROR(
      kLogger, "Unsupported geo conversion: %s -> %s.",
      source_snake.value_or(source_ros_type).c_str(), args.dst.c_str());
    BAGWIZ_LOG_ERROR(
      kLogger, "Supported sources: %s; targets: %s.", join_csv(mtc::from_snake_choices()).c_str(),
      join_csv(mtc::to_snake_choices()).c_str());
    return 1;
  }

  // Release the inspection reader before opening read/write passes.
  reader.reset();

  // 4. Origin: explicit --origin wins; ENU otherwise falls back to the first
  //    NavSatFix in the bag (and logs it so the value can be reused later, e.g.
  //    for a future reverse conversion). UTM tolerates an absent origin.
  std::optional<mtc::GeoOrigin> origin;
  if (args.origin.has_value()) {
    mtc::GeoOrigin parsed;
    std::string err;
    if (!parse_origin(*args.origin, parsed, err)) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }
    origin = parsed;
  } else if (crs == mtc::GeoCrs::kEnu) {
    const std::unordered_set<std::string> selected_set(selected.begin(), selected.end());
    origin = scan_first_origin(args.input_path, selected_set);
    if (!origin.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "ENU conversion needs an origin but no decodable NavSatFix was found to derive one; "
        "pass --origin <lat>,<lon>,<alt>.");
      return 1;
    }
    BAGWIZ_LOG_INFO(
      kLogger,
      "Using ENU origin from first NavSatFix: %.9f, %.9f, %.3f (pass --origin to override).",
      origin->lat, origin->lon, origin->alt);
  }

  // 5. Frame id and target schema text.
  const std::string frame_id = args.frame_id.value_or(default_frame_id(crs));
  const auto target_def = core::resolve_message_definition(*to_ros);
  if (target_def.text.empty()) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "Could not resolve a .msg definition for '%s' from $AMENT_PREFIX_PATH; the output will lack "
      "self-description for the converted topic(s).",
      to_ros->c_str());
  }

  // 6. Build the converter.
  mtc::GeoConvertConfig cfg;
  cfg.crs = crs;
  cfg.origin = origin;
  cfg.frame_id = frame_id;
  cfg.target_ros_type = *to_ros;
  cfg.target_has_covariance = route->target_has_covariance;
  std::unique_ptr<mtc::GeoPoseConverter> converter;
  try {
    converter = std::make_unique<mtc::GeoPoseConverter>(cfg);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not initialise converter: %s", e.what());
    return 1;
  }

  const std::unordered_set<std::string> selected_set(selected.begin(), selected.end());

  // 7a. Explicit -o: write a fresh bag, leaving <input> untouched.
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    auto make_writer = [&]() {
      io::CreateOptions copts;
      copts.format = io::Format::Auto;
      copts.layout = io::Layout::Auto;
      copts.mcap_compression = "none";
      return io::open_write(*args.output_path, copts);
    };
    return execute_pass(
      args.input_path, selected_set, *to_ros, target_def, *converter, make_writer);
  }

  // 7b. In-place: pin format/layout to <input>'s identity (the tmp suffix that
  //     write_bag_inplace uses cannot be interpreted by Auto resolution).
  const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not detect storage format of input bag '%s'.",
      args.input_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  int pass_status = 0;
  try {
    core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_pass(
        args.input_path, selected_set, *to_ros, target_def, *converter,
        [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("convert msgtype geo: pass failed; aborting in-place swap");
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;
    }
    BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::commands
