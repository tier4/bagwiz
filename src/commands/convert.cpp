// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/cdr_to_ros1.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/msg_definition_resolver.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/ros1_message_definitions.hpp"
#include "bagwiz/core/ros1_meta_synthesizer.hpp"
#include "bagwiz/core/ros1_to_cdr.hpp"
#include "bagwiz/core/schema_resolver.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/rosbag1_reader.hpp"
#include "bagwiz/io/rosbag1_writer.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
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

struct PerConn
{
  std::string topic;
  std::string ros1_type;
  std::string ros2_type;  // empty if topic is being skipped
  bool keep = false;      // false → drop messages on this conn
  uint64_t written = 0;
  uint64_t failures = 0;
  uint64_t overflow_events = 0;  // Total Time/Duration sign-flip events seen
                                 // for this conn across all messages.
};

// Reason a topic was excluded from 1to2 output. The bagwiz convert
// pipeline distinguishes topic-level errors (whole topic dropped, run
// exits non-zero, topic listed in the end-of-run summary) from
// per-message errors (rate-limited warning, message dropped, run can
// still succeed). Each value below is a topic-level cause.
enum class TopicSkipReason : int {
  // Source ROS 1 type does not parse as `pkg/Type` and has no rename
  // override entry. Catches typos and accidental ROS 2-shaped names in
  // ROS 1 inputs.
  TypeNameInvalid,
  // No source (bag-embedded / AMENT / introspection) could produce a
  // ROS 2 .msg text. Usually means the matching ROS 2 distro is not
  // sourced.
  SchemaUnresolvable,
  // ROS 2 schema text resolved, but `synthesize_ros1_meta()` refused
  // to canonicalise it. The synthesizer refuses when a field cannot be
  // expressed in ROS 1 form at all — currently the only such case is
  // `wstring`, which has no ROS 1 wire-equivalent counterpart.
  CanonicalisationRefused,
  // The MD5 we computed from the ROS 2 schema does not match the bag's
  // ROS 1 connection md5sum. The default behaviour is to skip the
  // topic so a wire-shape disagreement does not silently corrupt the
  // output; the policy is reshaped by --strict (abort) and
  // --allow-md5-mismatch (downgrade to a warning, admit the topic).
  Md5Mismatch,
  // Writer rejected the topic declaration (storage backend error).
  WriterDeclareFailed,
};

const char * topic_skip_reason_label(TopicSkipReason r)
{
  switch (r) {
    case TopicSkipReason::TypeNameInvalid:
      return "invalid ROS 1 type name";
    case TopicSkipReason::SchemaUnresolvable:
      return "ROS 2 schema not resolvable from any source";
    case TopicSkipReason::CanonicalisationRefused:
      return "ROS 2 schema cannot be canonicalised to ROS 1 form";
    case TopicSkipReason::Md5Mismatch:
      return "ROS 1 md5sum does not match ROS 2 schema";
    case TopicSkipReason::WriterDeclareFailed:
      return "writer rejected topic declaration";
  }
  return "unknown";
}

// Aggregated record emitted at end-of-run so users see exactly which
// topics were dropped and why. Carries the reason category plus a
// human-readable detail string (md5 hex pair, dlerror text, etc.).
struct SkippedTopic
{
  std::string topic;
  std::string ros1_type;
  std::string ros2_type;
  TopicSkipReason reason;
  std::string detail;
};

// Per-topic state used by 2to1, indexed by ROS 2 topic name.
struct TwoToOnePerTopic
{
  std::string topic;
  std::string ros2_type;
  std::string ros1_type;
  bool keep = false;
  uint32_t conn_id = 0;
  uint64_t written = 0;
  uint64_t failures = 0;
  uint64_t overflow_events = 0;  // see PerConn::overflow_events.
};

// Append the per-message overflow events emitted by the converter to the
// per-topic running tally, and (rate-limited) log the first three
// detailed events per topic. bagwiz is a wire converter, not a data
// cleanser: the wire bytes are transcribed unchanged and we surface a
// warning so the operator can decide whether the post-2038 (or pre-1970)
// timestamp is intentional.
//
// `limit` is the number of detailed log lines this topic has produced so
// far; the function increments it. Logging stops past `kMaxOverflowLogs`
// so a torrent of bad timestamps doesn't drown out other warnings.
template <typename Events>
void log_overflow_events_rate_limited(
  const std::string & topic, const Events & events, uint64_t & topic_total,
  uint64_t & detailed_logged)
{
  constexpr uint64_t kMaxOverflowLogs = 3U;
  for (const auto & ev : events) {
    ++topic_total;
    if (detailed_logged >= kMaxOverflowLogs) {
      continue;
    }
    BAGWIZ_LOG_WARN(
      kLogger, "Topic '%s': %s.%s value 0x%08x has high bit set; bytes transcribed unchanged",
      topic.c_str(), ev.type.c_str(), ev.field.c_str(), static_cast<unsigned>(ev.bits));
    ++detailed_logged;
  }
}

}  // namespace

// `bagwiz convert` is a command group for cross-format bag conversion.
// Ships `1to2` (ROS 1 -> ROS 2), `2to1` (ROS 2 -> ROS 1), and
// `storage` (ROS 2 mcap <-> sqlite3 repack).
class ConvertCommand : public Command
{
public:
  std::string_view name() const override { return "convert"; }
  std::string_view description() const override { return "Convert between bag formats"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_1to2(app);
    configure_2to1(app);
    configure_storage(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::k1to2:
        return run_1to2();
      case Subcommand::k2to1:
        return run_2to1();
      case Subcommand::kStorage:
        return run_storage();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, k1to2, k2to1, kStorage };
  Subcommand selected_ = Subcommand::kNone;

  struct OneToTwoArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;  // empty when --storage not passed; resolved at run time
    // Policy flags. Mutually exclusive at the CLI surface; default
    // behaviour (both false) is "topic-skip on any topic-level error,
    // exit 2 if anything was skipped".
    bool strict = false;              // any topic-level error → abort
    bool allow_md5_mismatch = false;  // Md5Mismatch downgraded to warn-only
    bool overwrite = false;           // replace any pre-existing output_path
  } r1_to_r2_args_;

  struct TwoToOneArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    bool strict = false;     // any topic-level error → abort
    bool overwrite = false;  // replace any pre-existing output_path
  } r2_to_r1_args_;

  struct StorageArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    std::string storage;     // empty when --storage not passed; resolved at run time
    bool overwrite = false;  // replace any pre-existing output_path
  } storage_args_;

  void configure_1to2(CLI::App & app)
  {
    auto * sub = app.add_subcommand("1to2", "Convert a ROS 1 .bag to a ROS 2 rosbag");
    sub->add_option("input", r1_to_r2_args_.input_path, "ROS 1 .bag file")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "output", r1_to_r2_args_.output_path, "Output rosbag2 directory (or .mcap/.db3 file)")
      ->required();
    sub
      ->add_option(
        "-s,--storage", r1_to_r2_args_.storage,
        "Output storage backend (default: inferred from output extension)")
      ->check(CLI::IsMember({"mcap", "sqlite3"}));
    auto * strict_flag = sub->add_flag(
      "--strict", r1_to_r2_args_.strict,
      "Abort on the first topic-level error (md5 mismatch, schema "
      "unresolvable, refused canonicalisation, writer reject) instead "
      "of skipping the topic and continuing.");
    auto * allow_flag = sub->add_flag(
      "--allow-md5-mismatch", r1_to_r2_args_.allow_md5_mismatch,
      "Treat md5 mismatch between the bag's ROS 1 connection record and "
      "the synthesised ROS 2 schema as a warning rather than a topic "
      "skip. Useful for known wire-equivalent renames (e.g. "
      "sensor_msgs/CameraInfo's D/K/R/P → d/k/r/p across the version "
      "boundary).");
    strict_flag->excludes(allow_flag);
    sub->add_flag(
      "--overwrite", r1_to_r2_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->footer(
      "Each topic's ROS 2 schema is resolved from $AMENT_PREFIX_PATH (or the\n"
      "introspection typesupport library when no .msg is on disk); a ROS 1\n"
      "md5sum is synthesised from the resolved schema and compared against\n"
      "the bag's connection record. Topics whose md5 does not match are\n"
      "skipped with a warning and the run exits non-zero (or aborted with\n"
      "--strict, or admitted with --allow-md5-mismatch).\n"
      "If --storage is omitted, the backend is inferred from the output path's\n"
      "extension (.mcap or .db3); other paths (e.g. a directory) require --storage.");
    sub->callback([this]() { selected_ = Subcommand::k1to2; });
  }

  int run_1to2()
  {
    const auto & args = r1_to_r2_args_;

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

    std::unique_ptr<io::Rosbag1Reader> reader;
    try {
      reader = std::make_unique<io::Rosbag1Reader>(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // The reader populates connections lazily as chunks are walked; pull
    // one message just to start the chunk loop so we can enumerate the
    // very first chunk's connections before declaring topics. We then
    // re-process the message after declaring the topic. Simpler
    // alternative: declare topics on-the-fly the first time we see a
    // conn_id we have not seen yet — that's what we do below.

    io::CreateOptions copts;
    copts.format = target_format;
    copts.layout = io::Layout::Auto;  // factory picks Directory unless path ends in .mcap/.db3
    // Disable mcap chunk compression by default — conversion output is
    // typically a re-record, callers can recompress later if they want.
    copts.mcap_compression = "none";

    std::unique_ptr<io::BagWriter> writer;
    try {
      writer = io::open_write(args.output_path, copts);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    std::unordered_map<uint32_t, PerConn> per_conn;
    std::unordered_set<std::string> declared_topics;

    // Track schema resolution outcomes so we can summarise at the end of
    // the run. Without this, an MCAP output that silently lost
    // self-description for half its topics would surface only as a
    // Foxglove "schema encoding '' is not supported" error downstream.
    std::size_t resolved_defs = 0;
    std::vector<SkippedTopic> skipped_topics;
    bool aborted_strict = false;  // set when --strict turns a topic-level
                                  // error into an immediate run abort.

    // Categorise + record one topic-level skip. Caller is expected to
    // also flip `pc.keep = false` so the message loop stops admitting
    // payloads for the affected conn_id. Honours --strict by setting
    // `aborted_strict` so the run can return early; the caller still
    // returns the PerConn so the immediate ensure_declared invocation
    // does not segfault, but the message loop checks the flag and bails.
    const auto record_skip = [&skipped_topics, &aborted_strict, &args](
                               const PerConn & pc, TopicSkipReason reason, std::string detail) {
      SkippedTopic s;
      s.topic = pc.topic;
      s.ros1_type = pc.ros1_type;
      s.ros2_type = pc.ros2_type;
      s.reason = reason;
      s.detail = std::move(detail);
      BAGWIZ_LOG_WARN(
        kLogger, "Skipping topic '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros1_type.c_str(),
        s.ros2_type.empty() ? "?" : s.ros2_type.c_str(), topic_skip_reason_label(reason),
        s.detail.empty() ? "" : " — ", s.detail.c_str());
      skipped_topics.push_back(std::move(s));
      if (args.strict) {
        aborted_strict = true;
      }
    };

    auto ensure_declared = [&](uint32_t conn_id) -> PerConn * {
      auto it = per_conn.find(conn_id);
      if (it != per_conn.end()) {
        return &it->second;
      }

      // Find the source connection metadata.
      const io::Ros1Connection * src = nullptr;
      for (const auto & c : reader->connections()) {
        if (c.conn_id == conn_id) {
          src = &c;
          break;
        }
      }
      if (src == nullptr) {
        // Should not happen for a well-formed bag; log once and remember
        // as skipped so we don't keep searching.
        BAGWIZ_LOG_WARN(kLogger, "Unknown conn_id %u in message stream; skipping", conn_id);
        PerConn pc;
        pc.keep = false;
        return &per_conn.emplace(conn_id, std::move(pc)).first->second;
      }

      PerConn pc;
      pc.topic = src->topic;
      pc.ros1_type = src->type;

      const auto mapped = core::map_ros1_type(src->type);
      if (!mapped) {
        record_skip(pc, TopicSkipReason::TypeNameInvalid, "");
        pc.keep = false;
        return &per_conn.emplace(conn_id, std::move(pc)).first->second;
      }
      pc.ros2_type = *mapped;

      // Same topic may appear under multiple conn_ids in ROS 1 bags
      // (e.g. one publisher per chunk). Declare the topic once with
      // BagWriter; subsequent writes to the same topic are accepted.
      if (!declared_topics.contains(pc.topic)) {
        // Resolve the ROS 2 schema. ROS 1 bags have no embedded ROS 2
        // schema, so the bag-embedded path is unused; AMENT and
        // introspection are the candidate sources.
        core::ResolveSchemaInput resolve_in;
        resolve_in.ros2_type = pc.ros2_type;
        const auto resolved = core::resolve_schema(resolve_in);
        if (!resolved.ok) {
          // None of the three sources produced a schema. Most failure
          // detail lives on the AMENT / introspection candidates; pull
          // them into one short string for the summary.
          std::string detail;
          for (const auto & c : resolved.candidates) {
            if (!c.error.empty()) {
              if (!detail.empty()) {
                detail += "; ";
              }
              detail += c.error;
            }
          }
          record_skip(pc, TopicSkipReason::SchemaUnresolvable, std::move(detail));
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }

        // Synthesise the canonical ROS 1 form to compare md5 against
        // the bag-recorded md5sum. The canonicalisation can refuse
        // outright (currently only for `wstring`, which has no ROS 1
        // wire-equivalent representation) — surface that as a topic-
        // level skip.
        const auto meta = core::synthesize_ros1_meta(pc.ros2_type, resolved.text);
        if (!meta.ok) {
          record_skip(pc, TopicSkipReason::CanonicalisationRefused, meta.error);
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }

        if (meta.meta.md5sum != src->md5sum) {
          // Silent-corruption guard: producer and our schema disagree
          // on the wire shape. Default policy is topic-skip + non-zero
          // exit; --allow-md5-mismatch downgrades to a warning that
          // still admits the topic (useful for known wire-equivalent
          // renames like sensor_msgs/CameraInfo's D/K/R/P → d/k/r/p
          // across the ROS 1/ROS 2 boundary); --strict promotes any
          // topic-level error to abort.
          std::string detail = "bag md5=" + src->md5sum + " synthesised=" + meta.meta.md5sum +
                               " (source=" + std::string(core::to_string(resolved.source)) + ")";
          if (args.allow_md5_mismatch) {
            BAGWIZ_LOG_WARN(
              kLogger,
              "md5 mismatch on topic '%s' (%s -> %s) admitted by "
              "--allow-md5-mismatch: %s",
              pc.topic.c_str(), pc.ros1_type.c_str(), pc.ros2_type.c_str(), detail.c_str());
            // Fall through and declare the topic; wire bytes are
            // forwarded as-is, schema text reflects the local ROS 2
            // type. Receiver-side compatibility is the user's call.
          } else {
            record_skip(pc, TopicSkipReason::Md5Mismatch, std::move(detail));
            pc.keep = false;
            return &per_conn.emplace(conn_id, std::move(pc)).first->second;
          }
        }

        io::TopicInfo t;
        t.name = pc.topic;
        t.type = pc.ros2_type;
        t.serialization_format = "cdr";
        t.offered_qos_profiles = "";  // ROS 1 has no equivalent
        t.schema_text = resolved.text;
        t.schema_encoding = resolved.encoding;
        ++resolved_defs;

        try {
          writer->declare_topic(t);
          declared_topics.insert(pc.topic);
          BAGWIZ_LOG_INFO(
            kLogger, "Mapped '%s': %s -> %s [md5 ok via %s]", pc.topic.c_str(),
            pc.ros1_type.c_str(), pc.ros2_type.c_str(),
            std::string(core::to_string(resolved.source)).c_str());
        } catch (const std::exception & e) {
          record_skip(pc, TopicSkipReason::WriterDeclareFailed, e.what());
          pc.keep = false;
          return &per_conn.emplace(conn_id, std::move(pc)).first->second;
        }
      }

      pc.keep = !pc.ros2_type.empty();
      return &per_conn.emplace(conn_id, std::move(pc)).first->second;
    };

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    // Per-topic counters for rate-limited overflow logs. Keyed by topic
    // (not conn_id) since we surface the warnings at topic granularity.
    std::unordered_map<std::string, uint64_t> overflow_log_counts;
    io::Ros1Message msg;
    while (true) {
      try {
        if (!reader->next(msg)) {
          break;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "ros1 read error: %s", e.what());
        return 1;
      }
      ++total_in;

      auto * pc = ensure_declared(msg.conn_id);
      if (aborted_strict) {
        // ensure_declared just hit a topic-level error and --strict was
        // set; bail without writing the half-finished output.
        BAGWIZ_LOG_ERROR(kLogger, "Aborting per --strict: see preceding skip warning");
        return 2;
      }
      if (pc == nullptr || !pc->keep) {
        continue;
      }

      auto result = core::convert_ros1_to_cdr(pc->ros2_type, msg.payload);
      if (!result.ok) {
        ++pc->failures;
        if (pc->failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "convert failed on '%s' (type %s): %s", pc->topic.c_str(),
            pc->ros2_type.c_str(), result.error.c_str());
        }
        continue;
      }

      if (!result.overflows.empty()) {
        log_overflow_events_rate_limited(
          pc->topic, result.overflows, pc->overflow_events, overflow_log_counts[pc->topic]);
      }

      try {
        writer->write(pc->topic, msg.timestamp_ns, std::span<const std::byte>(result.cdr));
        ++pc->written;
        ++total_out;
      } catch (const std::exception & e) {
        // Per-message write failures are downgraded to a (rate-limited)
        // warning so a single bad message cannot abort the entire bag.
        // Hard storage errors (disk full, etc.) will keep firing here
        // and the user will see the per-topic failure tally at the end.
        ++pc->failures;
        if (pc->failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", pc->topic.c_str(),
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
      kLogger, "Conversion done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, declared_topics.size());
    if (resolved_defs > 0) {
      BAGWIZ_LOG_INFO(
        kLogger, "resolved %zu message definition(s) with md5 verified against bag", resolved_defs);
    }
    for (const auto & [conn_id, pc] : per_conn) {
      if (pc.keep && pc.failures > 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "Topic '%s': %" PRIu64 " message(s) failed to convert", pc.topic.c_str(),
          pc.failures);
      }
      if (pc.keep && pc.overflow_events > 0) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "Topic '%s': %" PRIu64
          " Time/Duration sign-flip event(s); "
          "wire bytes preserved (bagwiz transcribes timestamps verbatim)",
          pc.topic.c_str(), pc.overflow_events);
      }
    }

    // Topic-skip summary. Exit code reflects whether any topic was
    // dropped: zero only when every topic in the input was emitted.
    if (!skipped_topics.empty()) {
      BAGWIZ_LOG_WARN(kLogger, "Skipped %zu topic(s):", skipped_topics.size());
      // Show at most the first 5 in detail to keep the trailing summary
      // tractable on bags with many divergent topics; tally the rest by
      // reason category so users still see scope.
      constexpr std::size_t kMaxDetailRows = 5;
      const std::size_t shown = std::min(kMaxDetailRows, skipped_topics.size());
      for (std::size_t i = 0; i < shown; ++i) {
        const auto & s = skipped_topics[i];
        BAGWIZ_LOG_WARN(
          kLogger, "  - '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros1_type.c_str(),
          s.ros2_type.empty() ? "?" : s.ros2_type.c_str(), topic_skip_reason_label(s.reason),
          s.detail.empty() ? "" : " — ", s.detail.c_str());
      }
      if (skipped_topics.size() > kMaxDetailRows) {
        BAGWIZ_LOG_WARN(
          kLogger, "  (... %zu more skipped topic(s) not shown)",
          skipped_topics.size() - kMaxDetailRows);
      }
      return 2;  // non-zero exit so callers can detect partial conversion
    }

    return 0;
  }

  void configure_2to1(CLI::App & app)
  {
    auto * sub = app.add_subcommand("2to1", "Convert a ROS 2 rosbag to a ROS 1 .bag file");
    sub->add_option("input", r2_to_r1_args_.input_path, "ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("output", r2_to_r1_args_.output_path, "Output ROS 1 .bag file")->required();
    sub->add_flag(
      "--strict", r2_to_r1_args_.strict,
      "Abort on the first topic-level error (schema unresolvable, "
      "refused canonicalisation, writer reject) instead of skipping "
      "the topic and continuing.");
    sub->add_flag(
      "--overwrite", r2_to_r1_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->footer(
      "Each topic's ROS 2 schema is taken from the bag's self-describing\n"
      "record when present, or resolved from $AMENT_PREFIX_PATH /\n"
      "introspection typesupport otherwise. The ROS 1 connection's md5sum\n"
      "and message_definition are synthesised from that schema. Topics\n"
      "whose schema cannot be resolved are skipped and the run exits non-zero.\n"
      "Output is a non-compressed ROS 1 bag v2.0; rosbag2-layer compression\n"
      "(compression_mode: FILE / MESSAGE) on the input is not supported.");
    sub->callback([this]() { selected_ = Subcommand::k2to1; });
  }

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

  int run_2to1()
  {
    const auto & args = r2_to_r1_args_;

    if (const int rc = check_input_compression(args.input_path); rc != 0) {
      return rc;
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

    // Build per-topic state from the reader's topic list. The reader
    // returns TopicInfo entries up front (no message scan needed); we
    // declare connections in the writer eagerly so any
    // unresolvable-type warnings surface before the message loop runs.
    std::unique_ptr<io::Rosbag1Writer> writer;
    try {
      writer = std::make_unique<io::Rosbag1Writer>(args.output_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open output %s: %s", args.output_path.c_str(), e.what());
      return 1;
    }

    std::unordered_map<std::string, TwoToOnePerTopic> per_topic;
    std::vector<SkippedTopic> skipped_topics;
    std::size_t synthesised_defs = 0;
    bool aborted_strict = false;

    // Categorise + record one topic-level skip in 2to1. The 2to1 record
    // shape is the same as 1to2 (`SkippedTopic`); only the field
    // population order differs (here we know `ros2_type` first).
    const auto record_skip_2to1 = [&skipped_topics, &aborted_strict, &args](
                                    const TwoToOnePerTopic & st, TopicSkipReason reason,
                                    std::string detail) {
      SkippedTopic s;
      s.topic = st.topic;
      s.ros1_type = st.ros1_type;
      s.ros2_type = st.ros2_type;
      s.reason = reason;
      s.detail = std::move(detail);
      BAGWIZ_LOG_WARN(
        kLogger, "Skipping topic '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros2_type.c_str(),
        s.ros1_type.empty() ? "?" : s.ros1_type.c_str(), topic_skip_reason_label(reason),
        s.detail.empty() ? "" : " — ", s.detail.c_str());
      skipped_topics.push_back(std::move(s));
      if (args.strict) {
        aborted_strict = true;
      }
    };

    for (const auto & t : reader->topics()) {
      TwoToOnePerTopic state;
      state.topic = t.name;
      state.ros2_type = t.type;

      auto mapped = core::map_ros2_type(t.type);
      if (!mapped) {
        record_skip_2to1(state, TopicSkipReason::TypeNameInvalid, "");
        per_topic.emplace(state.topic, std::move(state));
        continue;
      }
      state.ros1_type = *mapped;

      // Resolve the ROS 2 schema, preferring the bag-embedded text when
      // the storage layer surfaces one. A self-described bag is the
      // only schema source guaranteed to match what the producer
      // actually serialised, even if the local AMENT install drifts;
      // resolve_schema() falls back to AMENT and then introspection
      // when no bag-embedded text is available.
      core::ResolveSchemaInput resolve_in;
      resolve_in.ros2_type = state.ros2_type;
      resolve_in.bag_embedded_text = t.schema_text;
      resolve_in.bag_embedded_encoding = t.schema_encoding;
      const auto resolved = core::resolve_schema(resolve_in);
      if (!resolved.ok) {
        std::string detail;
        for (const auto & c : resolved.candidates) {
          if (!c.error.empty()) {
            if (!detail.empty()) {
              detail += "; ";
            }
            detail += c.error;
          }
        }
        record_skip_2to1(state, TopicSkipReason::SchemaUnresolvable, std::move(detail));
        per_topic.emplace(state.topic, std::move(state));
        continue;
      }

      // Synthesise the canonical ROS 1 form's md5 + concatenated
      // message_definition. The synthesizer applies the wire-equivalent
      // normalisation rules (drop ROS 2-only sequence/string bounds,
      // strip default values, restore the Header.seq prefix, refuse
      // wstring) and computes the md5 over the resulting ROS 1-form
      // text.
      const auto meta = core::synthesize_ros1_meta(state.ros2_type, resolved.text);
      if (!meta.ok) {
        record_skip_2to1(state, TopicSkipReason::CanonicalisationRefused, meta.error);
        per_topic.emplace(state.topic, std::move(state));
        continue;
      }

      // Crosscheck against the pinned ROS 1 reference table for known
      // canonical types. The synthesizer's input is the resolved ROS 2
      // schema text, and the introspection fallback can't recover
      // `<type> NAME=value` constant declarations (they are not exposed
      // on the rosidl introspection metadata). For constant-bearing
      // types (sensor_msgs/NavSatStatus, diagnostic_msgs/DiagnosticStatus,
      // ...) this drops constant lines from the synthesized text, which
      // shifts the canonical md5. Writing that mis-md5 into the ROS 1
      // connection record is silent corruption: receivers reject the
      // connection at read time. Prefer the pinned canonical metadata
      // whenever it disagrees with the synthesized form so the output
      // bag stays compatible with stock ROS 1 readers.
      std::string md5_to_write = meta.meta.md5sum;
      std::string msgdef_to_write = meta.meta.message_definition;
      const char * md5_origin_label = "synthesised";
      const auto * pinned = core::find_ros1_meta(state.ros1_type);
      if (pinned != nullptr && pinned->md5sum != meta.meta.md5sum) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "Synthesised md5 for '%s' (%s -> %s) differs from pinned canonical (%s vs %s, "
          "source=%s); using pinned canonical so the ROS 1 connection record stays "
          "compatible with stock ROS 1 readers.",
          state.topic.c_str(), state.ros2_type.c_str(), state.ros1_type.c_str(),
          meta.meta.md5sum.c_str(), pinned->md5sum.c_str(),
          std::string(core::to_string(resolved.source)).c_str());
        md5_to_write = pinned->md5sum;
        msgdef_to_write = pinned->message_definition;
        md5_origin_label = "pinned-canonical";
      }

      try {
        state.conn_id =
          writer->declare_connection(state.topic, state.ros1_type, md5_to_write, msgdef_to_write);
        state.keep = true;
        ++synthesised_defs;
        BAGWIZ_LOG_INFO(
          kLogger, "Mapped '%s': %s -> %s [md5 %s via %s]", state.topic.c_str(),
          state.ros2_type.c_str(), state.ros1_type.c_str(), md5_origin_label,
          std::string(core::to_string(resolved.source)).c_str());
      } catch (const std::exception & e) {
        record_skip_2to1(state, TopicSkipReason::WriterDeclareFailed, e.what());
      }
      per_topic.emplace(state.topic, std::move(state));
      if (aborted_strict) {
        break;  // out of the per-topic setup loop; the abort handler
                // below returns 2 without entering the message loop.
      }
    }

    if (aborted_strict) {
      BAGWIZ_LOG_ERROR(kLogger, "Aborting per --strict: see preceding skip warning");
      return 2;
    }

    uint64_t total_in = 0;
    uint64_t total_out = 0;
    std::unordered_map<std::string, uint64_t> overflow_log_counts;
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
      auto it = per_topic.find(msg.topic->name);
      if (it == per_topic.end() || !it->second.keep) {
        continue;
      }
      auto & st = it->second;

      auto result = core::convert_cdr_to_ros1(st.ros2_type, msg.payload);
      if (!result.ok) {
        ++st.failures;
        if (st.failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "convert failed on '%s' (type %s): %s", st.topic.c_str(), st.ros2_type.c_str(),
            result.error.c_str());
        }
        continue;
      }

      if (!result.overflows.empty()) {
        log_overflow_events_rate_limited(
          st.topic, result.overflows, st.overflow_events, overflow_log_counts[st.topic]);
      }

      try {
        writer->write(st.conn_id, msg.timestamp_ns, std::span<const std::byte>(result.ros1));
        ++st.written;
        ++total_out;
      } catch (const std::exception & e) {
        ++st.failures;
        if (st.failures <= 3) {
          BAGWIZ_LOG_WARN(
            kLogger, "writer->write failed on '%s': %s; skipping message", st.topic.c_str(),
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

    std::size_t kept_topics = 0;
    for (const auto & entry : per_topic) {
      if (entry.second.keep) {
        ++kept_topics;
      }
    }
    BAGWIZ_LOG_INFO(
      kLogger, "Conversion done: %" PRIu64 "/%" PRIu64 " messages written across %zu topic(s)",
      total_out, total_in, kept_topics);
    if (synthesised_defs > 0) {
      BAGWIZ_LOG_INFO(kLogger, "synthesised ROS 1 metadata for %zu topic(s)", synthesised_defs);
    }
    for (const auto & entry : per_topic) {
      const auto & st = entry.second;
      if (st.keep && st.failures > 0) {
        BAGWIZ_LOG_WARN(
          kLogger, "Topic '%s': %" PRIu64 " message(s) failed to convert", st.topic.c_str(),
          st.failures);
      }
      if (st.keep && st.overflow_events > 0) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "Topic '%s': %" PRIu64
          " Time/Duration sign-flip event(s); "
          "wire bytes preserved (bagwiz transcribes timestamps verbatim)",
          st.topic.c_str(), st.overflow_events);
      }
    }

    // Topic-skip summary — matches the 1to2 shape; the topic-level
    // skip policy is direction-agnostic.
    if (!skipped_topics.empty()) {
      BAGWIZ_LOG_WARN(kLogger, "Skipped %zu topic(s):", skipped_topics.size());
      constexpr std::size_t kMaxDetailRows = 5;
      const std::size_t shown = std::min(kMaxDetailRows, skipped_topics.size());
      for (std::size_t i = 0; i < shown; ++i) {
        const auto & s = skipped_topics[i];
        BAGWIZ_LOG_WARN(
          kLogger, "  - '%s' (%s -> %s): %s%s%s", s.topic.c_str(), s.ros2_type.c_str(),
          s.ros1_type.empty() ? "?" : s.ros1_type.c_str(), topic_skip_reason_label(s.reason),
          s.detail.empty() ? "" : " — ", s.detail.c_str());
      }
      if (skipped_topics.size() > kMaxDetailRows) {
        BAGWIZ_LOG_WARN(
          kLogger, "  (... %zu more skipped topic(s) not shown)",
          skipped_topics.size() - kMaxDetailRows);
      }
      return 2;
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
    // Mirror 1to2: leave compression off so the output is predictable;
    // callers can recompress with `ros2 bag convert` if they want.
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
