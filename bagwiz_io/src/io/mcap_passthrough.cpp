// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_passthrough.hpp"

#include "mcap_chunk_codec.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr std::size_t kRecordHeaderBytes = 1 + 8;  // opcode + record length
// Message record body prefix: channelId u16, sequence u32, logTime u64,
// publishTime u64 — the payload follows.
constexpr std::size_t kMessagePrefixBytes = 2 + 4 + 8 + 8;
// Cap for record bodies the walk materializes in memory (Header, top-level
// Channel records, MessageIndex regions). Real records of these kinds are at
// most a few hundred KB (schema texts); anything larger signals corruption.
constexpr std::uint64_t kMaxSmallRecordBytes = 64ull * 1024 * 1024;
constexpr std::size_t kCopySlabBytes = 8ull * 1024 * 1024;

std::uint16_t read_u16(const std::byte * p)
{
  std::uint16_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint32_t read_u32(const std::byte * p)
{
  std::uint32_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

std::uint64_t read_u64(const std::byte * p)
{
  std::uint64_t v = 0;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// One data-section chunk with the action the edit assigns to it.
struct PlanEntry
{
  enum class Action : std::uint8_t {
    kCopy,      // untouched: copy chunk + MessageIndex records byte-for-byte
    kReencode,  // touched: decompress, filter records, re-encode
    kDrop       // entirely outside the edit: omit
  };

  const mcap::ChunkIndex * index = nullptr;
  std::uint64_t start = 0;  // file offset of the Chunk record
  std::uint64_t span = 0;   // chunkLength + messageIndexLength
  std::string codec;        // compression string from the Chunk record itself
  Action action = Action::kReencode;
};

// The whole rewrite as one single-use object, so the probe/walk/write phases
// can share state without threading a dozen parameters through free
// functions. fail() records the fallback reason; genuine I/O errors throw.
class PassthroughEngine
{
public:
  PassthroughEngine(
    const std::filesystem::path & input, const std::filesystem::path & output,
    const McapPassthroughEdit & edit)
  : input_(input), output_(output), edit_(edit)
  {
  }

  std::optional<McapPassthroughResult> run(std::string * fallback_reason)
  {
    std::optional<McapPassthroughResult> result;
    if (probe() && walk() && write_output()) {
      result = build_result();
    }
    reader_.close();
    if (!result && fallback_reason != nullptr) {
      *fallback_reason = reason_;
    }
    return result;
  }

private:
  using Action = PlanEntry::Action;

  bool fail(std::string reason)
  {
    reason_ = std::move(reason);
    return false;
  }

  // --- phase 0: summary probe -------------------------------------------

  bool probe()
  {
    if (auto status = reader_.open(input_.string()); !status.ok()) {
      return fail("cannot open input as mcap: " + status.message);
    }
    if (auto status = reader_.readSummary(mcap::ReadSummaryMethod::NoFallbackScan); !status.ok()) {
      return fail("input has no readable summary section: " + status.message);
    }

    channels_ = reader_.channels();
    schemas_ = reader_.schemas();

    // Duplicate topic names (legal in mcap) are merged onto one channel by
    // the decoded pipeline; preserving them verbatim would diverge.
    std::unordered_set<std::string> seen_names;
    for (const auto & [id, channel] : channels_) {
      if (!seen_names.insert(channel->topic).second) {
        return fail("multiple channels share topic name " + channel->topic);
      }
      if (channel->schemaId != 0 && schemas_.find(channel->schemaId) == schemas_.end()) {
        return fail("channel " + channel->topic + " references a missing schema");
      }
      if (edit_.drop_topics.count(channel->topic) != 0) {
        dropped_ids_.insert(id);
      } else if (auto it = edit_.rename.find(channel->topic); it != edit_.rename.end()) {
        renamed_ids_.emplace(id, it->second);
      }
    }

    // A rename target colliding with another surviving topic would also
    // need the pipeline's merge semantics.
    std::unordered_set<std::string> out_names;
    for (const auto & [id, channel] : channels_) {
      if (dropped_ids_.count(id) != 0) {
        continue;
      }
      const auto renamed = renamed_ids_.find(id);
      const std::string & name = renamed != renamed_ids_.end() ? renamed->second : channel->topic;
      if (!out_names.insert(name).second) {
        return fail("rename target collides with an existing topic: " + name);
      }
    }

    if (edit_.start_ns.has_value()) {
      start_ns_ = static_cast<std::uint64_t>(std::max<std::int64_t>(*edit_.start_ns, 0));
    }
    if (edit_.end_ns.has_value()) {
      end_ns_ = static_cast<std::uint64_t>(std::max<std::int64_t>(*edit_.end_ns, 0));
    }
    return true;
  }

  // --- phase 0: data-section coverage walk ------------------------------

  bool read_at(std::uint64_t offset, std::byte * dst, std::size_t size)
  {
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(offset));
    file_.read(reinterpret_cast<char *>(dst), static_cast<std::streamsize>(size));
    return static_cast<bool>(file_) && static_cast<std::size_t>(file_.gcount()) == size;
  }

  bool read_small_body(std::uint64_t offset, std::uint64_t length, std::vector<std::byte> & buf)
  {
    if (length > kMaxSmallRecordBytes) {
      return false;
    }
    buf.resize(static_cast<std::size_t>(length));
    return read_at(offset, buf.data(), buf.size());
  }

  // Walk every top-level record header from the Header to DataEnd. This is
  // the authoritative eligibility check: the summary indexes are only
  // trusted after the walk confirms chunks sit exactly where the ChunkIndex
  // records claim and nothing else (top-level Messages, Attachments,
  // Metadata, unknown records) lives in the data section. The walk output
  // doubles as the copy plan, so eligible inputs pay for it once.
  bool walk()
  {
    file_.open(input_, std::ios::binary);
    if (!file_) {
      return fail("cannot reopen input for the record walk");
    }

    std::vector<const mcap::ChunkIndex *> sorted;
    sorted.reserve(reader_.chunkIndexes().size());
    for (const auto & ci : reader_.chunkIndexes()) {
      sorted.push_back(&ci);
    }
    std::sort(sorted.begin(), sorted.end(), [](const auto * a, const auto * b) {
      return a->chunkStartOffset < b->chunkStartOffset;
    });
    plan_.reserve(sorted.size());

    std::uint64_t pos = 8;  // past the leading magic
    std::size_t next_chunk = 0;
    bool header_seen = false;
    std::vector<std::byte> body;
    while (true) {
      std::byte hdr[kRecordHeaderBytes];
      if (!read_at(pos, hdr, sizeof(hdr))) {
        return fail("data section ends without a DataEnd record");
      }
      const auto opcode = std::to_integer<std::uint8_t>(hdr[0]);
      const std::uint64_t length = read_u64(hdr + 1);
      const std::uint64_t body_pos = pos + kRecordHeaderBytes;

      if (!header_seen) {
        if (opcode != static_cast<std::uint8_t>(mcap::OpCode::Header)) {
          return fail("file does not start with a Header record");
        }
        if (!read_small_body(body_pos, length, body)) {
          return fail("cannot read the Header record");
        }
        mcap::Record record{mcap::OpCode::Header, length, body.data()};
        if (auto status = mcap::McapReader::ParseHeader(record, &header_); !status.ok()) {
          return fail("cannot parse the Header record: " + status.message);
        }
        header_seen = true;
        pos = body_pos + length;
        continue;
      }

      switch (static_cast<mcap::OpCode>(opcode)) {
        case mcap::OpCode::Schema:
          // Not copied: the surviving Schema set is re-emitted from the
          // summary right after the Header.
          break;
        case mcap::OpCode::Channel: {
          if (!read_small_body(body_pos, length, body) || length < 2) {
            return fail("cannot read a top-level Channel record");
          }
          toplevel_channel_ids_.insert(read_u16(body.data()));
          break;
        }
        case mcap::OpCode::Chunk: {
          if (next_chunk >= sorted.size() || pos != sorted[next_chunk]->chunkStartOffset) {
            return fail("chunk record not covered by the chunk index");
          }
          const mcap::ChunkIndex & ci = *sorted[next_chunk];
          if (ci.chunkLength != kRecordHeaderBytes + length) {
            return fail("chunk index disagrees with the chunk record length");
          }
          if (!classify_chunk(ci, pos)) {
            return false;
          }
          ++next_chunk;
          pos = ci.chunkStartOffset + ci.chunkLength + ci.messageIndexLength;
          continue;
        }
        case mcap::OpCode::Attachment:
          // Dropped, like the decoded pipeline drops them — but counted, so
          // the caller can warn instead of losing content silently.
          ++attachments_skipped_;
          break;
        case mcap::OpCode::Metadata:
          // Same as Attachment. rosbag2 stamps every recording with one
          // Metadata record, so this is the common case, not an anomaly.
          ++metadata_skipped_;
          break;
        case mcap::OpCode::DataEnd:
          if (next_chunk != sorted.size()) {
            return fail("chunk index lists chunks the data section does not contain");
          }
          return true;
        default:
          return fail(
            "unsupported record in the data section (opcode " + std::to_string(opcode) + ")");
      }
      pos = body_pos + length;
    }
  }

  // Classify one chunk and, for verbatim copies, tally its message counts
  // from the MessageIndex records that will be copied along with it.
  bool classify_chunk(const mcap::ChunkIndex & ci, std::uint64_t pos)
  {
    PlanEntry entry;
    entry.index = &ci;
    entry.start = pos;
    entry.span = ci.chunkLength + ci.messageIndexLength;

    // The chunk record's own header: start/end time (16 bytes) and the
    // compression string, needed to cross-check the index entry and to
    // re-encode with the chunk's own codec.
    constexpr std::size_t kChunkPrefixBytes = 8 + 8 + 8 + 4 + 4;
    constexpr std::uint32_t kMaxCodecBytes = 32;
    std::byte prefix[kChunkPrefixBytes];
    const std::uint64_t body_pos = pos + kRecordHeaderBytes;
    if (
      ci.chunkLength < kRecordHeaderBytes + kChunkPrefixBytes ||
      !read_at(body_pos, prefix, sizeof(prefix))) {
      return fail("chunk record too short for its header");
    }
    const std::uint64_t chunk_start_time = read_u64(prefix);
    const std::uint64_t chunk_end_time = read_u64(prefix + 8);
    const std::uint32_t codec_len = read_u32(prefix + 28);
    if (
      codec_len > kMaxCodecBytes ||
      kRecordHeaderBytes + kChunkPrefixBytes + codec_len > ci.chunkLength) {
      return fail("chunk record carries an implausible compression string");
    }
    std::byte codec_buf[kMaxCodecBytes];
    if (codec_len > 0 && !read_at(body_pos + kChunkPrefixBytes, codec_buf, codec_len)) {
      return fail("cannot read a chunk's compression string");
    }
    entry.codec.assign(reinterpret_cast<const char *>(codec_buf), codec_len);

    // Only classify by time when the chunk record and its index entry agree
    // and the bounds are non-degenerate; otherwise re-encode, which
    // recomputes the truth per message record.
    const bool times_trusted = chunk_start_time == ci.messageStartTime &&
                               chunk_end_time == ci.messageEndTime &&
                               !(ci.messageStartTime == 0 && ci.messageEndTime == 0);
    const bool has_dropped = std::any_of(
      ci.messageIndexOffsets.begin(), ci.messageIndexOffsets.end(),
      [&](const auto & e) { return dropped_ids_.count(e.first) != 0; });
    const bool has_renamed = std::any_of(
      ci.messageIndexOffsets.begin(), ci.messageIndexOffsets.end(),
      [&](const auto & e) { return renamed_ids_.count(e.first) != 0; });
    const bool all_dropped = !ci.messageIndexOffsets.empty() &&
                             std::all_of(
                               ci.messageIndexOffsets.begin(), ci.messageIndexOffsets.end(),
                               [&](const auto & e) { return dropped_ids_.count(e.first) != 0; });

    const bool outside_window = ci.messageEndTime < start_ns_ || ci.messageStartTime >= end_ns_;
    const bool inside_window = ci.messageStartTime >= start_ns_ && ci.messageEndTime < end_ns_;
    if (times_trusted && (outside_window || all_dropped)) {
      entry.action = Action::kDrop;
    } else if (
      times_trusted && inside_window && !has_dropped && !has_renamed && ci.messageIndexLength > 0 &&
      !ci.messageIndexOffsets.empty()) {
      entry.action = Action::kCopy;
    } else {
      entry.action = Action::kReencode;
      if (!known_codec(entry.codec)) {
        return fail("chunk that must be re-encoded uses unsupported codec " + entry.codec);
      }
    }

    if (entry.action == Action::kCopy && !tally_copied_chunk(ci)) {
      return false;
    }
    plan_.push_back(std::move(entry));
    return true;
  }

  static bool known_codec(const std::string & codec)
  {
    return codec.empty() || codec == "none" || codec == "zstd" || codec == "lz4";
  }

  // Per-channel message counts of a verbatim chunk, read from the
  // MessageIndex records that follow it: each record's `records` field is a
  // length-prefixed array of (logTime u64, offset u64) pairs, so the count
  // is the field's byte length / 16 — no pair iteration needed.
  bool tally_copied_chunk(const mcap::ChunkIndex & ci)
  {
    std::vector<std::byte> region;
    if (!read_small_body(ci.chunkStartOffset + ci.chunkLength, ci.messageIndexLength, region)) {
      return fail("cannot read a chunk's MessageIndex region");
    }
    std::size_t pos = 0;
    std::uint64_t chunk_messages = 0;
    while (pos + kRecordHeaderBytes <= region.size()) {
      const auto opcode = std::to_integer<std::uint8_t>(region[pos]);
      const std::uint64_t length = read_u64(region.data() + pos + 1);
      if (
        opcode != static_cast<std::uint8_t>(mcap::OpCode::MessageIndex) || length < 2 + 4 ||
        length > region.size() - pos - kRecordHeaderBytes) {
        return fail("malformed MessageIndex region after a chunk");
      }
      const std::byte * mi_body = region.data() + pos + kRecordHeaderBytes;
      const std::uint16_t channel_id = read_u16(mi_body);
      const std::uint32_t records_bytes = read_u32(mi_body + 2);
      if (records_bytes % 16 != 0 || records_bytes + 2 + 4 > length) {
        return fail("malformed MessageIndex record after a chunk");
      }
      const std::uint64_t count = records_bytes / 16;
      channel_counts_[channel_id] += count;
      chunk_messages += count;
      pos += kRecordHeaderBytes + static_cast<std::size_t>(length);
    }
    if (pos != region.size()) {
      return fail("MessageIndex region after a chunk has trailing bytes");
    }
    message_count_ += chunk_messages;
    if (chunk_messages > 0) {
      min_time_ = std::min(min_time_, ci.messageStartTime);
      max_time_ = std::max(max_time_, ci.messageEndTime);
    }
    return true;
  }

  // --- phases 1-3: write the output file --------------------------------

  bool write_output()
  {
    collect_survivors();

    mcap::FileWriter out;
    if (auto status = out.open(output_.string()); !status.ok()) {
      throw std::runtime_error(
        "cannot open pass-through output " + output_.string() + ": " + status.message);
    }

    if (!write_data_section(out)) {
      // Late abort (rename verification, malformed chunk interior): remove
      // the partial file so the caller can fall back onto a clean slate.
      out.end();
      std::error_code ec;
      std::filesystem::remove(output_, ec);
      return false;
    }
    write_summary_section(out);
    out.end();
    return true;
  }

  void collect_survivors()
  {
    // std::map orders the output by id, keeping emission deterministic.
    std::map<mcap::SchemaId, const mcap::Schema *> schema_by_id;
    std::map<mcap::ChannelId, const mcap::Channel *> channel_by_id;
    for (const auto & [id, channel] : channels_) {
      if (dropped_ids_.count(id) != 0) {
        continue;
      }
      channel_by_id.emplace(id, channel.get());
      if (channel->schemaId != 0) {
        schema_by_id.emplace(channel->schemaId, schemas_.at(channel->schemaId).get());
      }
    }
    for (const auto & entry : schema_by_id) {
      out_schemas_.push_back(*entry.second);
    }
    for (const auto & [id, channel] : channel_by_id) {
      mcap::Channel copy = *channel;
      if (auto it = renamed_ids_.find(id); it != renamed_ids_.end()) {
        copy.topic = it->second;
      }
      out_channels_.push_back(std::move(copy));
    }
  }

  bool write_data_section(mcap::FileWriter & out)
  {
    mcap::McapWriter::writeMagic(out);
    // The recording profile travels with the data; the library string names
    // the producer, matching how every mcap writer signs its output.
    mcap::McapWriter::write(out, mcap::Header{header_.profile, "bagwiz"});

    // Top-level Schema/Channel block: see mcap_passthrough.hpp. The static
    // record writers serialize the ids exactly as set, which is the whole
    // point — the instance API would renumber them.
    for (const auto & schema : out_schemas_) {
      mcap::Schema copy = schema;
      mcap::McapWriter::write(out, copy);
    }
    for (const auto & channel : out_channels_) {
      mcap::Channel copy = channel;
      mcap::McapWriter::write(out, copy);
    }

    for (std::size_t i = 0; i < plan_.size();) {
      switch (plan_[i].action) {
        case Action::kCopy: {
          // Coalesce the run of contiguous verbatim chunks into one span so
          // sparse edits degenerate into a few large sequential copies.
          std::size_t j = i;
          while (j + 1 < plan_.size() && plan_[j + 1].action == Action::kCopy &&
                 plan_[j + 1].start == plan_[j].start + plan_[j].span) {
            ++j;
          }
          copy_span(out, i, j);
          i = j + 1;
          break;
        }
        case Action::kReencode:
          if (!reencode_chunk(out, plan_[i])) {
            return false;
          }
          ++i;
          break;
        case Action::kDrop:
          ++chunks_dropped_;
          ++i;
          break;
      }
    }

    // Rename verification: every renamed channel's embedded Channel record
    // must have been stripped from a re-encoded chunk (or declared at the
    // top level of the input). A writer that embedded it in a chunk
    // carrying none of that channel's messages would otherwise leave a
    // stale record under the old name inside a verbatim chunk.
    for (const auto & [id, new_topic] : renamed_ids_) {
      if (rename_seen_.count(id) == 0 && toplevel_channel_ids_.count(id) == 0) {
        return fail("rename: the renamed topic's embedded Channel record was not located");
      }
    }

    mcap::McapWriter::write(out, mcap::DataEnd{0});
    return true;
  }

  void copy_span(mcap::FileWriter & out, std::size_t first, std::size_t last)
  {
    const std::uint64_t span_start = plan_[first].start;
    const std::uint64_t span_end = plan_[last].start + plan_[last].span;
    const std::uint64_t delta = out.size() - span_start;

    file_.clear();
    file_.seekg(static_cast<std::streamoff>(span_start));
    if (copy_slab_.empty()) {
      copy_slab_.resize(kCopySlabBytes);
    }
    std::uint64_t remaining = span_end - span_start;
    while (remaining > 0) {
      const auto want =
        static_cast<std::streamsize>(std::min<std::uint64_t>(remaining, copy_slab_.size()));
      file_.read(reinterpret_cast<char *>(copy_slab_.data()), want);
      if (file_.gcount() != want) {
        throw std::runtime_error("short read while copying chunks from " + input_.string());
      }
      out.write(copy_slab_.data(), static_cast<std::uint64_t>(want));
      remaining -= static_cast<std::uint64_t>(want);
    }

    for (std::size_t k = first; k <= last; ++k) {
      mcap::ChunkIndex ci = *plan_[k].index;
      ci.chunkStartOffset += delta;
      for (auto & entry : ci.messageIndexOffsets) {
        entry.second += delta;
      }
      out_chunk_indexes_.push_back(std::move(ci));
      note_written_codec(plan_[k].codec);
      ++chunks_copied_;
    }
  }

  bool reencode_chunk(mcap::FileWriter & out, const PlanEntry & entry)
  {
    std::vector<std::byte> raw(static_cast<std::size_t>(entry.index->chunkLength));
    if (!read_at(entry.start, raw.data(), raw.size())) {
      throw std::runtime_error("short read of a chunk record from " + input_.string());
    }
    auto decoded = detail::decompress_chunk_record(raw);
    if (!decoded.error.empty()) {
      return fail("chunk decode failed: " + decoded.error);
    }

    // Filter the decompressed records: keep surviving Message records
    // verbatim, strip embedded Schema/Channel records (the top-level block
    // re-declares the survivors, renames applied), reject anything else.
    std::vector<std::byte> blob;
    blob.reserve(decoded.records.size());
    std::map<mcap::ChannelId, std::vector<std::pair<mcap::Timestamp, mcap::ByteOffset>>> indexes;
    std::uint64_t kept = 0;
    std::uint64_t renamed = 0;
    std::uint64_t min_time = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t max_time = 0;

    const std::byte * data = decoded.records.data();
    const std::size_t size = decoded.records.size();
    std::size_t pos = 0;
    while (pos + kRecordHeaderBytes <= size) {
      const auto opcode = std::to_integer<std::uint8_t>(data[pos]);
      const std::uint64_t length = read_u64(data + pos + 1);
      if (length > size - pos - kRecordHeaderBytes) {
        return fail("chunk interior record overruns its decompressed blob");
      }
      const std::byte * body = data + pos + kRecordHeaderBytes;
      switch (static_cast<mcap::OpCode>(opcode)) {
        case mcap::OpCode::Message: {
          if (length < kMessagePrefixBytes) {
            return fail("malformed Message record inside a chunk");
          }
          const std::uint16_t channel_id = read_u16(body);
          const std::uint64_t log_time = read_u64(body + 6);
          if (dropped_ids_.count(channel_id) != 0 || log_time < start_ns_ || log_time >= end_ns_) {
            break;
          }
          indexes[channel_id].emplace_back(log_time, blob.size());
          blob.insert(blob.end(), data + pos, data + pos + kRecordHeaderBytes + length);
          ++kept;
          if (renamed_ids_.count(channel_id) != 0) {
            ++renamed;
          }
          min_time = std::min(min_time, log_time);
          max_time = std::max(max_time, log_time);
          break;
        }
        case mcap::OpCode::Schema:
          break;
        case mcap::OpCode::Channel: {
          if (length < 2) {
            return fail("malformed Channel record inside a chunk");
          }
          const std::uint16_t channel_id = read_u16(body);
          if (renamed_ids_.count(channel_id) != 0) {
            rename_seen_.insert(channel_id);
          }
          break;
        }
        default:
          return fail("unsupported record inside a chunk (opcode " + std::to_string(opcode) + ")");
      }
      pos += kRecordHeaderBytes + static_cast<std::size_t>(length);
    }
    if (pos != size) {
      return fail("chunk interior has trailing bytes");
    }

    if (kept == 0) {
      ++chunks_dropped_;
      return true;
    }

    const auto compressed = detail::compress_chunk_records(blob, entry.codec);

    mcap::Chunk chunk;
    chunk.messageStartTime = min_time;
    chunk.messageEndTime = max_time;
    chunk.uncompressedSize = blob.size();
    // bagwiz's writer skips chunk CRCs (see McapFileWriter); match it here.
    chunk.uncompressedCrc = 0;
    chunk.compression = entry.codec;
    chunk.compressedSize = compressed.size();
    chunk.records = compressed.data();

    const std::uint64_t chunk_start = out.size();
    mcap::McapWriter::write(out, chunk);
    const std::uint64_t index_start = out.size();

    mcap::ChunkIndex ci;
    ci.messageStartTime = min_time;
    ci.messageEndTime = max_time;
    ci.chunkStartOffset = chunk_start;
    ci.chunkLength = index_start - chunk_start;
    for (const auto & index_entry : indexes) {
      ci.messageIndexOffsets[index_entry.first] = out.size();
      mcap::MessageIndex mi;
      mi.channelId = index_entry.first;
      mi.records = index_entry.second;
      mcap::McapWriter::write(out, mi);
      channel_counts_[index_entry.first] += index_entry.second.size();
    }
    ci.messageIndexLength = out.size() - index_start;
    ci.compression = entry.codec;
    ci.compressedSize = compressed.size();
    ci.uncompressedSize = blob.size();
    out_chunk_indexes_.push_back(std::move(ci));

    message_count_ += kept;
    messages_renamed_ += renamed;
    min_time_ = std::min(min_time_, min_time);
    max_time_ = std::max(max_time_, max_time);
    note_written_codec(entry.codec);
    ++chunks_reencoded_;
    return true;
  }

  void note_written_codec(const std::string & codec)
  {
    const std::string normalized = codec == "none" ? "" : codec;
    if (!codec_noted_) {
      written_codec_ = normalized;
      codec_noted_ = true;
    } else if (written_codec_ != normalized) {
      written_codec_.clear();
    }
  }

  void write_summary_section(mcap::FileWriter & out)
  {
    const std::uint64_t summary_start = out.size();
    // The summary CRC covers everything from here through the footer's
    // summary_offset_start field; FileWriter accumulates it and the static
    // footer writer picks it up — the same recipe McapWriter::close() uses.
    out.crcEnabled = true;
    out.resetCrc();

    struct Group
    {
      mcap::OpCode opcode;
      std::uint64_t start;
      std::uint64_t length;
    };
    std::vector<Group> groups;

    if (!out_schemas_.empty()) {
      const std::uint64_t start = out.size();
      for (const auto & schema : out_schemas_) {
        mcap::Schema copy = schema;
        mcap::McapWriter::write(out, copy);
      }
      groups.push_back({mcap::OpCode::Schema, start, out.size() - start});
    }
    if (!out_channels_.empty()) {
      const std::uint64_t start = out.size();
      for (const auto & channel : out_channels_) {
        mcap::Channel copy = channel;
        mcap::McapWriter::write(out, copy);
      }
      groups.push_back({mcap::OpCode::Channel, start, out.size() - start});
    }
    if (!out_chunk_indexes_.empty()) {
      const std::uint64_t start = out.size();
      for (const auto & ci : out_chunk_indexes_) {
        mcap::McapWriter::write(out, ci);
      }
      groups.push_back({mcap::OpCode::ChunkIndex, start, out.size() - start});
    }
    {
      const std::uint64_t start = out.size();
      mcap::Statistics stats;
      stats.messageCount = message_count_;
      stats.schemaCount = static_cast<std::uint16_t>(out_schemas_.size());
      stats.channelCount = static_cast<std::uint32_t>(out_channels_.size());
      stats.attachmentCount = 0;
      stats.metadataCount = 0;
      stats.chunkCount = static_cast<std::uint32_t>(out_chunk_indexes_.size());
      stats.messageStartTime = message_count_ > 0 ? min_time_ : 0;
      stats.messageEndTime = message_count_ > 0 ? max_time_ : 0;
      for (const auto & entry : channel_counts_) {
        stats.channelMessageCounts[entry.first] = entry.second;
      }
      mcap::McapWriter::write(out, stats);
      groups.push_back({mcap::OpCode::Statistics, start, out.size() - start});
    }

    const std::uint64_t summary_offset_start = out.size();
    for (const auto & group : groups) {
      mcap::McapWriter::write(out, mcap::SummaryOffset{group.opcode, group.start, group.length});
    }
    mcap::McapWriter::write(
      out, mcap::Footer{summary_start, summary_offset_start}, /*crcEnabled=*/true);
    mcap::McapWriter::writeMagic(out);
  }

  // --- result ------------------------------------------------------------

  McapPassthroughResult build_result() const
  {
    McapPassthroughResult result;
    result.messages_written = message_count_;
    result.messages_renamed = messages_renamed_;
    result.chunks_copied = chunks_copied_;
    result.chunks_reencoded = chunks_reencoded_;
    result.chunks_dropped = chunks_dropped_;
    if (message_count_ > 0) {
      result.start_ns = static_cast<std::int64_t>(min_time_);
      result.end_ns = static_cast<std::int64_t>(max_time_);
    }
    result.chunk_compression = written_codec_;
    result.attachments_skipped = attachments_skipped_;
    result.metadata_skipped = metadata_skipped_;

    std::unordered_map<mcap::SchemaId, const mcap::Schema *> schema_of;
    for (const auto & schema : out_schemas_) {
      schema_of.emplace(schema.id, &schema);
    }
    for (const auto & channel : out_channels_) {
      TopicInfo info;
      info.name = channel.topic;
      info.serialization_format = channel.messageEncoding;
      if (auto it = schema_of.find(channel.schemaId);
          channel.schemaId != 0 && it != schema_of.end()) {
        const mcap::Schema & schema = *it->second;
        info.type = schema.name;
        info.schema_encoding = schema.encoding;
        if (!schema.data.empty()) {
          info.schema_text.assign(
            reinterpret_cast<const char *>(schema.data.data()), schema.data.size());
        }
      }
      if (auto qos = channel.metadata.find("offered_qos_profiles"); qos != channel.metadata.end()) {
        info.offered_qos_profiles = qos->second;
      }
      if (auto count = channel_counts_.find(channel.id); count != channel_counts_.end()) {
        result.per_topic_counts[channel.topic] = static_cast<std::int64_t>(count->second);
      }
      result.topics.push_back(std::move(info));
    }
    return result;
  }

  const std::filesystem::path & input_;
  const std::filesystem::path & output_;
  const McapPassthroughEdit & edit_;
  std::string reason_;

  mcap::McapReader reader_;
  std::ifstream file_;
  mcap::Header header_;
  std::unordered_map<mcap::ChannelId, mcap::ChannelPtr> channels_;
  std::unordered_map<mcap::SchemaId, mcap::SchemaPtr> schemas_;
  std::unordered_set<mcap::ChannelId> dropped_ids_;
  std::unordered_map<mcap::ChannelId, std::string> renamed_ids_;
  std::unordered_set<mcap::ChannelId> toplevel_channel_ids_;
  std::unordered_set<mcap::ChannelId> rename_seen_;
  std::uint64_t start_ns_ = 0;
  std::uint64_t end_ns_ = mcap::MaxTime;
  std::vector<PlanEntry> plan_;
  std::vector<std::byte> copy_slab_;

  std::vector<mcap::Schema> out_schemas_;
  std::vector<mcap::Channel> out_channels_;
  std::vector<mcap::ChunkIndex> out_chunk_indexes_;
  std::unordered_map<mcap::ChannelId, std::uint64_t> channel_counts_;
  std::uint64_t message_count_ = 0;
  std::uint64_t messages_renamed_ = 0;
  std::uint64_t chunks_copied_ = 0;
  std::uint64_t chunks_reencoded_ = 0;
  std::uint64_t chunks_dropped_ = 0;
  std::uint64_t attachments_skipped_ = 0;
  std::uint64_t metadata_skipped_ = 0;
  std::uint64_t min_time_ = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t max_time_ = 0;
  std::string written_codec_;
  bool codec_noted_ = false;
};

}  // namespace

std::optional<McapPassthroughResult> mcap_passthrough_rewrite(
  const std::filesystem::path & input, const std::filesystem::path & output,
  const McapPassthroughEdit & edit, std::string * fallback_reason)
{
  PassthroughEngine engine(input, output, edit);
  return engine.run(fallback_reason);
}

}  // namespace bagwiz::io
