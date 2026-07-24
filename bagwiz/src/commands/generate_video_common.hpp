// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__GENERATE_VIDEO_COMMON_HPP_
#define COMMANDS__GENERATE_VIDEO_COMMON_HPP_

#include "bagwiz/commands/generate_video.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/undistort.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

// Internals of `generate video`, split out of generate_video.cpp so the
// validation, pass-1 scan, tmp-file lifecycle, and frame pipeline units can be
// unit-tested without driving the full command. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// ---- input validation -------------------------------------------------------

// Outcome of validate_video_inputs(). `check` is the source-topic
// classification (its topic_type drives decoding); `camera_info_topic` is the
// validated / derived cam-info topic (nullopt when the run needs none or none
// could be derived). `error` is empty on success; on failure it holds the
// message that was already logged.
struct VideoInputValidation
{
  VideoSourceCheck check;
  std::optional<std::string> camera_info_topic;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// The command's pre-flight checks: source topic presence + renderable type,
// cam-info validation (explicit --cam-info) or derivation (from the image
// topic name), the cam-info requirement of --undistort / --pcd, and every
// point-cloud topic's presence + type. Logs the command's errors and returns
// on the first failure.
[[nodiscard]] VideoInputValidation validate_video_inputs(const GenerateVideoArgs & args);

// Fail-fast output checks run before the expensive encode: an existing
// `output_path` without --overwrite stops the run, and the output's parent
// directory is created when missing. Returns "" on success; on failure logs
// and returns the message.
[[nodiscard]] std::string validate_video_output_path(
  const std::filesystem::path & output_path, bool overwrite);

// ---- pass 1: frame-rate + point-cloud scan -----------------------------------

// Timestamps + count for a single topic, gathered by a payload-free scan.
struct TopicSpan
{
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
  std::uint64_t count = 0;
};

// Outcome of scan_video_inputs(). The pcd_spans entries are owned here; the
// encode loops move them out. `error` is empty on success; on failure it holds
// the message that was already logged.
struct VideoInputScan
{
  TopicSpan span;
  core::video::FrameRate fps;
  std::vector<core::pointcloud::PointCloudIndex> pcd_spans;
  // Per pcd topic: whether it can be matched by capture time (every cloud
  // carried a header.stamp). Topics that fall back to record time are matched
  // by record time on both sides so the overlay stays in one clock.
  std::vector<bool> pcd_topic_has_stamps;
  double global_property_min = 0.0;
  double global_property_max = 0.0;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Pass 1: derive the frame rate from the image topic's message timestamps and,
// when point-cloud overlay topics are given, scan each for its index and the
// selected property's global min/max. Logs the command's errors and returns
// with !ok() on the first failure.
[[nodiscard]] VideoInputScan scan_video_inputs(const GenerateVideoArgs & args);

// Threading is only worthwhile when there is enough work to hide the overhead
// of launching a thread and opening a fresh BagReader per frame.
[[nodiscard]] bool should_use_threaded_projection(
  bool has_pointcloud_topics, bool enable_threaded, std::uint64_t frame_count,
  unsigned int hardware_concurrency);

// ---- pass-2 geometry ---------------------------------------------------------

// The camera info (already scaled by --resize) and TF buffer the encode
// loop needs for --undistort / --pcd, loaded up front so a failure aborts
// before the encode. camera_info is set iff `camera_info_topic` was resolved;
// the TF buffer iff point-cloud topics are present. Filled via an out
// parameter because tf2::BufferCore is immobile (it owns a mutex), so this
// struct cannot be returned by value.
struct VideoGeometry
{
  std::optional<core::image::CameraInfo> camera_info;
  std::optional<tf2::BufferCore> tf_buffer;
};

// Load the pass-2 geometry into `out`: camera info from `camera_info_topic`
// when given, and the bag's TF when point-cloud overlay topics are present.
// Returns "" on success; on failure logs and returns the message.
[[nodiscard]] std::string load_video_geometry(
  const GenerateVideoArgs & args, const std::optional<std::string> & camera_info_topic,
  VideoGeometry & out);

// ---- partial tmp output -------------------------------------------------------

// The sibling temp path the video is encoded into before being moved into
// place, e.g. out.avi -> out.bagwiz-partial.avi. The real extension is kept on
// the temp file: both the encoder's codec choice and the libav muxer are
// selected from the extension, so a bare ".bagwiz-partial" suffix would be
// rejected.
[[nodiscard]] std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output);

// RAII owner of the partial tmp output's lifecycle: construction clears any
// stale tmp left by a previous aborted run; destruction removes the tmp when
// it still exists (a no-op once finalize_video_output renamed it away), so no
// error path can leave a partial file behind. Declare it BEFORE the encoder so
// the encoder is destroyed — closing the file — before the tmp is removed.
class PartialFileGuard
{
public:
  explicit PartialFileGuard(std::filesystem::path tmp_path);
  ~PartialFileGuard();
  PartialFileGuard(const PartialFileGuard &) = delete;
  PartialFileGuard & operator=(const PartialFileGuard &) = delete;
  PartialFileGuard(PartialFileGuard &&) = delete;
  PartialFileGuard & operator=(PartialFileGuard &&) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept { return tmp_path_; }

private:
  std::filesystem::path tmp_path_;
};

// Move the finished tmp video into place: apply the --overwrite clobber policy
// via core::prepare_output_path, then rename, falling back to copy + remove
// across filesystems. Returns "" on success; on failure logs and returns the
// message (the caller's PartialFileGuard removes the tmp).
[[nodiscard]] std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path,
  bool overwrite);

// ---- pass 2: frame pipeline ---------------------------------------------------

// Open the input bag for the encode pass, restricted to the image topic. Logs
// "failed to open ..." and returns nullptr on failure.
[[nodiscard]] std::unique_ptr<io::BagReader> open_encode_reader(const GenerateVideoArgs & args);

// Owned decode buffer that survives across BagReader::next() calls, which
// invalidate raw payload spans. Used by both the synchronous and the threaded
// point-cloud overlay paths.
struct FrameBuffer
{
  std::int64_t timestamp_ns = 0;     // bag record time
  std::int64_t header_stamp_ns = 0;  // image header.stamp (0 if unset)
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  core::video::SourcePixelFormat pixel_format = core::video::SourcePixelFormat::kBgr8;
  std::string encoding;
  std::vector<std::byte> data;
};

// Decode + resize half of the frame pipeline: normalizes each message (raw
// Image or CompressedImage) to a canonical packed BGR24 raster and scales it
// by the requested factor. Errors are logged with the count of frames written
// so far, matching the monolith's log lines.
class FrameNormalizer
{
public:
  FrameNormalizer(std::string topic_type, double resize_scale)
  : topic_type_(std::move(topic_type)), resize_scale_(resize_scale)
  {
  }

  // Decode one message payload into an owned canonical BGR24 frame.
  // `frame_index` is the count of frames written so far (used in the log line
  // on failure). Returns nullopt after logging when the payload does not
  // decode.
  [[nodiscard]] std::optional<FrameBuffer> decode(
    std::int64_t timestamp_ns, std::span<const std::byte> payload, std::uint64_t frame_index) const;

  // Resize a decoded frame in-place by the configured scale, preserving aspect
  // ratio. Returns false and logs when the result would be zero-size.
  [[nodiscard]] bool resize(FrameBuffer & frame) const;

private:
  std::string topic_type_;
  double resize_scale_;
};

// Encode half of the frame pipeline: owns the video encoder (opened lazily on
// the first frame, which fixes the run's geometry and pixel encoding), the
// undistort remap, and the point-cloud overlay state. All failures are logged
// and reported as false / a non-empty string, matching the monolith's
// log-then-abort shape.
class VideoFrameEncoder
{
public:
  // `camera_info` must be non-null when --undistort or --pcd is in play (input
  // validation guarantees a cam-info topic then). `overlay_min` /
  // `overlay_max` are the pass-1 global property range.
  VideoFrameEncoder(
    const std::filesystem::path & tmp_path, core::video::FrameRate fps,
    const GenerateVideoArgs & args, const core::image::CameraInfo * camera_info, double overlay_min,
    double overlay_max);

  // Encode one normalized frame; when `projected` is non-null its points are
  // overlaid first. Returns false after logging on any failure.
  [[nodiscard]] bool encode(
    FrameBuffer & frame, const std::vector<core::pointcloud::ProjectedPoint> * projected);

  // Flush and close the stream. Returns "" on success; on failure logs and
  // returns the message. Either way the encoder is closed afterwards (the tmp
  // file can be renamed or removed).
  [[nodiscard]] std::string finish();

  // True once the first frame opened the encoder. A finished run still reports
  // its geometry and frame count for the summary line.
  [[nodiscard]] bool started() const { return encoder_ != nullptr; }
  [[nodiscard]] std::uint64_t written() const { return written_; }
  [[nodiscard]] std::uint32_t width() const { return enc_w_; }
  [[nodiscard]] std::uint32_t height() const { return enc_h_; }

private:
  std::filesystem::path tmp_path_;
  core::video::FrameRate fps_;
  bool rectify_;  // --undistort or --pcd: feed frames through the undistort remap
  const core::image::CameraInfo * camera_info_;
  double overlay_min_;
  double overlay_max_;
  core::pointcloud::ColorScheme colorscheme_;
  std::uint32_t point_size_;
  float alpha_;

  std::unique_ptr<core::video::VideoEncoder> encoder_;
  std::unique_ptr<core::image::UndistortHelper> undistort_helper_;
  std::uint32_t enc_w_ = 0;
  std::uint32_t enc_h_ = 0;
  // to_packed_raster yields canonical BGR24, so every frame's encoding is
  // "bgr8"; the encoder still tracks it to guard against a mid-run change.
  std::string enc_encoding_;
  std::uint64_t written_ = 0;
};

// Synchronous encode loop: decode, optionally project, and encode
// frame-by-frame, keeping one cached fetcher per pcd topic so small bags or
// single-threaded runs do not pay the per-frame BagReader open/close cost.
// Moves the pcd index entries out of `scan`. `camera_info` / `tf_buffer` must
// be non-null when args.pointcloud_topics is non-empty. Returns a process exit
// code; errors are logged inside.
[[nodiscard]] int run_encode_loop_sync(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo * camera_info, tf2::BufferCore * tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder);

// Threaded encode loop: keeps one frame of projection work running ahead so
// fetch/parse/project for frame N+1 overlaps with encoding frame N. Only
// reached when point-cloud topics are present, so `camera_info` / `tf_buffer`
// are always set. Returns a process exit code; errors are logged inside.
[[nodiscard]] int run_encode_loop_async(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo & camera_info, tf2::BufferCore & tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder);

// Dispatch the encode pass: the threaded projection pipeline when it can pay
// for itself (should_use_threaded_projection), otherwise the synchronous loop.
// Both pointers must be non-null when point-cloud topics are present (the
// threaded path dereferences them). Reader/decoder exceptions are caught with
// the command's "error reading topic" log. Returns a process exit code.
[[nodiscard]] int run_encode_pass(
  io::BagReader & reader, const GenerateVideoArgs & args, VideoInputScan & scan,
  const core::image::CameraInfo * camera_info, tf2::BufferCore * tf_buffer,
  const FrameNormalizer & normalizer, VideoFrameEncoder & encoder);

// Close out the encode: require at least one rendered frame (pass 1 saw
// messages, so a frameless pass 2 means the bag changed between passes), flush
// + close the encoder, and move the tmp output into place. Returns "" on
// success; logs and returns the message on failure.
[[nodiscard]] std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite);

// ---- summary ------------------------------------------------------------------

// The end-of-run INFO line plus the H.264 playback guidance (with the VLC
// install hint when no vlc executable is on the host).
void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps);

}  // namespace bagwiz::commands

#endif  // COMMANDS__GENERATE_VIDEO_COMMON_HPP_
