// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/slam/cloud_filters.hpp"
#ifdef BAGWIZ_WITH_SLAM_CUDA
#include "bagwiz/core/slam/cloud_voxelize_gpu.hpp"
#endif
#include "bagwiz/core/slam/frame_feed_queue.hpp"
#include "bagwiz/core/slam/glim_estimator.hpp"
#include "bagwiz/core/slam/gnss_alignment.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/slam/scan_match_recovery.hpp"
#include "bagwiz/core/slam/warmup_recovery.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/mapping/callbacks.hpp>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/mapping/sub_mapping.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#ifdef BAGWIZ_WITH_SLAM_CUDA
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#endif

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PoseTranslationPrior.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
// Variance [m^2] for an "unconstrained" axis of a Gaussian prior: large enough
// that its information (1/variance) is negligible next to the LiDAR/IMU factors,
// emulating the fixed-precision path's zero z-information without an actual
// singular (infinite-variance) model.
constexpr double kUnconstrainedZVariance = 1e8;  // ~ (1e4 m)^2

// Max number of preprocessed scans buffered between the pipeline producer
// (preprocess) and consumer (odometry + mapping). The consumer is ~3x slower per
// scan, so a small buffer keeps it fed while bounding extra host memory (each
// buffered frame is the preprocessed cloud, tens of MB on a dense scan). IMU/GNSS
// events are weight-0 and do not count toward this bound.
constexpr std::size_t kFeedQueueCapacityScans = 8;

// Caps on the pre-init data buffered for warmup-window recovery (config.
// recover_start). Buffering runs until GLIM MARGINALIZES its first frame, not
// until init converges: that is IMU init (~1 s) plus the odometry smoother lag
// (~5 s), so ~6 s of data (~1k IMU samples, tens of scans) on a normal run. The
// post-boundary surplus is discarded at use time (recover_warmup filters on
// stamp < boundary; backpropagate_imu suffix-trims the rest). The caps sit far
// above that and only bound memory if init never converges (e.g. a fully static
// bag), where recovery is disabled rather than buffering unboundedly. IMU
// ~200 Hz -> 100k is >8 min.
constexpr std::size_t kWarmupMaxImuSamples = 100000;

// Caps on the trailing data buffered for cooldown-window recovery (config.
// recover_end). Unlike the warmup buffers, the cooldown boundary (the last frame
// to reach a finalized submap) is only known at end of sequence, so these are a
// SLIDING ring that keeps the most recent samples and drops the oldest past the
// cap — never disabling recovery, since a long bag is normal, not a failure. Only
// data past the boundary is used (recover_cooldown filters on stamp > boundary;
// forwardpropagate_imu prefix-trims the rest). The boundary trails the stream end
// by the odometry smoother lag PLUS the sub_mapping finalization delay — the same
// ~5 s order the warmup buffers span (see kWarmupMaxImuSamples above), NOT ~1 s —
// so the ring must retain several seconds; the caps sit far above that (40k IMU at
// ~200 Hz is >3 min), bounding memory at ~2 MB regardless of bag length.
constexpr std::size_t kCooldownMaxImuSamples = 40000;
// Recent marginalized-frame kinematic states retained for the cooldown anchor,
// keyed by id. The anchor is the newest frame in a finalized submap; the only
// frames marginalized after it are the trailing ones sub_mapping never finalized
// (a submap's worth at most — tens of frames), so the anchor sits within the last
// few entries here and 256 is far more than that gap needs. If the cap were ever
// exceeded, recover_cooldown no-ops safely: the anchor state is simply not found.
constexpr std::size_t kCooldownMaxFrameStates = 256;

// Scan-matching recovery. Window scans keep their preprocessed LiDAR-frame
// points so recover_*() can register them against the optimized map (not just
// their stamps). Warmup buffers until GLIM's first frame (bounded like its stamp
// buffer, overflow disables recovery); cooldown keeps a trailing ring. A cloud
// is a downsampled preprocessed scan (~10-20k points, sub-MB), so these caps
// bound the buffers to well under a few hundred MB on any bag.
constexpr std::size_t kWarmupMaxScanClouds = 512;
constexpr std::size_t kCooldownMaxScanClouds = 256;
// Boundary-neighboring optimized frames used to seed the scan-match target (the
// first N by stamp for warmup, the last N for cooldown). A short recovery window
// at low speed is well covered by a handful of frames; the accepted recovery
// scans are then folded into the target so it grows along the chain.
constexpr std::size_t kRecoverySeedFrames = 12;

core::TrajectoryPose to_pose(double stamp, const Eigen::Isometry3d & transform)
{
  const Eigen::Vector3d translation = transform.translation();
  Eigen::Quaterniond rotation(transform.rotation());
  rotation.normalize();

  core::TrajectoryPose pose;
  pose.timestamp_ns = static_cast<std::int64_t>(std::llround(stamp * 1e9));
  pose.tx = translation.x();
  pose.ty = translation.y();
  pose.tz = translation.z();
  pose.qx = rotation.x();
  pose.qy = rotation.y();
  pose.qz = rotation.z();
  pose.qw = rotation.w();
  return pose;
}

// Copy a preprocessed frame's LiDAR-frame points to plain double xyz, the source
// cloud the scan-matching recovery registers against the optimized map. The
// preprocessed cloud is already downsampled, so this is a cheap sub-MB copy.
std::vector<Eigen::Vector3d> preprocessed_xyz(const glim::PreprocessedFrame::Ptr & frame)
{
  std::vector<Eigen::Vector3d> pts;
  if (!frame) {
    return pts;
  }
  pts.reserve(frame->points.size());
  for (const auto & p : frame->points) {
    pts.emplace_back(p.x(), p.y(), p.z());
  }
  return pts;
}

// IMU in the mapping stages is enabled iff we run the LiDAR-IMU backend (an
// extrinsic was provided and insert_imu is fed). In LiDAR-only mode it must stay
// off — GLIM's defaults enable it, which would make insert_submap build IMU
// factors from uninitialised bias/velocity and warn that frames are not
// IMU-framed. Everything else keeps GLIM's stock defaults: the exported map's
// density is controlled separately, by re-binning the optimized per-frame points
// (see CloudMapper::Impl::fill_map), so the sub mapping that drives the
// optimization is left untouched.
// use_gpu switches the sub/global registration factor to gtsam_points' GPU VGICP
// (built on a CUDA voxelmap). enable_gpu lets GLIM allocate the GPU stream/buffers.
// Both fall back to GLIM's stock CPU VGICP when use_gpu is false, so the CPU path
// is byte-for-byte unchanged.
glim::SubMappingParams make_sub_mapping_params(bool enable_imu, bool use_gpu)
{
  glim::SubMappingParams params;
  params.enable_imu = enable_imu;
  if (use_gpu) {
    params.enable_gpu = true;
    params.registration_error_factor_type = "VGICP_GPU";
  }
  return params;
}
glim::GlobalMappingParams make_global_mapping_params(bool enable_imu, bool use_gpu)
{
  glim::GlobalMappingParams params;
  params.enable_imu = enable_imu;
  if (use_gpu) {
    params.enable_gpu = true;
    params.registration_error_factor_type = "VGICP_GPU";
  }
  return params;
}

#ifdef BAGWIZ_WITH_SLAM_CUDA
// GLIM's GPU registration factors (VGICP_GPU — used by the --gpu odometry smoother
// and by GPU sub/global mapping) only get batched, asynchronous GPU linearization
// when a NonlinearFactorSetGPU is registered on gtsam_points' process-global
// LinearizationHook. GLIM registers it inside its own executables (offline_viewer,
// ROS nodes), NOT in the library modules we construct in-process — so we must do
// it ourselves, exactly once, before the first GLIM module is built (the odometry
// smoother creates an Ext optimizer in its constructor, and that snapshots the
// hook list at construction time). Without it every VGICP_GPU factor falls back to
// per-factor *synchronous* GPU linearization ("performing linearization in sync
// mode seriously affects the processing speed!!") and the GPU path runs far slower
// than intended. call_once keeps repeated CloudMapper constructions (e.g. across
// tests in one process) from stacking duplicate hooks.
void register_gpu_linearization_hook_once()
{
  static std::once_flag flag;
  std::call_once(flag, [] {
    gtsam_points::LinearizationHook::register_hook(
      [] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
  });
}
#endif

// Build preprocessor params. A non-positive num_threads falls back to the
// default (4) so both stages share the same baseline; otherwise they share
// the requested thread budget because they run sequentially. input_resolution
// and the range crop are written onto GLIM's stock preprocess fields (which
// bagwiz otherwise leaves at their defaults, running GLIM with no config dir).
glim::CloudPreprocessorParams make_preprocessor_params(const CloudMapperConfig & cfg)
{
  glim::CloudPreprocessorParams params;
  params.num_threads = cfg.num_threads > 0 ? cfg.num_threads : 4;
  params.downsample_resolution = cfg.input_resolution;
  params.distance_near_thresh = cfg.range_min;
  params.distance_far_thresh = cfg.range_max;
  return params;
}

// Build the scan-match recovery params. Only the accept/reject gate is
// user-exposed (via CloudMapperConfig); every other field stays at
// ScanMatchParams' loose-init default.
ScanMatchParams make_recovery_params(const CloudMapperConfig & cfg)
{
  ScanMatchParams params;
  params.min_inlier_fraction = cfg.recovery_min_inlier_fraction;
  return params;
}

// Per-frame point geometry held in the stash. Float by default (the CPU export
// stays byte-identical to the historical output). In use_gpu mode it is
// int16-quantized about the frame's own centroid (Tier-1c), roughly halving the
// host stash held across the whole run on a large bag; the quantization error
// (< ~1 mm at LiDAR range) is far below input_resolution and the GPU path is
// outside the reproducibility guarantee. Exactly one of `f` / `q` is populated.
struct FramePoints
{
  std::vector<std::array<float, 3>> f;         // float, populated when !use_gpu
  std::vector<std::array<std::int16_t, 3>> q;  // int16, populated when use_gpu
  std::array<float, 3> center{0.0F, 0.0F, 0.0F};
  float scale = 1.0F;

  [[nodiscard]] std::size_t size() const { return q.empty() ? f.size() : q.size(); }
  [[nodiscard]] bool empty() const { return f.empty() && q.empty(); }

  // Dequantized LiDAR-frame point i as float xyz (identity for the float path).
  [[nodiscard]] std::array<float, 3> at(std::size_t i) const
  {
    if (q.empty()) {
      return f[i];
    }
    return {
      center[0] + static_cast<float>(q[i][0]) * scale,
      center[1] + static_cast<float>(q[i][1]) * scale,
      center[2] + static_cast<float>(q[i][2]) * scale};
  }
};

// Build a FramePoints from float LiDAR-frame xyz. !use_gpu keeps the floats
// verbatim (CPU export byte-identical). use_gpu int16-quantizes about the cloud
// centroid: center = (min+max)/2, scale = max axis half-extent / 32767.
FramePoints make_frame_points(std::vector<std::array<float, 3>> && pts, bool use_gpu)
{
  FramePoints fp;
  if (!use_gpu || pts.empty()) {
    fp.f = std::move(pts);
    return fp;
  }
  std::array<float, 3> lo = pts[0];
  std::array<float, 3> hi = pts[0];
  for (const auto & p : pts) {
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], p[a]);
      hi[a] = std::max(hi[a], p[a]);
    }
  }
  float half_extent = 0.0F;
  for (int a = 0; a < 3; ++a) {
    fp.center[a] = 0.5F * (lo[a] + hi[a]);
    half_extent = std::max(half_extent, 0.5F * (hi[a] - lo[a]));
  }
  // 32767 = int16 max; guard a degenerate (zero-extent / single-point) frame.
  fp.scale = half_extent > 0.0F ? half_extent / 32767.0F : 1.0F;
  const float inv_scale = 1.0F / fp.scale;
  fp.q.reserve(pts.size());
  for (const auto & p : pts) {
    std::array<std::int16_t, 3> q{};
    for (int a = 0; a < 3; ++a) {
      const std::int64_t r =
        std::clamp<std::int64_t>(std::llround((p[a] - fp.center[a]) * inv_scale), -32767, 32767);
      q[a] = static_cast<std::int16_t>(r);
    }
    fp.q.push_back(q);
  }
  return fp;
}

}  // namespace

struct CloudMapper::Impl
{
  // Full points of one odometry frame, captured at insert() while GLIM still
  // holds them. Sub mapping subsamples keyframes and drops per-frame points to
  // save memory, so these full LiDAR-frame points (the only ones dense enough to
  // build a high-resolution map) must be copied out the moment they arrive.
  // Keyed by EstimationFrame::id so they can be paired with the frame's
  // optimized submap-relative pose at capture time.
  struct StashedPoints
  {
    FramePoints points;              // LiDAR-frame coordinates (float, or int16 in use_gpu)
    std::vector<float> intensities;  // empty unless the scan had intensities
  };

  // One frame of a submap, captured BEFORE the submap is inserted into global
  // mapping (which overwrites SubMap::T_world_origin with the chained global
  // estimate). T_origin_frame is the frame's pose relative to the submap origin
  // and is invariant under global optimization; the optimized world pose is
  // recovered later as (optimized T_world_origin) * T_origin_frame. The points
  // are this frame's full LiDAR-frame cloud, moved in from the stash.
  struct FrameRef
  {
    std::int64_t id = 0;
    double stamp = 0.0;
    Eigen::Isometry3d T_origin_frame = Eigen::Isometry3d::Identity();
    FramePoints points;              // LiDAR-frame, full density (float, or int16 in use_gpu)
    std::vector<float> intensities;  // parallel to points; may be empty
  };
  struct SubMapEntry
  {
    glim::SubMap::Ptr submap;  // kept so its T_world_origin can be read post-optimize
    std::vector<FrameRef> frames;
  };

  const CloudMapperConfig config;  // Con.4: set once at construction, never mutated
  glim::TimeKeeper time_keeper;
  glim::CloudPreprocessor preprocessor;
  // CT (LiDAR-only) or CPU (LiDAR-IMU) behind the common base interface.
  std::unique_ptr<glim::OdometryEstimationBase> odometry;
  std::unique_ptr<glim::SubMapping> sub_mapping;
  std::unique_ptr<glim::GlobalMapping> global_mapping;
  std::vector<SubMapEntry> entries;
  std::unordered_map<std::int64_t, StashedPoints> stash;  // frame id -> full points

  // One buffered window scan retained for scan-matching recovery: its stamp and
  // its preprocessed LiDAR-frame points (the source cloud register_scan aligns
  // to the optimized map). Points are double xyz, copied from the preprocessed
  // frame before odometry consumes it.
  struct WindowScan
  {
    double stamp = 0.0;
    std::vector<Eigen::Vector3d> points;
  };

  // --- Warmup-window recovery (config.recover_start) --------------------------
  // The scans captured before GLIM's first estimation frame get no odometry
  // pose. recover_warmup() recovers them by scan-matching each against the
  // optimized map (see scan_match_recovery.hpp); when a LiDAR-IMU extrinsic is
  // present the buffered IMU additionally seeds each initial guess and provides a
  // fallback if a registration fails its gate. Buffered on the single
  // odometry-executing thread (consume_* serial, odometry_loop pipeline) and
  // consumed by recover_warmup() in finish() only after the workers join, so no
  // synchronization is needed.
  struct WarmupState
  {
    bool active = false;             // recover_start (IMU no longer required)
    bool boundary_captured = false;  // first frame seen -> `boundary` is set
    bool overflowed = false;         // window exceeded the cap before init
    std::vector<BackpropImu> imu;    // raw (pre-bias), ascending, pre-boundary
    std::vector<WindowScan> scans;   // pre-boundary window scans (stamp + points)
    BackpropBoundary boundary;       // converged state at the first frame (IMU)
    std::int64_t boundary_id = -1;   // that frame's EstimationFrame::id
  };
  WarmupState warmup;

  // --- Cooldown-window recovery (config.recover_end) --------------------------
  // The symmetric counterpart of `warmup`: at end-of-sequence the newest scans
  // are still inside the odometry smoother window and never get marginalized into
  // a finalized submap, so the trajectory stops one window short of the last
  // input scan. This buffers a trailing ring of raw IMU + scan stamps and the
  // converged state of the LATEST marginalized frame (the boundary), so
  // recover_cooldown() can integrate the IMU FORWARD from that boundary to
  // recover per-scan poses for the trailing scans. Buffered on the same single
  // odometry-executing thread as `warmup`; consumed in finish() after the workers
  // join. Unlike `warmup`, the boundary is only known at the end (it is the newest
  // frame, not the first), so the buffers are a sliding ring that keeps the most
  // recent samples rather than giving up on overflow.
  struct CooldownState
  {
    bool active = false;           // recover_end (IMU no longer required)
    std::deque<BackpropImu> imu;   // raw (pre-bias), ascending, trailing ring
    std::deque<WindowScan> scans;  // trailing-ring window scans (stamp + points)
    // Converged state of recent marginalized frames, keyed by EstimationFrame::id
    // in insertion (ascending-stamp) order, kept as a trailing ring.
    // recover_cooldown() anchors on the LAST frame that reached a finalized submap
    // (the max-stamp frame in `entries`) -- NOT necessarily the last frame
    // marginalized, since GLIM may drop the newest from its end-of-sequence submap
    // -- so the anchor's kinematic state is looked up here by id rather than
    // latched onto a single frame.
    std::deque<std::pair<std::int64_t, BackpropBoundary>> frame_states;
  };
  CooldownState cooldown;

  // One GNSS fix in the local metric frame, with its stamp in seconds (matching
  // EstimationFrame::stamp) so it can be interpolated against submap timestamps.
  struct GnssMetric
  {
    double stamp = 0.0;
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    std::array<double, 9> covariance{};  // local ENU, m^2, row-major
    std::uint8_t covariance_type = 0;    // sensor_msgs/NavSatFix: 0 UNKNOWN .. 3 KNOWN
  };
  // GNSS fixes collected via insert_gnss (config.enable_gnss only), consumed by
  // build_gnss_factors() in finish().
  std::vector<GnssMetric> gnss_points;
  // GNSS translation-prior factors built in finish() and injected into the
  // global factor graph via the on_smoother_update callback during optimize().
  std::vector<gtsam::NonlinearFactor::shared_ptr> gnss_factors;

  // --- Feed pipeline (active unless config.disable_pipeline) -----------------
  // A 3-stage producer/consumer so the CPU preprocess (T1), odometry (T2), and
  // sub/global mapping (T3) overlap on separate threads. Each GLIM module stays
  // synchronous and is owned by exactly one thread; the two bounded FIFO queues
  // preserve bag order, so every module sees the same input sequence as the serial
  // path (CT output is bit-identical at num_threads=1; IMU/GPU/multithread are
  // tolerance-only, as for the serial multithreaded path). Stage layout:
  //   T1 producer (caller thread): build+TimeKeeper+preprocess -> odom_queue
  //   T2 odometry thread: odometry->insert_frame -> marginalized frames -> map_queue
  //   T3 mapping thread:  stash + sub_mapping->insert_frame + drain + global insert
  // IMU/GNSS events ride BOTH queues so each stage applies them in bag order.

  // Q1 event (producer -> odometry): a preprocessed scan, an IMU sample, or a GNSS fix.
  struct FeedEvent
  {
    enum class Kind { Scan, Imu, Gnss };
    Kind kind = Kind::Scan;
    glim::PreprocessedFrame::Ptr frame;  // Scan
    double imu_stamp = 0.0;              // Imu (seconds)
    Eigen::Vector3d imu_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro = Eigen::Vector3d::Zero();
    GnssPoint gnss;  // Gnss
  };

  // Q2 event (odometry -> mapping): a marginalized odometry frame, or the same
  // IMU/GNSS forwarded so the mapping stage applies them in order.
  struct MapEvent
  {
    enum class Kind { Frame, Imu, Gnss };
    Kind kind = Kind::Frame;
    glim::EstimationFrame::ConstPtr frame;  // Frame (marginalized)
    double imu_stamp = 0.0;
    Eigen::Vector3d imu_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_gyro = Eigen::Vector3d::Zero();
    GnssPoint gnss;
  };

  FrameFeedQueue<FeedEvent> odom_queue{kFeedQueueCapacityScans};
  FrameFeedQueue<MapEvent> map_queue{kFeedQueueCapacityScans};
  std::thread odometry_thread;
  std::thread mapping_thread;
  bool pipeline_started = false;
  // One process-wide std::cout mute spanning the whole streaming feed. GLIM's
  // LiDAR-IMU init dumps an LM table to std::cout from odometry->insert_frame (on
  // T2); muting from a single (main) thread avoids the cross-thread rdbuf-swap race
  // that per-thread guards would cause. Released in finish() after the workers join.
  std::unique_ptr<detail::ScopedCoutSilence> feed_silence;

  ~Impl()
  {
    // If feeding was interrupted (finish() never called, or an exception) the
    // workers may be blocked on pop()/push(); cancel both queues + join so a
    // joinable thread is never destroyed (which would std::terminate). Members
    // destruct after this body, so the threads stop touching the GLIM modules first.
    map_queue.cancel();
    odom_queue.cancel();
    if (odometry_thread.joinable()) {
      odometry_thread.join();
    }
    if (mapping_thread.joinable()) {
      mapping_thread.join();
    }
  }

  // Build the GLIM RawPoints for one scan and run TimeKeeper + preprocess. Returns
  // null when TimeKeeper rejects the scan (same drop as the serial path). Runs on
  // the producer (caller) thread.
  glim::PreprocessedFrame::Ptr prepare(const LidarScan & scan)
  {
    auto raw = std::make_shared<glim::RawPoints>();
    raw->stamp = static_cast<double>(scan.stamp_ns) * 1e-9;

    const std::size_t num_points = scan.points.size();
    raw->points.reserve(num_points);
    for (const auto & point : scan.points) {
      raw->points.emplace_back(point[0], point[1], point[2], 1.0);
    }
    if (!scan.intensities.empty()) {
      raw->intensities = scan.intensities;
    }

    // A time-less cloud is fed explicit zero per-point times (already motion-
    // undistorted), NOT an empty vector — that would make glim::TimeKeeper
    // synthesize order-based pseudo times and wrongly "deskew" a concatenated cloud.
    if (scan.has_per_point_time && scan.times.size() == num_points) {
      raw->times = scan.times;
    } else {
      raw->times.assign(num_points, 0.0);
    }

    if (!time_keeper.process(raw)) {
      return nullptr;
    }
    return preprocessor.preprocess(raw);
  }

  // Consumer side of one preprocessed scan: odometry -> stash+sub -> drain. The
  // active-frame return is intentionally ignored (the trajectory comes from the
  // globally-optimized submap poses in finish()); only marginalized frames feed
  // sub mapping. Identical to the serial path's odometry-onward body.
  void consume_scan(const glim::PreprocessedFrame::Ptr & preprocessed)
  {
    warmup_note_scan(preprocessed);
    cooldown_note_scan(preprocessed);
    std::vector<glim::EstimationFrame::ConstPtr> marginalized;
    odometry->insert_frame(preprocessed, marginalized);
    for (const auto & frame : marginalized) {
      warmup_note_frame(frame);
      cooldown_note_frame(frame);
      feed_sub_mapping(frame);
    }
    drain_submaps();
  }

  // Consumer side of one IMU sample: route to all three stages (no-ops in
  // LiDAR-only mode), each of which buffers it in its own preintegrator.
  void consume_imu(double stamp, const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro)
  {
    warmup_note_imu(stamp, acc, gyro);
    cooldown_note_imu(stamp, acc, gyro);
    odometry->insert_imu(stamp, acc, gyro);
    sub_mapping->insert_imu(stamp, acc, gyro);
    global_mapping->insert_imu(stamp, acc, gyro);
  }

  // T2 body: pop odom_queue, run odometry, and forward the marginalized frames +
  // IMU/GNSS to map_queue in order, then close map_queue. The smoother's *remaining*
  // (in-window) frames are NOT flushed here — finish() flushes them via
  // get_remaining_frames() on the main thread after the joins, exactly as the serial
  // path does; flushing them here too would double-feed sub mapping. A GLIM error (or
  // a downstream failure) is latched into both queues so the producer / finish()
  // rethrows it.
  void odometry_loop()
  {
    try {
      FeedEvent event;
      while (odom_queue.pop(event)) {
        switch (event.kind) {
          case FeedEvent::Kind::Scan: {
            warmup_note_scan(event.frame);
            cooldown_note_scan(event.frame);
            std::vector<glim::EstimationFrame::ConstPtr> marginalized;
            odometry->insert_frame(event.frame, marginalized);
            for (auto & frame : marginalized) {
              warmup_note_frame(frame);
              cooldown_note_frame(frame);
              MapEvent out;
              out.kind = MapEvent::Kind::Frame;
              out.frame = std::move(frame);
              if (!forward_to_map(std::move(out), 1)) {
                return;
              }
            }
            break;
          }
          case FeedEvent::Kind::Imu: {
            warmup_note_imu(event.imu_stamp, event.imu_acc, event.imu_gyro);
            cooldown_note_imu(event.imu_stamp, event.imu_acc, event.imu_gyro);
            odometry->insert_imu(event.imu_stamp, event.imu_acc, event.imu_gyro);
            MapEvent out;
            out.kind = MapEvent::Kind::Imu;
            out.imu_stamp = event.imu_stamp;
            out.imu_acc = event.imu_acc;
            out.imu_gyro = event.imu_gyro;
            if (!forward_to_map(std::move(out), 0)) {
              return;
            }
            break;
          }
          case FeedEvent::Kind::Gnss: {
            MapEvent out;
            out.kind = MapEvent::Kind::Gnss;
            out.gnss = event.gnss;
            if (!forward_to_map(std::move(out), 0)) {
              return;
            }
            break;
          }
        }
      }
    } catch (...) {
      const auto error = std::current_exception();
      odom_queue.fail(error);  // unblock the producer (T1)
      map_queue.fail(error);   // unblock the mapping thread (T3)
      return;
    }
    map_queue.close();  // normal end: T3 drains the rest and exits
  }

  // Push one event to map_queue. Returns false when the mapping thread is gone
  // (failed or cancelled); on a real downstream error, surface it onto odom_queue so
  // the producer (T1) unblocks and finish() rethrows it.
  bool forward_to_map(MapEvent event, std::size_t weight)
  {
    if (map_queue.push(std::move(event), weight)) {
      return true;
    }
    if (auto error = map_queue.error()) {
      odom_queue.fail(error);
    }
    return false;
  }

  // T3 body: pop map_queue and run the mapping stages in order. feed_sub_mapping
  // stashes the frame's points before sub mapping can drop them, and drain_submaps
  // captures each completed submap's relative poses before global mapping overwrites
  // them — the same ordering as the serial path.
  void mapping_loop()
  {
    try {
      MapEvent event;
      while (map_queue.pop(event)) {
        switch (event.kind) {
          case MapEvent::Kind::Frame:
            feed_sub_mapping(event.frame);
            drain_submaps();
            break;
          case MapEvent::Kind::Imu:
            sub_mapping->insert_imu(event.imu_stamp, event.imu_acc, event.imu_gyro);
            global_mapping->insert_imu(event.imu_stamp, event.imu_acc, event.imu_gyro);
            break;
          case MapEvent::Kind::Gnss:
            add_gnss(event.gnss);
            break;
        }
      }
    } catch (...) {
      // Latch so T2's forward_to_map fails -> T2 fails odom_queue -> T1 unblocks.
      map_queue.fail(std::current_exception());
    }
  }

  // Start the pipeline (both worker threads + the streaming cout mute) on first use.
  // insert/insert_imu/insert_gnss are all called from the single producer (bag-read)
  // thread, so no lock is needed here.
  void start_pipeline_if_needed()
  {
    if (!pipeline_started) {
      pipeline_started = true;
      feed_silence = std::make_unique<detail::ScopedCoutSilence>();
      odometry_thread = std::thread([this] { odometry_loop(); });
      mapping_thread = std::thread([this] { mapping_loop(); });
    }
  }

  // Enqueue one event for the odometry stage (weight 1 for a scan to bound buffered
  // scans, 0 for tiny IMU/GNSS). If a worker has died, rethrow its latched exception
  // on the producer thread.
  void enqueue_odom(FeedEvent event, std::size_t weight)
  {
    start_pipeline_if_needed();
    if (!odom_queue.push(std::move(event), weight)) {
      if (auto error = odom_queue.error()) {
        std::rethrow_exception(error);
      }
    }
  }

  explicit Impl(const CloudMapperConfig & cfg)
  : config(cfg),
    preprocessor(make_preprocessor_params(cfg)),
    odometry(detail::make_odometry_estimator(cfg.t_lidar_imu, cfg.num_threads, cfg.use_gpu)),
    sub_mapping(
      std::make_unique<glim::SubMapping>(
        make_sub_mapping_params(cfg.t_lidar_imu.has_value(), cfg.use_gpu))),
    global_mapping(
      std::make_unique<glim::GlobalMapping>(
        make_global_mapping_params(cfg.t_lidar_imu.has_value(), cfg.use_gpu)))
  {
    // Warmup / cooldown recovery scan-matches the window scans against the
    // optimized map, so it runs in LiDAR-only mode too; a LiDAR-IMU extrinsic only
    // adds the IMU init/fallback path inside recover_*(). Gated solely on the
    // recover_start / recover_end toggles.
    warmup.active = cfg.recover_start;
    cooldown.active = cfg.recover_end;
  }

  // Copy a frame's full LiDAR-frame points (and intensities, if any) out of GLIM
  // before sub mapping drops them, keyed by id.
  void stash_frame(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!frame || !frame->frame || frame->frame->size() == 0) {
      return;
    }
    const auto & cloud = frame->frame;
    const std::size_t n = cloud->size();

    // GLIM stores each estimation frame's points in frame->frame_id coordinates,
    // and that frame DIFFERS between backends: the LiDAR frame for the CT
    // (LiDAR-only) backend, but the IMU frame for the CPU (LiDAR-IMU) backend
    // (GLIM builds it as points_imu = T_imu_lidar * points_lidar). The map and
    // trajectory downstream both place these points with T_world_lidar, so bring
    // every frame's points back into the LiDAR frame first. T_lidar_sensor is
    // identity for the CT backend (frame_id == LIDAR, points already LiDAR-frame)
    // and equals T_lidar_imu for the IMU backend (frame_id == IMU); GLIM's own
    // T_world_sensor() selects T_world_lidar / T_world_imu by frame_id. Skipping
    // this corrupts the whole map whenever the IMU<-LiDAR extrinsic is not
    // identity (e.g. a 180-deg-flipped IMU), placing every point off by exactly
    // that extrinsic.
    const Eigen::Isometry3d T_lidar_sensor =
      frame->T_world_lidar.inverse() * frame->T_world_sensor();

    std::vector<std::array<float, 3>> pts;
    pts.reserve(n);
    StashedPoints stashed;
    // Intensities are sourced from the preprocessed input frame, NOT from
    // cloud->intensities. GLIM's LiDAR-only backend (OdometryEstimationCT) never
    // copies intensities onto its estimation-frame cloud, so cloud->has_intensities()
    // is false there; the LiDAR-IMU backend does copy them. raw_frame->intensities
    // is populated by the preprocessor in BOTH modes and is index-aligned 1:1 with
    // cloud->points (the estimation cloud is built from the same preprocessed points,
    // only deskewed — deskewing preserves count and order), so it pairs correctly
    // with the geometry read from cloud->points below.
    const bool has_intensities = frame->raw_frame && frame->raw_frame->intensities.size() == n;
    if (has_intensities) {
      stashed.intensities.reserve(n);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d p = T_lidar_sensor * cloud->points[i].head<3>();
      pts.push_back(
        {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
      if (has_intensities) {
        stashed.intensities.push_back(static_cast<float>(frame->raw_frame->intensities[i]));
      }
    }
    stashed.points = make_frame_points(std::move(pts), config.use_gpu);
    stash[frame->id] = std::move(stashed);
  }

  // Stash the frame's full points, then hand it to sub mapping.
  void feed_sub_mapping(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!frame) {
      return;
    }
    stash_frame(frame);
    sub_mapping->insert_frame(frame);
  }

  // ---- Warmup-window recovery buffering (odometry-executing thread only) -----
  // Buffer one raw IMU sample until the first frame is seen. Disables recovery
  // (releasing the buffers) if the pre-init window overflows the cap.
  void warmup_note_imu(double stamp, const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro)
  {
    if (!warmup.active || warmup.boundary_captured) {
      return;
    }
    if (warmup.imu.size() >= kWarmupMaxImuSamples) {
      warmup_disable();
      return;
    }
    warmup.imu.push_back({stamp, acc, gyro});
  }

  // Buffer one pre-init window scan (stamp + LiDAR-frame points). See
  // warmup_note_imu.
  void warmup_note_scan(const glim::PreprocessedFrame::Ptr & frame)
  {
    if (!warmup.active || warmup.boundary_captured || !frame) {
      return;
    }
    if (warmup.scans.size() >= kWarmupMaxScanClouds) {
      warmup_disable();
      return;
    }
    warmup.scans.push_back({frame->stamp, preprocessed_xyz(frame)});
  }

  // Capture the boundary state off GLIM's first estimation frame (id 0): its
  // converged world pose, world velocity, and IMU biases. Called for every
  // marginalized frame but latches on the first.
  void warmup_note_frame(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!warmup.active || warmup.boundary_captured || !frame) {
      return;
    }
    warmup.boundary.stamp = frame->stamp;
    warmup.boundary.T_world_imu = frame->T_world_imu;
    warmup.boundary.v_world_imu = frame->v_world_imu;
    warmup.boundary.acc_bias = frame->imu_bias.head<3>();
    warmup.boundary.gyro_bias = frame->imu_bias.tail<3>();
    warmup.boundary_id = frame->id;
    warmup.boundary_captured = true;
  }

  // Give up on recovery and release the buffers (init never converged within the
  // cap). boundary_captured stays false so recover_warmup() no-ops.
  void warmup_disable()
  {
    warmup.active = false;
    warmup.overflowed = true;
    warmup.imu.clear();
    warmup.imu.shrink_to_fit();
    warmup.scans.clear();
    warmup.scans.shrink_to_fit();
  }

  // ---- Cooldown-window recovery buffering (odometry-executing thread only) ---
  // Buffer one raw IMU sample in the trailing ring, dropping the oldest past the
  // cap so memory stays bounded on a long bag (the boundary is near the stream
  // end, so only the most recent samples are ever needed).
  void cooldown_note_imu(double stamp, const Eigen::Vector3d & acc, const Eigen::Vector3d & gyro)
  {
    if (!cooldown.active) {
      return;
    }
    cooldown.imu.push_back({stamp, acc, gyro});
    if (cooldown.imu.size() > kCooldownMaxImuSamples) {
      cooldown.imu.pop_front();
    }
  }

  // Buffer one window scan (stamp + LiDAR-frame points) in the trailing ring.
  // See cooldown_note_imu.
  void cooldown_note_scan(const glim::PreprocessedFrame::Ptr & frame)
  {
    if (!cooldown.active || !frame) {
      return;
    }
    cooldown.scans.push_back({frame->stamp, preprocessed_xyz(frame)});
    if (cooldown.scans.size() > kCooldownMaxScanClouds) {
      cooldown.scans.pop_front();
    }
  }

  // Record one marginalized frame's converged state (world pose, world velocity,
  // IMU biases) in the trailing ring, keyed by id. Called for every marginalized
  // frame; recover_cooldown() later picks whichever of these is the newest frame
  // to reach a finalized submap as its anchor (which may not be the very last one
  // recorded, if GLIM drops the newest from its end-of-sequence submap).
  void cooldown_note_frame(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!cooldown.active || !frame) {
      return;
    }
    BackpropBoundary state;
    state.stamp = frame->stamp;
    state.T_world_imu = frame->T_world_imu;
    state.v_world_imu = frame->v_world_imu;
    state.acc_bias = frame->imu_bias.head<3>();
    state.gyro_bias = frame->imu_bias.tail<3>();
    cooldown.frame_states.emplace_back(frame->id, state);
    if (cooldown.frame_states.size() > kCooldownMaxFrameStates) {
      cooldown.frame_states.pop_front();
    }
  }

  // Capture each frame's submap-local relative pose and pair it with the full
  // points stashed at insert time, then hand the submap to global mapping. Order
  // matters: insert_submap rewrites T_world_origin and drops the per-frame point
  // clouds, so the relative poses are read first.
  void capture_and_insert(const glim::SubMap::Ptr & submap)
  {
    SubMapEntry entry;
    entry.submap = submap;
    const Eigen::Isometry3d T_origin_world = submap->T_world_origin.inverse();
    entry.frames.reserve(submap->frames.size());
    for (const auto & frame : submap->frames) {
      if (!frame) {
        continue;
      }
      FrameRef ref;
      ref.id = frame->id;
      ref.stamp = frame->stamp;
      ref.T_origin_frame = T_origin_world * frame->T_world_lidar;
      const auto found = stash.find(frame->id);
      if (found != stash.end()) {
        ref.points = std::move(found->second.points);
        ref.intensities = std::move(found->second.intensities);
        stash.erase(found);
      }
      entry.frames.push_back(std::move(ref));
    }
    entries.push_back(std::move(entry));
    global_mapping->insert_submap(submap);
  }

  void drain_submaps()
  {
    for (const auto & submap : sub_mapping->get_submaps()) {
      if (submap) {
        capture_and_insert(submap);
      }
    }
  }

  // Record one GNSS fix (already projected to the local metric frame). Stamp is
  // converted to seconds to match EstimationFrame stamps; the ENU covariance is
  // carried through for per-prior weighting.
  void add_gnss(const GnssPoint & p)
  {
    GnssMetric m;
    m.stamp = static_cast<double>(p.stamp_ns) * 1e-9;
    m.xyz = Eigen::Vector3d(p.position[0], p.position[1], p.position[2]);
    m.covariance = p.covariance;
    m.covariance_type = p.covariance_type;
    gnss_points.push_back(m);
  }

  // Linear-interpolate the GNSS position at time `t` (seconds). gnss_points must
  // be sorted by stamp; `t` outside the span clamps to the nearest endpoint.
  Eigen::Vector3d interpolate_gnss(double t) const
  {
    const auto right = std::lower_bound(
      gnss_points.begin(), gnss_points.end(), t,
      [](const GnssMetric & g, double tt) { return g.stamp < tt; });
    if (right == gnss_points.begin()) {
      return right->xyz;
    }
    if (right == gnss_points.end()) {
      return gnss_points.back().xyz;
    }
    const auto left = right - 1;
    const double tl = left->stamp;
    const double tr = right->stamp;
    const double p = (tr > tl) ? (t - tl) / (tr - tl) : 0.0;
    return (1.0 - p) * left->xyz + p * right->xyz;
  }

  // Covariance + type of the GNSS fix closest in time to `t`. gnss_points must be
  // sorted by stamp. Used to weight a submap's prior; nearest (not interpolated)
  // since covariance matrices do not interpolate linearly and adjacent fixes are
  // ~equally representative over a submap's short span.
  const GnssMetric & nearest_gnss(double t) const
  {
    const auto right = std::lower_bound(
      gnss_points.begin(), gnss_points.end(), t,
      [](const GnssMetric & g, double tt) { return g.stamp < tt; });
    if (right == gnss_points.begin()) {
      return *right;
    }
    if (right == gnss_points.end()) {
      return gnss_points.back();
    }
    const auto left = right - 1;
    return (t - left->stamp <= right->stamp - t) ? *left : *right;
  }

  // Build GNSS translation-prior factors from the collected submaps + fixes
  // (ported from glim_ext's gnss_global backend, run synchronously instead of in
  // a background thread). Leaves gnss_factors empty unless at least two submaps
  // are fully covered by the GNSS timespan and the SLAM baseline between the
  // first and last of them exceeds config.gnss_min_baseline.
  void build_gnss_factors()
  {
    gnss_factors.clear();
    if (gnss_points.size() < 2 || entries.empty()) {
      return;
    }

    std::sort(
      gnss_points.begin(), gnss_points.end(),
      [](const GnssMetric & a, const GnssMetric & b) { return a.stamp < b.stamp; });
    const double t_lo = gnss_points.front().stamp;
    const double t_hi = gnss_points.back().stamp;

    // Antenna lever-arm in the submap-origin sensor frame. config.gnss_antenna_offset
    // is the antenna phase center in the cloud (LiDAR) frame; the submap origin X(i)
    // is the LiDAR pose for the CT backend but the IMU pose for the CPU backend, so
    // re-express the antenna point in the IMU frame there (p_imu = T_imu_lidar *
    // p_lidar). {0,0,0} leaves it zero -> identical to the no-correction path.
    Eigen::Vector3d lever_origin(
      config.gnss_antenna_offset[0], config.gnss_antenna_offset[1], config.gnss_antenna_offset[2]);
    if (config.t_lidar_imu) {
      const Eigen::Isometry3d T_lidar_imu = detail::to_isometry(*config.t_lidar_imu);
      lever_origin = T_lidar_imu.inverse() * lever_origin;
    }

    std::vector<std::uint64_t> ids;
    std::vector<std::array<double, 3>> est;
    std::vector<std::array<double, 3>> offsets;  // per-submap antenna offset in world
    std::vector<std::array<double, 3>> gnss;
    std::vector<std::array<double, 9>> covs;  // nearest-fix ENU covariance per submap
    std::vector<std::uint8_t> cov_types;
    for (const auto & entry : entries) {
      if (!entry.submap || entry.frames.empty()) {
        continue;
      }
      // Only constrain submaps whose whole frame span is covered by GNSS, so the
      // mid-frame stamp interpolates between real fixes (mirrors glim_ext's
      // submap-within-window check).
      if (entry.frames.front().stamp < t_lo || entry.frames.back().stamp > t_hi) {
        continue;
      }
      const double t_mid = entry.frames[entry.frames.size() / 2].stamp;
      const Eigen::Vector3d origin = entry.submap->T_world_origin.translation();
      // Rotate the body-fixed lever-arm into the world frame with the submap's
      // pre-optimization orientation (Option A: the heading used for the offset is
      // frozen at build time; iSAM2 then moves the origin under the prior).
      const Eigen::Vector3d offset_world = entry.submap->T_world_origin.rotation() * lever_origin;
      const Eigen::Vector3d fix = interpolate_gnss(t_mid);
      const GnssMetric & near = nearest_gnss(t_mid);
      ids.push_back(static_cast<std::uint64_t>(entry.submap->id));
      est.push_back({origin.x(), origin.y(), origin.z()});
      offsets.push_back({offset_world.x(), offset_world.y(), offset_world.z()});
      gnss.push_back({fix.x(), fix.y(), fix.z()});
      covs.push_back(near.covariance);
      cov_types.push_back(near.covariance_type);
    }
    if (ids.size() < 2) {
      return;
    }

    // Pre-optimization baseline: too little motion makes the planar alignment
    // ill-conditioned (matches glim_ext's min_baseline gate).
    const double dx = est.front()[0] - est.back()[0];
    const double dy = est.front()[1] - est.back()[1];
    const double dz = est.front()[2] - est.back()[2];
    if (std::sqrt(dx * dx + dy * dy + dz * dz) < config.gnss_min_baseline) {
      return;
    }

    // Estimate the world<-GNSS transform (antenna-to-antenna so the lever-arm does
    // not contaminate the fit) and map each fix back onto its submap origin; that
    // mapped position is the submap's translation-prior target. The fitted ENU->world
    // rotation lets each fix's covariance be expressed in the world frame.
    const GnssOffsetTargets aligned = gnss_targets_with_offset(est, offsets, gnss);
    if (aligned.targets.size() != ids.size()) {
      return;
    }

    // Fixed-precision fallback (used when a fix has no usable covariance or
    // gnss_use_covariance is off): the original glim_ext-style behavior.
    const Eigen::Vector3d precisions(
      config.gnss_prior_inf_scale[0], config.gnss_prior_inf_scale[1],
      config.gnss_prior_inf_scale[2]);
    const gtsam::SharedNoiseModel fixed_model = gtsam::noiseModel::Diagonal::Precisions(precisions);

    // Vertical (z) handling mirrors the fixed path: honor a configured z precision,
    // otherwise leave height effectively unconstrained (a large variance ~ zero
    // information) so GNSS height — the weakest GNSS axis — does not fight the LiDAR.
    const double z_variance = config.gnss_prior_inf_scale[2] > 0.0
                                ? 1.0 / config.gnss_prior_inf_scale[2]
                                : kUnconstrainedZVariance;

    using gtsam::symbol_shorthand::X;
    gnss_factors.reserve(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const gtsam::Point3 target(
        aligned.targets[i][0], aligned.targets[i][1], aligned.targets[i][2]);

      gtsam::SharedNoiseModel base = fixed_model;
      if (config.gnss_use_covariance && cov_types[i] != kNavSatCovarianceTypeUnknown) {
        // Horizontal ENU covariance {c_ee, c_en, c_ne, c_nn} from the nearest fix,
        // rotated into the world frame, inflated and floored; z left per z_variance.
        const std::array<double, 4> cov_h = {covs[i][0], covs[i][1], covs[i][3], covs[i][4]};
        const std::array<double, 9> w = gnss_world_prior_covariance(
          cov_h, aligned.world_from_enu_cos, aligned.world_from_enu_sin,
          config.gnss_horizontal_sigma_floor, config.gnss_covariance_inflation, z_variance);
        Eigen::Matrix3d cov_w;
        cov_w << w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8];
        base = gtsam::noiseModel::Gaussian::Covariance(cov_w);
      }

      // Robust-wrap so one multipath outlier cannot dominate; a Huber k of 0
      // disables it.
      gtsam::SharedNoiseModel model = base;
      if (config.gnss_robust_huber_k > 0.0) {
        model = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(config.gnss_robust_huber_k), base);
      }

      gtsam::NonlinearFactor::shared_ptr factor(
        new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(ids[i]), target, model));
      gnss_factors.push_back(factor);
    }
  }

  // Optimized world pose per frame = T_world_origin * T_origin_frame. Keyed by
  // timestamp so the poses come out time-ordered and a duplicate stamp keeps one
  // entry (submap boundaries share no frames, but stay defensive).
  void fill_trajectory(CloudMap & result) const
  {
    std::map<std::int64_t, core::TrajectoryPose> poses;
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        const Eigen::Isometry3d T_world_frame = T_world_origin * ref.T_origin_frame;
        const auto pose = to_pose(ref.stamp, T_world_frame);
        poses[pose.timestamp_ns] = pose;
      }
    }
    result.trajectory.reserve(poses.size());
    for (const auto & entry : poses) {
      result.trajectory.push_back(entry.second);
    }
  }

  // A view of one globally-optimized frame: its stamp, id, world<-LiDAR pose, and
  // the stashed LiDAR-frame points (owned by `entries`). Used by the recovery
  // functions to seed the scan-match target and anchor the chain.
  struct OptimizedFrameView
  {
    double stamp = 0.0;
    std::int64_t id = 0;
    Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
    const FrameRef * ref = nullptr;
  };

  // Every optimized frame across all captured submaps, ascending by stamp. The
  // world pose is T_world_origin * T_origin_frame (T_origin_frame is invariant
  // under global optimization; T_world_origin already holds the optimized value).
  std::vector<OptimizedFrameView> collect_optimized_frames() const
  {
    std::vector<OptimizedFrameView> frames;
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        frames.push_back({ref.stamp, ref.id, T_world_origin * ref.T_origin_frame, &ref});
      }
    }
    std::sort(
      frames.begin(), frames.end(),
      [](const OptimizedFrameView & a, const OptimizedFrameView & b) { return a.stamp < b.stamp; });
    return frames;
  }

  // One optimized frame's full points in the world frame — the scan-match target
  // geometry. Empty when the frame's points were not stashed.
  static std::vector<Eigen::Vector3d> frame_world_points(const OptimizedFrameView & frame)
  {
    std::vector<Eigen::Vector3d> out;
    const FrameRef & ref = *frame.ref;
    out.reserve(ref.points.size());
    for (std::size_t i = 0; i < ref.points.size(); ++i) {
      const std::array<float, 3> p = ref.points.at(i);
      out.push_back(frame.T_world_lidar * Eigen::Vector3d(p[0], p[1], p[2]));
    }
    return out;
  }

  // Transform a LiDAR-frame source cloud into the world frame at `pose` — folds an
  // accepted recovery scan back into the growing scan-match target.
  static std::vector<Eigen::Vector3d> points_to_world(
    const std::vector<Eigen::Vector3d> & lidar, const Eigen::Isometry3d & pose)
  {
    std::vector<Eigen::Vector3d> out;
    out.reserve(lidar.size());
    for (const auto & p : lidar) {
      out.push_back(pose * p);
    }
    return out;
  }

  // Chain-register each window scan in `todo` against `recoverer` and merge the
  // recovered poses into result.trajectory. Each scan's initial guess is the IMU
  // propagation when available (imu_guess), else the previous accepted pose; the
  // fit is accepted if GICP converged, else the IMU guess is the fallback, else
  // the scan is skipped. Only a GICP-converged pose is folded back into the
  // growing target — an unverified IMU-fallback pose must not poison the geometry
  // later scans register against. Recovered poses dedup against the existing
  // trajectory by stamp (a real GLIM pose always wins a collision). Returns how
  // many new poses were added. Shared by recover_warmup / recover_cooldown, which
  // differ only in anchor selection, todo ordering, and the IMU boundary.
  template <typename ImuGuess>
  std::size_t apply_recovered_scans(
    CloudMap & result, const std::vector<const WindowScan *> & todo, ScanMatchRecoverer & recoverer,
    const Eigen::Isometry3d & anchor_pose, const ImuGuess & imu_guess) const
  {
    std::map<std::int64_t, core::TrajectoryPose> merged;
    for (const auto & pose : result.trajectory) {
      merged[pose.timestamp_ns] = pose;
    }

    Eigen::Isometry3d prev = anchor_pose;
    std::size_t recovered = 0;
    for (const WindowScan * ws : todo) {
      const auto guess = imu_guess(ws->stamp);
      const Eigen::Isometry3d init = guess ? *guess : prev;
      const ScanMatchResult res = recoverer.register_scan(ws->points, init);
      Eigen::Isometry3d accepted = init;
      if (res.converged) {
        accepted = res.T_world_lidar;
        // Fold only geometry-verified poses into the growing target; an
        // unverified IMU-fallback pose must not poison later registrations.
        recoverer.insert_target(points_to_world(ws->points, accepted));
      } else if (guess) {
        accepted = *guess;  // registration rejected -> fall back to the IMU guess
      } else {
        continue;  // no geometry fit and no IMU: leave this scan unrecovered
      }
      prev = accepted;
      const auto pose = to_pose(ws->stamp, accepted);
      if (merged.emplace(pose.timestamp_ns, pose).second) {
        ++recovered;
      }
    }
    if (recovered == 0) {
      return 0;
    }
    result.trajectory.clear();
    result.trajectory.reserve(merged.size());
    for (const auto & entry : merged) {
      result.trajectory.push_back(entry.second);
    }
    return recovered;
  }

  // Recover poses for the SLAM warmup window — the scans captured before GLIM's
  // first estimation frame — by scan-matching each against the optimized map, and
  // prepend them to the trajectory. Runs in finish() after the workers join, so it
  // is the sole reader of `warmup`. When a LiDAR-IMU extrinsic is present the
  // buffered IMU seeds each initial guess and is the fallback if a registration
  // fails its gate; in LiDAR-only mode the previous recovered pose seeds the next.
  // A no-op unless recovery is active, there are buffered window scans, and the
  // optimized map has at least one frame.
  void recover_warmup(CloudMap & result) const
  {
    if (!warmup.active || warmup.scans.empty()) {
      return;
    }
    const auto frames = collect_optimized_frames();
    if (frames.empty()) {
      return;
    }
    // Anchor = earliest optimized frame; only scans before it lack a pose.
    const OptimizedFrameView & anchor = frames.front();

    // Optional IMU init/fallback (LiDAR-IMU only, boundary captured): the
    // boundary-frame back-propagation re-anchored onto its optimized pose.
    std::vector<TimedPose> knots;
    bool have_imu = config.t_lidar_imu.has_value() && warmup.boundary_captured;
    Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_odom_imu0_inv = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_world_imu_anchor = Eigen::Isometry3d::Identity();
    if (have_imu) {
      const Eigen::Isometry3d T_lidar_imu = detail::to_isometry(*config.t_lidar_imu);
      std::optional<Eigen::Isometry3d> boundary_world_lidar;
      for (const auto & f : frames) {
        if (f.id == warmup.boundary_id) {
          boundary_world_lidar = f.T_world_lidar;
          break;
        }
      }
      if (boundary_world_lidar) {
        knots = backpropagate_imu(warmup.boundary, warmup.imu, default_gravity_world());
        T_imu_lidar = T_lidar_imu.inverse();
        T_odom_imu0_inv = warmup.boundary.T_world_imu.inverse();
        T_world_imu_anchor = *boundary_world_lidar * T_lidar_imu;
      } else {
        have_imu = false;
      }
    }
    const auto imu_guess = [&](double stamp) -> std::optional<Eigen::Isometry3d> {
      if (!have_imu || knots.size() < 2) {
        return std::nullopt;
      }
      const auto pose_imu = interpolate_pose(knots, stamp);
      if (!pose_imu) {
        return std::nullopt;
      }
      return T_world_imu_anchor * (T_odom_imu0_inv * *pose_imu) * T_imu_lidar;
    };

    // Seed the scan-match target with the first frames of the optimized map (the
    // warmup window is spatially adjacent to them).
    ScanMatchRecoverer recoverer{make_recovery_params(config)};
    const std::size_t seed_n = std::min<std::size_t>(kRecoverySeedFrames, frames.size());
    for (std::size_t i = 0; i < seed_n; ++i) {
      recoverer.insert_target(frame_world_points(frames[i]));
    }
    if (recoverer.target_empty()) {
      return;
    }

    // Recover the pre-anchor window scans closest-first (descending stamp) so the
    // chain grows outward from the optimized map.
    std::vector<const WindowScan *> todo;
    for (const auto & ws : warmup.scans) {
      if (ws.stamp < anchor.stamp) {
        todo.push_back(&ws);
      }
    }
    std::sort(todo.begin(), todo.end(), [](const WindowScan * a, const WindowScan * b) {
      return a->stamp > b->stamp;
    });

    result.recovered_start_pose_count =
      apply_recovered_scans(result, todo, recoverer, anchor.T_world_lidar, imu_guess);
  }

  // Recover poses for the SLAM cooldown window — the trailing scans still inside
  // the odometry smoother window at end-of-sequence, which never reach a finalized
  // submap — by scan-matching each against the optimized map, and append them to
  // the trajectory. The mirror of recover_warmup with the chain running forward
  // from the last optimized frame.
  void recover_cooldown(CloudMap & result) const
  {
    if (!cooldown.active || cooldown.scans.empty()) {
      return;
    }
    const auto frames = collect_optimized_frames();
    if (frames.empty()) {
      return;
    }
    // Anchor = latest optimized frame; only scans after it lack a pose.
    const OptimizedFrameView & anchor = frames.back();

    // Optional IMU init/fallback: forward-propagation from the anchor frame's
    // converged kinematic state (looked up by id in the trailing ring).
    std::vector<TimedPose> knots;
    bool have_imu = config.t_lidar_imu.has_value();
    Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_odom_imuN_inv = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_world_imu_anchor = Eigen::Isometry3d::Identity();
    if (have_imu) {
      const Eigen::Isometry3d T_lidar_imu = detail::to_isometry(*config.t_lidar_imu);
      const BackpropBoundary * boundary = nullptr;
      for (const auto & [id, state] : cooldown.frame_states) {
        if (id == anchor.id) {
          boundary = &state;
          break;
        }
      }
      if (boundary != nullptr) {
        const std::vector<BackpropImu> imu_window(cooldown.imu.begin(), cooldown.imu.end());
        knots = forwardpropagate_imu(*boundary, imu_window, default_gravity_world());
        T_imu_lidar = T_lidar_imu.inverse();
        T_odom_imuN_inv = boundary->T_world_imu.inverse();
        T_world_imu_anchor = anchor.T_world_lidar * T_lidar_imu;
      } else {
        have_imu = false;
      }
    }
    const auto imu_guess = [&](double stamp) -> std::optional<Eigen::Isometry3d> {
      if (!have_imu || knots.size() < 2) {
        return std::nullopt;
      }
      const auto pose_imu = interpolate_pose(knots, stamp);
      if (!pose_imu) {
        return std::nullopt;
      }
      return T_world_imu_anchor * (T_odom_imuN_inv * *pose_imu) * T_imu_lidar;
    };

    // Seed the scan-match target with the last frames of the optimized map.
    ScanMatchRecoverer recoverer{make_recovery_params(config)};
    const std::size_t seed_n = std::min<std::size_t>(kRecoverySeedFrames, frames.size());
    for (std::size_t i = frames.size() - seed_n; i < frames.size(); ++i) {
      recoverer.insert_target(frame_world_points(frames[i]));
    }
    if (recoverer.target_empty()) {
      return;
    }

    // Recover the post-anchor trailing scans in stamp order so the chain grows
    // forward from the optimized map.
    std::vector<const WindowScan *> todo;
    for (const auto & ws : cooldown.scans) {
      if (ws.stamp > anchor.stamp) {
        todo.push_back(&ws);
      }
    }
    std::sort(todo.begin(), todo.end(), [](const WindowScan * a, const WindowScan * b) {
      return a->stamp < b->stamp;
    });

    result.recovered_end_pose_count =
      apply_recovered_scans(result, todo, recoverer, anchor.T_world_lidar, imu_guess);
  }

  // Intensity is all-or-nothing across the whole map (mirrors GLIM's export and
  // what write_pcd expects): true only if every frame with points also carried
  // index-aligned intensities.
  [[nodiscard]] bool detect_with_intensity() const
  {
    bool any_points = false;
    for (const auto & entry : entries) {
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        any_points = true;
        if (ref.intensities.size() != ref.points.size()) {
          return false;
        }
      }
    }
    return any_points;
  }

  // Stream one frame's globally-optimized world points into `grid` (dequantizing
  // the int16 stash via FramePoints::at when in use_gpu mode).
  void add_frame_to_grid(
    VoxelGrid & grid, const Eigen::Isometry3d & T_world_frame, const FrameRef & ref,
    bool with_intensity) const
  {
    const std::size_t n = ref.points.size();
    for (std::size_t i = 0; i < n; ++i) {
      const std::array<float, 3> local = ref.points.at(i);
      const Eigen::Vector3d world = T_world_frame * Eigen::Vector3d(local[0], local[1], local[2]);
      if (with_intensity) {
        grid.add(
          static_cast<float>(world.x()), static_cast<float>(world.y()),
          static_cast<float>(world.z()), ref.intensities[i]);
      } else {
        grid.add(
          static_cast<float>(world.x()), static_cast<float>(world.y()),
          static_cast<float>(world.z()));
      }
    }
  }

  // Single-grid streaming voxelization in entries/frames order. This is the
  // historical path, byte-identical to it; used when num_threads <= 1 so the
  // `--threads 1` reproducibility guarantee is preserved exactly.
  void fill_map_streaming(CloudMap & result, bool with_intensity) const
  {
    VoxelGrid grid(config.input_resolution, with_intensity);
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        add_frame_to_grid(grid, T_world_origin * ref.T_origin_frame, ref, with_intensity);
      }
    }
    result.points = grid.points();
    result.intensities = grid.intensities();
  }

  // Parallel voxelization (Tier-1b): partition the frames across num_threads
  // worker grids, then merge in a fixed order. Deterministic per thread count
  // (each thread sums sequentially, fixed merge order), so it is run-to-run
  // reproducible at a given --threads value; it is NOT byte-identical to the
  // single-thread map (different FP-summation order + voxel order), consistent
  // with the multithreaded map already being tolerance-only.
  void fill_map_parallel(CloudMap & result, bool with_intensity) const
  {
    struct Job
    {
      Eigen::Isometry3d T_world_frame;
      const FrameRef * ref;
    };
    std::vector<Job> jobs;
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        jobs.push_back({T_world_origin * ref.T_origin_frame, &ref});
      }
    }
    if (jobs.empty()) {
      result.points.clear();
      result.intensities.clear();
      return;
    }
    const int nthreads = std::min<int>(config.num_threads, static_cast<int>(jobs.size()));
    std::vector<VoxelGrid> grids;
    grids.reserve(static_cast<std::size_t>(nthreads));
    for (int t = 0; t < nthreads; ++t) {
      grids.emplace_back(config.input_resolution, with_intensity);
    }
    const auto worker = [&](int t) {
      const std::size_t lo = jobs.size() * static_cast<std::size_t>(t) / nthreads;
      const std::size_t hi = jobs.size() * static_cast<std::size_t>(t + 1) / nthreads;
      for (std::size_t j = lo; j < hi; ++j) {
        add_frame_to_grid(
          grids[static_cast<std::size_t>(t)], jobs[j].T_world_frame, *jobs[j].ref, with_intensity);
      }
    };
    // Capture per-thread exceptions (e.g. bad_alloc when a huge map's voxels do
    // not fit) so a worker failure propagates as a clean exception the caller can
    // report, instead of std::terminate. jthread auto-joins on scope exit —
    // including during exception unwinding — closing every terminate path.
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(nthreads), nullptr);
    const auto safe_worker = [&](int t) {
      try {
        worker(t);
      } catch (...) {
        errors[static_cast<std::size_t>(t)] = std::current_exception();
      }
    };
    {
      std::vector<std::jthread> pool;
      pool.reserve(static_cast<std::size_t>(nthreads - 1));
      for (int t = 1; t < nthreads; ++t) {
        pool.emplace_back(safe_worker, t);
      }
      safe_worker(0);
    }  // jthread destructors join all background workers here
    for (const auto & error : errors) {
      if (error) {
        std::rethrow_exception(error);
      }
    }
    for (int t = 1; t < nthreads; ++t) {
      grids[0].merge_from(grids[static_cast<std::size_t>(t)]);
    }
    result.points = grids[0].points();
    result.intensities = grids[0].intensities();
  }

#ifdef BAGWIZ_WITH_SLAM_CUDA
  // Flatten every frame's globally-optimized world points (+ intensities) into a
  // single contiguous array for the GPU voxelizer.
  void build_world_points(
    std::vector<std::array<float, 3>> & out_points, std::vector<float> & out_intensities,
    bool with_intensity) const
  {
    std::size_t total = 0;
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      for (const auto & ref : entry.frames) {
        total += ref.points.size();
      }
    }
    out_points.reserve(total);
    if (with_intensity) {
      out_intensities.reserve(total);
    }
    for (const auto & entry : entries) {
      if (!entry.submap) {
        continue;
      }
      const Eigen::Isometry3d & T_world_origin = entry.submap->T_world_origin;
      for (const auto & ref : entry.frames) {
        if (ref.points.empty()) {
          continue;
        }
        const Eigen::Isometry3d T_world_frame = T_world_origin * ref.T_origin_frame;
        const std::size_t n = ref.points.size();
        for (std::size_t i = 0; i < n; ++i) {
          const std::array<float, 3> local = ref.points.at(i);
          const Eigen::Vector3d world =
            T_world_frame * Eigen::Vector3d(local[0], local[1], local[2]);
          out_points.push_back(
            {static_cast<float>(world.x()), static_cast<float>(world.y()),
             static_cast<float>(world.z())});
          if (with_intensity) {
            out_intensities.push_back(ref.intensities[i]);
          }
        }
      }
    }
  }

  // CPU voxelization of an already-flattened world array (the GPU fallback path).
  static void voxelize_flat_cpu(
    const std::vector<std::array<float, 3>> & pts, const std::vector<float> & ints,
    double resolution, bool with_intensity, CloudMap & result)
  {
    VoxelGrid grid(resolution, with_intensity);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (with_intensity) {
        grid.add(pts[i][0], pts[i][1], pts[i][2], ints[i]);
      } else {
        grid.add(pts[i][0], pts[i][1], pts[i][2]);
      }
    }
    result.points = grid.points();
    result.intensities = grid.intensities();
  }
#endif

  // Rebuild the exported map from every frame's full points, placed at the
  // frame's globally-optimized world pose and merged at config.input_resolution.
  // The optimization ran at GLIM's stock sub-map density; the emitted map is as
  // dense as the requested export voxel allows. Dispatch:
  //   - use_gpu (CUDA build): GPU voxelization, CPU fallback on GPU failure;
  //   - num_threads > 1: parallel CPU voxelization (Tier-1b);
  //   - else: single-grid streaming (byte-identical to the historical output).
  void fill_map(CloudMap & result) const
  {
    const bool with_intensity = detect_with_intensity();

#ifdef BAGWIZ_WITH_SLAM_CUDA
    if (config.use_gpu) {
      std::vector<std::array<float, 3>> world_points;
      std::vector<float> world_intensities;
      build_world_points(world_points, world_intensities, with_intensity);
      if (voxelize_gpu(
            world_points, world_intensities, config.input_resolution, result.points,
            result.intensities)) {
        return;
      }
      // GPU unavailable / OOM: voxelize the already-built flat arrays on the CPU.
      voxelize_flat_cpu(
        world_points, world_intensities, config.input_resolution, with_intensity, result);
      return;
    }
#endif

    if (config.num_threads > 1) {
      fill_map_parallel(result, with_intensity);
    } else {
      fill_map_streaming(result, with_intensity);
    }
  }
};

CloudMapper::CloudMapper(CloudMapperConfig config)
{
  // Silence GLIM's one-time construction chatter (it logs ~50 "config not found /
  // using default" lines while reading params; we drive it with no config dir on
  // purpose). RAII-restored so a throwing GLIM constructor cannot leave the
  // shared logger muted for the rest of the process; genuine runtime warnings
  // still surface afterwards.
  const detail::ScopedLoggerSilence silence;
#ifdef BAGWIZ_WITH_SLAM_CUDA
  // Must run before Impl builds any GLIM module (its odometry smoother snapshots
  // gtsam_points' hook list at construction). Gated on use_gpu so a CPU-backend
  // run never spins up a CUDA context via the GPU factor set.
  if (config.use_gpu) {
    register_gpu_linearization_hook_once();
  }
#endif
  impl_ = std::make_unique<Impl>(config);
}
CloudMapper::~CloudMapper() = default;
CloudMapper::CloudMapper(CloudMapper &&) noexcept = default;
CloudMapper & CloudMapper::operator=(CloudMapper &&) noexcept = default;

void CloudMapper::insert_imu(const ImuSample & imu)
{
  const double stamp = static_cast<double>(imu.stamp_ns) * 1e-9;
  const Eigen::Vector3d linear_acc(
    imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]);
  const Eigen::Vector3d angular_vel(
    imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
  // Route to all three stages (no-ops in LiDAR-only mode): odometry estimates
  // motion from it; sub/global mapping use it for their own IMU factors. In
  // pipeline mode the sample is queued so the consumer interleaves it with scans
  // in exact bag order (the order each preintegrator requires).
  if (impl_->config.disable_pipeline) {
    impl_->consume_imu(stamp, linear_acc, angular_vel);
    return;
  }
  Impl::FeedEvent event;
  event.kind = Impl::FeedEvent::Kind::Imu;
  event.imu_stamp = stamp;
  event.imu_acc = linear_acc;
  event.imu_gyro = angular_vel;
  impl_->enqueue_odom(std::move(event), 0);
}

void CloudMapper::insert_gnss(const GnssPoint & gnss)
{
  // GNSS factors live only in the global graph, so a fix is meaningful only when
  // global mapping runs; ignore otherwise. Buffered now, turned into submap
  // priors in finish().
  if (!impl_->config.enable_gnss) {
    return;
  }
  if (impl_->config.disable_pipeline) {
    impl_->add_gnss(gnss);
    return;
  }
  // Queued (not buffered directly) so the consumer thread stays the sole owner of
  // gnss_points; order among GNSS fixes does not matter (build_gnss_factors sorts).
  Impl::FeedEvent event;
  event.kind = Impl::FeedEvent::Kind::Gnss;
  event.gnss = gnss;
  impl_->enqueue_odom(std::move(event), 0);
}

void CloudMapper::insert(const LidarScan & scan)
{
  if (impl_->config.disable_pipeline) {
    // Fully synchronous path: preprocess + odometry + sub/global on the caller
    // thread. GLIM's LiDAR-IMU init bootstrap dumps an LM iteration table to
    // std::cout from inside odometry->insert_frame; mute std::cout for the whole
    // call (bagwiz prints via fmt::print, never std::cout, so only GLIM's chatter
    // is suppressed).
    const detail::ScopedCoutSilence cout_silence;
    const auto preprocessed = impl_->prepare(scan);
    if (preprocessed) {
      impl_->consume_scan(preprocessed);
    }
    return;
  }

  // Pipeline producer: build + TimeKeeper + preprocess on this (bag-read) thread,
  // then hand the preprocessed frame to the consumer thread which runs odometry +
  // sub/global mapping. std::cout is muted by the consumer for the streaming
  // duration, so no ScopedCoutSilence is needed here. enqueue() blocks while the
  // bounded queue is full (keeping the consumer ~one buffer ahead) and rethrows a
  // latched consumer error.
  const auto preprocessed = impl_->prepare(scan);
  if (!preprocessed) {
    return;
  }
  Impl::FeedEvent event;
  event.kind = Impl::FeedEvent::Kind::Scan;
  event.frame = preprocessed;
  impl_->enqueue_odom(std::move(event), 1);
}

CloudMap CloudMapper::finish()
{
  // Drain the 3-stage feed pipeline first: stop input, let odometry (T2) flush its
  // smoother window into the mapping stage (T3) and close map_queue, then join both
  // workers. After the joins this thread is the sole owner of the GLIM modules, so
  // the flush + optimize below run exactly as the serial path. No-op when the
  // pipeline was never started (config.disable_pipeline, or no inserts).
  if (impl_->pipeline_started) {
    impl_->odom_queue.close();
    impl_->odometry_thread.join();  // closes map_queue after flushing the odom tail
    impl_->mapping_thread.join();
    const auto odom_error = impl_->odom_queue.error();
    const auto map_error = impl_->map_queue.error();
    // Release the streaming cout mute now the workers are gone — before the finish-
    // scope mute below, so the two ScopedCoutSilence guards nest/destruct in LIFO
    // order (resetting the outer one while the inner is alive would corrupt the
    // saved rdbuf). Done before the rethrows so std::cout is restored on error too.
    impl_->feed_silence.reset();
    if (odom_error) {
      std::rethrow_exception(odom_error);
    }
    if (map_error) {
      std::rethrow_exception(map_error);
    }
  }

  // Mute GLIM's std::cout chatter for the whole flush + global optimization (same
  // rationale as insert(): bagwiz's own output goes through fmt::print, not cout).
  const detail::ScopedCoutSilence cout_silence;

  // Flush the odometry smoother window — the remaining frames are marginalized
  // exactly as glim's async pipeline does at end of sequence — into sub mapping,
  // then force out the final submap.
  for (const auto & frame : impl_->odometry->get_remaining_frames()) {
    // Capture the warmup boundary here too: on a bag shorter than the odometry
    // smoother window, the first frame (id 0) is never marginalized mid-stream
    // and only surfaces in this end-of-sequence flush. No-op once already caught.
    impl_->warmup_note_frame(frame);
    // Capture the cooldown boundary: these end-of-sequence frames are the newest
    // to reach a finalized submap, so the LAST one flushed here is the cooldown
    // anchor. cooldown_note_frame overwrites, so it keeps that latest frame.
    impl_->cooldown_note_frame(frame);
    impl_->feed_sub_mapping(frame);
  }
  // This drain only sees submaps the flushed remaining frames newly completed —
  // get_submaps() destructively swaps its queue, so the submaps drained during
  // insert() are already gone. submit_end_of_sequence() then forces a final
  // submap out of whatever odometry frames remain; it builds a fresh submap
  // rather than pulling from that queue, so there is no overlap.
  impl_->drain_submaps();
  for (const auto & submap : impl_->sub_mapping->submit_end_of_sequence()) {
    if (submap) {
      impl_->capture_and_insert(submap);
    }
  }

  // Build GNSS translation priors (config.enable_gnss) from the collected
  // submaps + fixes. They are injected into the global factor graph during
  // optimize() via the on_smoother_update callback below.
  std::size_t gnss_count = 0;
  if (impl_->config.enable_gnss) {
    impl_->build_gnss_factors();
    gnss_count = impl_->gnss_factors.size();
  }

  // The on_smoother_update slot is process-global, so register our injector only
  // around our own optimize() and remove it right after. Register first, capturing
  // the slot id, then hand it to an RAII guard whose destructor removes it. The
  // guard also protects against a throwing optimize() leaving a dangling `impl`
  // callback on the slot (which would fire — and dereference freed memory — for any
  // later mapper instance in the same process, e.g. across tests).
  int gnss_slot_id = -1;
  if (gnss_count > 0) {
    Impl * impl = impl_.get();
    gnss_slot_id = glim::GlobalMappingCallbacks::on_smoother_update.add(
      [impl](gtsam_points::ISAM2Ext &, gtsam::NonlinearFactorGraph & new_factors, gtsam::Values &) {
        // GlobalMapping::optimize() fires on_smoother_update exactly once, and
        // all submap poses X(i) already exist in iSAM2 by now, so the translation
        // priors are valid. Clearing after adding is a belt-and-suspenders guard
        // so they enter the graph exactly once even if GLIM's call count changes.
        if (!impl->gnss_factors.empty()) {
          new_factors.add(impl->gnss_factors);
          impl->gnss_factors.clear();
        }
      });
  }
  // id is the slot handle from the registration above (or -1 when no injector was
  // registered), so the destructor's guard is genuinely conditional.
  struct ScopedGnssCallback
  {
    int id;
    ~ScopedGnssCallback()
    {
      if (id >= 0) {
        glim::GlobalMappingCallbacks::on_smoother_update.remove(id);
      }
    }
  } gnss_callback{gnss_slot_id};

  // Heavy step: global matching-based iSAM2 optimization. Updates each held
  // submap's T_world_origin in place (GlobalMapping::update_submaps). With the
  // GNSS callback registered, the priors enter the graph in this single update.
  impl_->global_mapping->optimize();

  CloudMap result;
  result.gnss_factor_count = gnss_count;
  impl_->fill_trajectory(result);
  impl_->recover_warmup(result);
  impl_->recover_cooldown(result);
  // Surface a warmup buffer overflow so the caller can distinguish "gave up" from
  // "nothing to recover" (warmup_disable() sets this and clears warmup.active).
  result.warmup_overflowed = impl_->warmup.overflowed;
  impl_->fill_map(result);
  return result;
}

}  // namespace bagwiz::core::slam
