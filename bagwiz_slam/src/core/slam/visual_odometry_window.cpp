// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_window.hpp"  // NOLINT(build/include_subdir) src-local header

#include "visual_factors.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <glim/common/imu_integration.hpp>

#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <deque>
#include <exception>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core::slam::vio
{

namespace
{

double to_sec(std::int64_t stamp_ns)
{
  return 1.0e-9 * static_cast<double>(stamp_ns);
}

// SE(3) interpolation of one keyframe's IMU-rate predicted poses at t
// (seconds). Clamped rather than nullopt: an observation's stamp is always
// meant to fall inside [anchor, anchor + window_span_ns], but this is a fold,
// not a lookup -- a factor should never be dropped just because floating
// point stamp arithmetic lands it a hair outside either end.
Eigen::Isometry3d interpolate_pose(
  const std::vector<double> & times, const std::vector<Eigen::Isometry3d> & poses, double t)
{
  if (t <= times.front()) {
    return poses.front();
  }
  if (t >= times.back()) {
    return poses.back();
  }

  const auto hi_it = std::upper_bound(times.begin(), times.end(), t);
  const auto hi = static_cast<std::size_t>(hi_it - times.begin());
  const std::size_t lo = hi - 1;
  const double t0 = times[lo];
  const double t1 = times[hi];
  const double alpha = (t - t0) / (t1 - t0);

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
    poses[lo].translation() + alpha * (poses[hi].translation() - poses[lo].translation());
  result.linear() = Eigen::Quaterniond(poses[lo].rotation())
                      .slerp(alpha, Eigen::Quaterniond(poses[hi].rotation()))
                      .toRotationMatrix();
  return result;
}

}  // namespace

// One folded observation: a track's per-observation constant rig pose
// (body_P_sensor), the observation itself, and what triangulation and
// marginalized-keyframe assembly need to place and color it. Living in this
// TU only (not the header) is what keeps visual_factors.hpp -- and the
// gtsam_points / SmartProjectionRigFactor headers it pulls in -- out of the
// public interface.
struct FoldedObs
{
  std::uint64_t kf_index = 0;
  Eigen::Isometry3d body_P_sensor = Eigen::Isometry3d::Identity();
  gtsam::Point2 measurement;
  std::array<std::uint8_t, 3> rgb{};
  std::int64_t stamp_ns = 0;
};

// All of WindowSolver's state. Defined entirely in this TU (see the header's
// class comment): besides glim, a WindowKeyframe's preintegrated IMU
// measurement is a gtsam::PreintegratedImuMeasurements (gtsam/navigation/
// ImuFactor.h), and last_tracks keys on visual::TrackKey (visual_factors.hpp)
// -- neither of which the public interface needs to expose.
struct WindowSolver::Impl
{
  explicit Impl(WindowConfig config_in)
  : config(std::move(config_in)),
    // Default-constructs IMUIntegrationParams, which reads glim's
    // GlobalConfig ("config_sensors"'s imu_acc_noise/imu_gyro_noise/
    // imu_int_noise, defaulting to 0.01/0.001/0.001 when no config file is
    // installed) -- the same GlobalConfig-fallback hazard documented at
    // visual_odometry.cpp:33-37 for NaiveInitialStateEstimation.
    imu_integration(std::make_unique<glim::IMUIntegration>())
  {
    // push_keyframe() indexes window.back() unconditionally and
    // marginalize()'s while-loop pops until window.size() <=
    // max_keyframes -- max_keyframes < 1 would empty the deque out from
    // under a still-running push_keyframe() call. min_track_obs < 1 and
    // window_span_ns <= 0 are similarly nonsensical: the former would let
    // add_visual_factors() emit factors from a single observation (nothing
    // to triangulate), the latter would fold every observation against a
    // zero- or negative-length (or literally empty) prediction span.
    if (config.max_keyframes < 1) {
      throw std::invalid_argument("WindowConfig::max_keyframes must be >= 1");
    }
    if (config.min_track_obs < 1) {
      throw std::invalid_argument("WindowConfig::min_track_obs must be >= 1");
    }
    if (config.window_span_ns <= 0) {
      throw std::invalid_argument("WindowConfig::window_span_ns must be > 0");
    }
  }

  // One rebuilt-window keyframe: its gtsam keys (X/V/B(index)), the
  // observation group it anchors, the current state estimate, and the
  // preintegrated IMU measurements from the PREVIOUS keyframe to this one.
  // pim is empty for window.front() (no previous keyframe) and for a pair
  // with fewer than 2 IMU samples between them (see append_predicted_keyframe).
  struct WindowKeyframe
  {
    std::uint64_t index = 0;
    ObservationGroup group;
    gtsam::NavState nav;
    gtsam::imuBias::ConstantBias bias;
    std::optional<gtsam::PreintegratedImuMeasurements> pim;
  };

  WindowConfig config;
  std::unique_ptr<glim::IMUIntegration> imu_integration;
  std::deque<WindowKeyframe> window;
  std::uint64_t next_index = 0;
  bool initialized = false;
  // Set by initialize(); consumed by the first push_keyframe() call, which
  // attaches that first group to the already-created anchor keyframe rather
  // than predicting a new one -- there is no "previous" keyframe to predict
  // from, so the first pushed group is anchored at the initialized state.
  bool anchor_pending = false;

  // Edge priors on window.front(), tying the rebuilt graph to the world
  // frame. Reset to the loose initial values in set_initial_edge_noises();
  // replaced by the popped keyframe's marginal covariance in marginalize().
  gtsam::SharedNoiseModel edge_pose_noise;
  gtsam::SharedNoiseModel edge_velocity_noise;
  gtsam::SharedNoiseModel edge_bias_noise;

  // Stashed after every solve() for marginals (marginalize()) and
  // triangulation (collect_keyframe()).
  gtsam::NonlinearFactorGraph last_graph;
  gtsam::Values last_values;
  std::unordered_map<visual::TrackKey, std::vector<FoldedObs>, visual::TrackKeyHash> last_tracks;

  void set_initial_edge_noises();
  void append_predicted_keyframe(const ObservationGroup & group);
  void solve();
  void add_visual_factors(gtsam::NonlinearFactorGraph & graph);
  std::vector<MarginalizedKeyframe> marginalize();
  [[nodiscard]] MarginalizedKeyframe collect_keyframe(const WindowKeyframe & kf) const;
  [[nodiscard]] std::optional<Eigen::Vector3d> triangulate_track(
    const std::vector<FoldedObs> & obs_list) const;
};

void WindowSolver::Impl::set_initial_edge_noises()
{
  // Pose: tight, the anchor state is exact (either initialize()'s argument or
  // a just-marginalized keyframe's solved state before this fallback runs).
  // Velocity: deliberately loose -- the gravity-aligned init carries no
  // velocity information, and the visual factors must be free to pull it
  // toward the true value. Sigma 5.0 (not 1.0): a velocity error linear in
  // time is, to first order, absorbable by re-fitting the landmark's own
  // depth -- only the acceleration-induced curvature is actually visually
  // observable -- so a more confident prior out-pulls the surviving visual
  // signal and pins the error short of correction (measured: sigma 1.0 left
  // ~0.19 m/s of a 0.3 m/s injected error uncorrected; 5.0 recovers to
  // ~0.02 m/s). This value is a tuning default of this window solver, not
  // part of the frozen Phase 2 smart-factor recipe. Bias: loose but not as
  // loose as velocity.
  edge_pose_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-3);
  edge_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3, 5.0);
  edge_bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
}

void WindowSolver::Impl::append_predicted_keyframe(const ObservationGroup & group)
{
  const WindowKeyframe & prev = window.back();
  const double prev_sec = to_sec(prev.group.anchor_stamp_ns);
  const double cur_sec = to_sec(group.anchor_stamp_ns);

  int num_integrated = 0;
  imu_integration->integrate_imu(prev_sec, cur_sec, prev.bias, &num_integrated);

  WindowKeyframe kf;
  kf.index = next_index++;
  kf.group = group;
  kf.bias = prev.bias;  // carried forward; the bias random walk factor lets LM adjust it

  if (num_integrated >= 2) {
    // Copied once, here, and reused across every rebuild: gtsam::ImuFactor
    // applies first-order bias corrections internally, so re-integrating this
    // same interval whenever the bias estimate changes is unnecessary.
    kf.pim = gtsam::PreintegratedImuMeasurements(imu_integration->integrated_measurements());
    kf.nav = kf.pim->predict(prev.nav, prev.bias);
  } else {
    // Same degradation glim's own odometry uses: too few IMU samples to trust
    // an ImuFactor, so leave pim empty (solve() substitutes a loose velocity
    // BetweenFactor) and carry the previous state forward as the initial
    // guess for LM.
    kf.nav = prev.nav;
  }

  window.push_back(std::move(kf));
}

void WindowSolver::Impl::solve()
{
  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;

  // Rebuilt from scratch every call: no factor identity, no ISAM2 slots.
  // Values seed from each keyframe's CURRENT nav/bias, so LM starts from the
  // previous solve's answer even though the graph itself is brand new.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  for (const WindowKeyframe & kf : window) {
    values.insert(X(kf.index), kf.nav.pose());
    values.insert(V(kf.index), kf.nav.velocity());
    values.insert(B(kf.index), kf.bias);
  }

  const WindowKeyframe & oldest = window.front();
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    X(oldest.index), oldest.nav.pose(), edge_pose_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    V(oldest.index), oldest.nav.velocity(), edge_velocity_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    B(oldest.index), oldest.bias, edge_bias_noise);

  for (std::size_t i = 1; i < window.size(); ++i) {
    const WindowKeyframe & prev = window[i - 1];
    const WindowKeyframe & cur = window[i];

    if (cur.pim.has_value()) {
      graph.emplace_shared<gtsam::ImuFactor>(
        X(prev.index), V(prev.index), X(cur.index), V(cur.index), B(prev.index), *cur.pim);
    } else {
      // Same degradation glim's own odometry uses when an inter-keyframe gap
      // has too few IMU samples: couple the velocities loosely instead of an
      // ImuFactor built from a near-empty preintegration.
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
        V(prev.index), V(cur.index), gtsam::Vector3::Zero(),
        gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
      // Unlike glim's own odometry (which still has scan matching to hold
      // the pose in this situation), a camera-only window has NOTHING
      // constraining X(cur) here unless this keyframe also happens to carry
      // qualifying visual tracks -- an empty group, or every track short of
      // min_track_obs, leaves X(cur) with zero rows/columns in the linear
      // system: structurally unconstrained, not just loosely constrained.
      // gtsam's default LevenbergMarquardtParams (diagonalDamping = false,
      // used here) damps the Gauss-Newton Hessian additively (lambda * I,
      // not lambda * its own diagonal), which happens to make even a fully
      // disconnected variable well-conditioned -- verified empirically for
      // this exact scenario: it just doesn't move, no exception. Add the
      // pose BetweenFactor anyway: relying on an optimizer-internal damping
      // detail to keep an otherwise-nonsensical "zero information about
      // this pose" state well-posed is not something to depend on across
      // gtsam versions or a future lm_params change, and asserting the
      // relative pose the keyframes' CURRENT nav estimates already imply
      // (whatever append_predicted_keyframe or the last solve left them at)
      // costs nothing real -- it is not new information beyond that seed.
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        X(prev.index), X(cur.index), prev.nav.pose().between(cur.nav.pose()),
        gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
    }

    graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
      B(prev.index), B(cur.index), gtsam::imuBias::ConstantBias(),
      gtsam::noiseModel::Isotropic::Sigma(6, config.bias_random_walk_sigma));
  }

  add_visual_factors(graph);

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(20);
  gtsam::Values result;
  try {
    result = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
  } catch (const gtsam::IndeterminantLinearSystemException &) {
    // Defensive, not a specific known trigger given the pose BetweenFactor
    // fallback above (see its comment: not reproduced even with that
    // fallback removed, thanks to gtsam's default additive LM damping) --
    // guards against some other rank-deficient combination that reasoning
    // missed, or a future change to lm_params or the optimizer's defaults.
    // No logger exists on this class; keep the pre-solve seed values rather
    // than propagate the exception out of push_keyframe -- the keyframes'
    // nav/bias below simply don't move this round, and
    // last_graph/last_values still stash the attempted graph with every
    // current key present (as the SEED, not a solved point), so
    // triangulation and the next rebuild's Marginals lookup don't break.
    result = values;
  }

  for (WindowKeyframe & kf : window) {
    const gtsam::Pose3 pose = result.at<gtsam::Pose3>(X(kf.index));
    const gtsam::Vector3 velocity = result.at<gtsam::Vector3>(V(kf.index));
    kf.nav = gtsam::NavState(pose, velocity);
    kf.bias = result.at<gtsam::imuBias::ConstantBias>(B(kf.index));
  }

  last_graph = std::move(graph);
  last_values = std::move(result);
}

void WindowSolver::Impl::add_visual_factors(gtsam::NonlinearFactorGraph & graph)
{
  using gtsam::symbol_shorthand::X;

  last_tracks.clear();

  for (const WindowKeyframe & kf : window) {
    // IMU-rate pose prediction across the whole window span this keyframe
    // anchors, refreshed on every rebuild from the keyframe's LATEST velocity
    // and bias -- the advantage of rebuilding over an incremental smoother,
    // where these per-observation poses would otherwise go stale between
    // bias updates.
    const double anchor_sec = to_sec(kf.group.anchor_stamp_ns);
    const double span_end_sec = to_sec(kf.group.anchor_stamp_ns + config.window_span_ns);

    std::vector<double> pred_times;
    std::vector<Eigen::Isometry3d> pred_poses;
    imu_integration->integrate_imu(
      anchor_sec, span_end_sec, kf.nav, kf.bias, pred_times, pred_poses);
    const Eigen::Isometry3d & pred_anchor = pred_poses.front();

    for (const VisualObservation & obs : kf.group.observations) {
      if (
        obs.camera_id < 0 || static_cast<std::size_t>(obs.camera_id) >= config.t_imu_cams.size()) {
        continue;  // no extrinsic for this camera: nothing to fold or project through
      }

      const Eigen::Isometry3d pred_obs =
        interpolate_pose(pred_times, pred_poses, to_sec(obs.stamp_ns));
      // Delta is the keyframe-local motion between the anchor and this
      // observation's own stamp; folding it into the rig pose is what lets
      // simultaneous and staggered camera triggers share one factor shape.
      const Eigen::Isometry3d delta = pred_anchor.inverse() * pred_obs;
      const Eigen::Isometry3d body_p_sensor =
        delta * config.t_imu_cams[static_cast<std::size_t>(obs.camera_id)];

      last_tracks[visual::track_key(obs)].push_back(
        FoldedObs{kf.index, body_p_sensor, gtsam::Point2(obs.x, obs.y), obs.rgb, obs.stamp_ns});
    }
  }

  const auto calibration = visual::normalized_calibration();
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, config.obs_sigma);
  const auto params = visual::make_smart_projection_params();
  const auto min_obs = static_cast<std::size_t>(config.min_track_obs);

  // The emitted factor order becomes part of the rebuilt graph, so walk
  // tracks in key order instead of the hash map's.
  std::vector<visual::TrackKey> keys;
  keys.reserve(last_tracks.size());
  for (const auto & entry : last_tracks) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  for (const visual::TrackKey & key : keys) {
    const std::vector<FoldedObs> & obs_list = last_tracks.at(key);

    std::unordered_set<std::uint64_t> distinct_kfs;
    for (const FoldedObs & obs : obs_list) {
      distinct_kfs.insert(obs.kf_index);
    }
    if (distinct_kfs.size() < 2 || obs_list.size() < min_obs) {
      continue;  // constrains nothing, or too short to trust
    }

    // One rig camera per observation, its pose the folded per-observation
    // extrinsic. Observations from the same keyframe repeat that keyframe's
    // pose key with distinct entry indices, the rig factor's documented
    // non-unique-keys mode -- exactly Phase 2's make_factor, but keyed per
    // keyframe instead of per submap.
    auto rig = std::make_shared<gtsam::CameraSet<visual::RigCamera>>();
    rig->reserve(obs_list.size());
    auto factor = std::make_shared<visual::RigFactor>(noise, rig, params);
    for (std::size_t i = 0; i < obs_list.size(); ++i) {
      rig->emplace_back(gtsam::Pose3(obs_list[i].body_P_sensor.matrix()), calibration);
      factor->add(obs_list[i].measurement, X(obs_list[i].kf_index), i);
    }
    graph.add(factor);
  }
}

std::vector<MarginalizedKeyframe> WindowSolver::Impl::marginalize()
{
  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;

  std::vector<MarginalizedKeyframe> out;
  while (window.size() > static_cast<std::size_t>(config.max_keyframes)) {
    // Triangulate BEFORE popping: last_tracks/last_values still hold the
    // just-solved graph including this keyframe's own factors and values.
    const WindowKeyframe popped = window.front();
    out.push_back(collect_keyframe(popped));
    window.pop_front();

    if (window.empty()) {
      break;
    }
    const WindowKeyframe & new_oldest = window.front();

    try {
      gtsam::Marginals marginals(last_graph, last_values);
      edge_pose_noise =
        gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(X(new_oldest.index)));
      edge_velocity_noise =
        gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(V(new_oldest.index)));
      edge_bias_noise =
        gtsam::noiseModel::Gaussian::Covariance(marginals.marginalCovariance(B(new_oldest.index)));
    } catch (const std::exception &) {
      // Degenerate windows happen (e.g. too few visual constraints to make
      // the linear system full rank). Per-variable marginals already ignore
      // cross-correlations -- an accepted approximation because the global
      // co-visibility layer re-optimizes everything -- so falling back to
      // the loose initial priors on top of that is a reasonable second
      // approximation rather than propagating a bad covariance.
      set_initial_edge_noises();
    }

    // Prune IMU history at or before the new oldest keyframe's own anchor
    // (find_imu_data's cursor includes the sample AT that anchor too; a
    // dt = 0 first step in any future integrate_imu() call is harmless):
    // nothing before it is needed by any future integrate_imu() call, which
    // always starts its scan from the front of the queue.
    std::vector<double> delta_times;
    std::vector<Eigen::Matrix<double, 7, 1>> imu_data;
    const int cursor = imu_integration->find_imu_data(
      0.0, to_sec(new_oldest.group.anchor_stamp_ns), delta_times, imu_data);
    imu_integration->erase_imu_data(cursor);
  }
  return out;
}

MarginalizedKeyframe WindowSolver::Impl::collect_keyframe(const WindowKeyframe & kf) const
{
  MarginalizedKeyframe out;
  out.stamp_ns = kf.group.anchor_stamp_ns;
  out.T_world_imu = Eigen::Isometry3d(kf.nav.pose().matrix());
  out.v_world_imu = kf.nav.velocity();
  out.imu_bias = kf.bias.vector();

  std::vector<visual::TrackKey> keys;
  keys.reserve(last_tracks.size());
  for (const auto & entry : last_tracks) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());

  for (const visual::TrackKey & key : keys) {
    const std::vector<FoldedObs> & obs_list = last_tracks.at(key);

    const FoldedObs * own = nullptr;
    std::unordered_set<std::uint64_t> distinct_kfs;
    for (const FoldedObs & obs : obs_list) {
      distinct_kfs.insert(obs.kf_index);
      if (obs.kf_index == kf.index) {
        own = &obs;
      }
    }
    if (own == nullptr || distinct_kfs.size() < 2) {
      continue;  // not observed from this keyframe, or constrains nothing on its own
    }

    const std::optional<Eigen::Vector3d> point = triangulate_track(obs_list);
    if (!point.has_value()) {
      continue;
    }
    out.landmarks_world.push_back(*point);
    out.landmark_rgb.push_back(own->rgb);  // this keyframe's own view of the track
  }
  return out;
}

std::optional<Eigen::Vector3d> WindowSolver::Impl::triangulate_track(
  const std::vector<FoldedObs> & obs_list) const
{
  using gtsam::symbol_shorthand::X;

  const auto calibration = visual::normalized_calibration();
  gtsam::CameraSet<visual::RigCamera> cameras;
  visual::RigCamera::MeasurementVector measurements;
  cameras.reserve(obs_list.size());
  measurements.reserve(obs_list.size());

  for (const FoldedObs & obs : obs_list) {
    const gtsam::Pose3 kf_pose = last_values.at<gtsam::Pose3>(X(obs.kf_index));
    const Eigen::Isometry3d T_world_cam = Eigen::Isometry3d(kf_pose.matrix()) * obs.body_P_sensor;
    cameras.emplace_back(gtsam::Pose3(T_world_cam.matrix()), calibration);
    measurements.push_back(obs.measurement);
  }

  // Same recipe as visual_factors.cpp's seed triangulation: rank tolerance
  // sized for normalized (not pixel) coordinates, and a 3-sigma reprojection
  // gate against obs_sigma.
  const gtsam::TriangulationParameters params(
    1.0e-9, /*enableEPI=*/false, 150.0, 3.0 * config.obs_sigma);
  const gtsam::TriangulationResult result = gtsam::triangulateSafe(cameras, measurements, params);
  if (!result.valid()) {
    return std::nullopt;
  }
  return *result;
}

WindowSolver::WindowSolver(WindowConfig config) : impl_(std::make_unique<Impl>(std::move(config)))
{
}

WindowSolver::~WindowSolver() = default;

void WindowSolver::insert_imu(
  double stamp, const Eigen::Vector3d & linear_acc, const Eigen::Vector3d & angular_vel)
{
  impl_->imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

void WindowSolver::initialize(
  std::int64_t stamp_ns, const Eigen::Isometry3d & T_world_imu, const Eigen::Vector3d & v_world_imu,
  const Eigen::Matrix<double, 6, 1> & imu_bias)
{
  impl_->window.clear();
  impl_->next_index = 0;
  impl_->last_tracks.clear();
  impl_->last_graph = gtsam::NonlinearFactorGraph();
  impl_->last_values = gtsam::Values();

  Impl::WindowKeyframe kf;
  kf.index = impl_->next_index++;
  kf.group.anchor_stamp_ns = stamp_ns;
  kf.nav = gtsam::NavState(gtsam::Pose3(T_world_imu.matrix()), v_world_imu);
  kf.bias = gtsam::imuBias::ConstantBias(imu_bias);
  impl_->window.push_back(std::move(kf));

  impl_->set_initial_edge_noises();
  impl_->anchor_pending = true;
  impl_->initialized = true;
}

bool WindowSolver::initialized() const
{
  return impl_->initialized;
}

std::vector<MarginalizedKeyframe> WindowSolver::push_keyframe(const ObservationGroup & group)
{
  // Precondition (documented on the declaration): initialize() must have
  // been called first. window.back() below is otherwise UB on an empty
  // deque; fail soft rather than crash a caller that raced initialization.
  if (!impl_->initialized) {
    return {};
  }

  if (impl_->anchor_pending) {
    // First push: the window already holds the initialize()d anchor state
    // with no observations; attach this group to it instead of predicting a
    // new keyframe, since there is no previous keyframe to predict from.
    impl_->window.back().group = group;
    impl_->anchor_pending = false;
  } else {
    impl_->append_predicted_keyframe(group);
  }

  impl_->solve();
  return impl_->marginalize();
}

std::optional<Eigen::Isometry3d> WindowSolver::predict_T_world_imu(std::int64_t stamp_ns)
{
  if (impl_->window.empty()) {
    return std::nullopt;
  }

  const Impl::WindowKeyframe & newest = impl_->window.back();
  int num_integrated = 0;
  impl_->imu_integration->integrate_imu(
    to_sec(newest.group.anchor_stamp_ns), to_sec(stamp_ns), newest.bias, &num_integrated);

  if (num_integrated < 2) {
    return Eigen::Isometry3d(newest.nav.pose().matrix());
  }

  const gtsam::NavState predicted =
    impl_->imu_integration->integrated_measurements().predict(newest.nav, newest.bias);
  return Eigen::Isometry3d(predicted.pose().matrix());
}

std::vector<MarginalizedKeyframe> WindowSolver::window_snapshot()
{
  if (impl_->window.empty()) {
    return {};
  }

  // Refresh the fold cache and solved states so every entry is consistent
  // with the CURRENT window, even if called without an intervening push.
  impl_->solve();

  std::vector<MarginalizedKeyframe> result;
  result.reserve(impl_->window.size());
  for (const Impl::WindowKeyframe & kf : impl_->window) {
    result.push_back(impl_->collect_keyframe(kf));
  }
  return result;
}

}  // namespace bagwiz::core::slam::vio
