// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__PROGRESS_BAR_HPP_
#define BAGWIZ__CORE__SLAM__PROGRESS_BAR_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace bagwiz::core::slam
{

// Decide whether the `map slam` progress UI should render. It is shown only on
// an interactive stderr, with NO_COLOR unset and the user's --no-progress flag
// not passed — mirroring `tf`'s "fancy output" gate (src/commands/tf.cpp:178)
// so piped / CI runs stay free of carriage-return spam. Pure for unit testing.
bool progress_enabled(bool stderr_is_tty, bool no_color_env_set, bool no_progress_flag);

// Sum the per-topic message counts (from BagReader::compute_stats) for the
// given topics, skipping empty names and topics absent from the stats. This is
// the number of messages the SLAM read loop will stream — the progress bar's
// denominator. Returns 0 when none of the topics carry a known count (e.g. an
// MCAP whose summary statistics are missing), signalling the caller to fall
// back to an indeterminate bar.
std::int64_t progress_total(
  const io::BagReader::Stats & stats, std::span<const std::string> topics);

// RAII progress reporter for the bag-read / GLIM-feed phase of `map slam`.
// Determinate (with ETA) when the total message count is known; otherwise an
// indeterminate bar that animates as messages arrive. A no-op when disabled
// (non-TTY / --no-progress): every method stays safe to call and renders
// nothing. Draws to stderr so the run summary on stdout stays clean.
class ScanProgress
{
public:
  // total <= 0 selects the indeterminate bar. `enabled` false makes this a
  // complete no-op (no terminal output is ever produced).
  ScanProgress(std::int64_t total, bool enabled);
  ~ScanProgress();
  ScanProgress(const ScanProgress &) = delete;
  ScanProgress & operator=(const ScanProgress &) = delete;
  ScanProgress(ScanProgress &&) = delete;
  ScanProgress & operator=(ScanProgress &&) = delete;

  // Report `processed` messages read so far, with `scans` decoded scans shown
  // as postfix. Redraws are throttled (per-mille change or ~50 ms) so a high
  // IMU rate cannot flood stderr.
  void update(std::int64_t processed, std::int64_t scans);

  // Finalize the bar (100% / completed) and drop to a fresh line. Idempotent.
  void done();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  // null when disabled
};

// RAII thread-driven spinner for the blocking global-optimization phase
// (CloudMapper::finish()), which exposes no per-step progress. A worker thread
// animates an indeterminate bar from construction until stop() / destruction.
// A no-op when disabled.
class FinalizeSpinner
{
public:
  FinalizeSpinner(std::string label, bool enabled);
  ~FinalizeSpinner();
  FinalizeSpinner(const FinalizeSpinner &) = delete;
  FinalizeSpinner & operator=(const FinalizeSpinner &) = delete;
  FinalizeSpinner(FinalizeSpinner &&) = delete;
  FinalizeSpinner & operator=(FinalizeSpinner &&) = delete;

  // Stop the animation, mark it complete, and join the worker. Idempotent;
  // the destructor calls it.
  void stop();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;  // null when disabled
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__PROGRESS_BAR_HPP_
