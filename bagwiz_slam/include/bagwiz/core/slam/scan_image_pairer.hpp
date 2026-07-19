// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__SCAN_IMAGE_PAIRER_HPP_
#define BAGWIZ__CORE__SLAM__SCAN_IMAGE_PAIRER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <string>
#include <vector>

// Pairs each camera image of the colorize pass (`map slam --cam`) with the
// temporally NEAREST LiDAR scan, whose world-frame points act as the scene's
// occluder geometry at the image's own time (see MapColorizer::add_image's
// `dynamic_points`). The pairing must be tight, but the recording pipeline
// publishes scans and images tens to a hundred milliseconds after capture, so
// an image usually arrives before the scan captured nearest to it: images
// wait in a short pending queue until the scans bracketing their stamp have
// arrived (scans arrive in stamp order, so once a scan at or beyond the
// image's stamp has been seen, no closer scan can still come), and only the
// latest few scans are kept as pairing candidates. GLIM-free plain data
// throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

// An image pairs with its nearest scan only when the stamps differ by at most
// this many nanoseconds; a farther scan is no useful visibility oracle.
inline constexpr std::int64_t kScanPairWindowNs = 150'000'000;

// Number of recent scans kept as pairing candidates.
inline constexpr std::size_t kScanSlotHistorySize = 4;

class ScanImagePairer
{
public:
  // One camera image awaiting its bracketing scans: the camera index (into
  // the caller's colorizer array), the capture stamp, and the undecoded
  // message (type + payload) carried through untouched.
  struct PendingImage
  {
    std::size_t cam = 0;
    std::int64_t stamp_ns = 0;
    std::string type;
    std::vector<std::byte> payload;
  };

  // One decided image: the image plus the world points of its temporally
  // nearest scan, or an empty span when no scan lies within the pair window.
  // The span borrows the scan slot's storage and stays valid until the next
  // push_scan() call.
  struct Decision
  {
    PendingImage image;
    std::span<const std::array<float, 3>> dynamic_points;
  };

  explicit ScanImagePairer(std::int64_t pair_window_ns = kScanPairWindowNs)
  : pair_window_ns_(pair_window_ns)
  {
  }

  ScanImagePairer(const ScanImagePairer &) = delete;
  ScanImagePairer & operator=(const ScanImagePairer &) = delete;

  // Slot one scan's world-frame points. Scans must arrive in stamp order;
  // only the latest kScanSlotHistorySize slots are kept.
  void push_scan(std::int64_t stamp_ns, std::vector<std::array<float, 3>> world_points);

  // Queue one camera image for pairing.
  void push_image(PendingImage image);

  // End of stream: no closer scan can still arrive, so every still-pending
  // image becomes decidable.
  void finish() { finished_ = true; }

  // True while decide_front() has an image to emit: the pending queue is a
  // FIFO and only its front is examined — an image is decidable once finish()
  // was called or some slotted scan has a stamp at or beyond the image's.
  [[nodiscard]] bool has_decidable() const;

  // Pop the front image and pair it with its temporally nearest slotted scan
  // (the earlier slot wins a tie). Requires has_decidable().
  [[nodiscard]] Decision decide_front();

private:
  struct ScanSlot
  {
    std::int64_t stamp_ns = 0;
    std::vector<std::array<float, 3>> world_points;
  };

  std::int64_t pair_window_ns_;
  std::deque<PendingImage> pending_images_;
  std::deque<ScanSlot> scans_;  // the latest few, in arrival (stamp) order
  bool finished_ = false;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__SCAN_IMAGE_PAIRER_HPP_
