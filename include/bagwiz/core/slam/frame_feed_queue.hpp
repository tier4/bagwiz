// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__FRAME_FEED_QUEUE_HPP_
#define BAGWIZ__CORE__SLAM__FRAME_FEED_QUEUE_HPP_

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <mutex>
#include <utility>

// Bounded single-producer/single-consumer FIFO used by CloudMapper to overlap its
// producer (bag read + GLIM preprocess) with its consumer (GLIM odometry +
// sub/global mapping). Strict FIFO ordering means the consumer sees events in
// exactly the order the producer pushed them, which is the bag order — this is
// what keeps the pipelined output bit-identical to the serial path.
//
// Capacity is measured in caller-supplied "weight": heavy preprocessed-scan events
// weigh 1, tiny IMU/GNSS events weigh 0, so the in-flight memory is bounded by the
// number of buffered scans regardless of the IMU rate. push() blocks while adding
// an item would exceed capacity AND the queue is non-empty; pop() blocks until an
// item is available, or the queue is closed-and-drained / cancelled / failed.
//
// The close()/fail()/error() contract mirrors core::pipeline::BoundedMessageQueue;
// a slam-local generic template is used here because that class is hardcoded to
// OwnedMessage and bounds by payload bytes, neither of which fits a GLIM frame.
namespace bagwiz::core::slam
{

template <typename T>
class FrameFeedQueue
{
public:
  explicit FrameFeedQueue(std::size_t capacity_weight) : capacity_(capacity_weight) {}

  FrameFeedQueue(const FrameFeedQueue &) = delete;
  FrameFeedQueue & operator=(const FrameFeedQueue &) = delete;
  FrameFeedQueue(FrameFeedQueue &&) = delete;
  FrameFeedQueue & operator=(FrameFeedQueue &&) = delete;
  ~FrameFeedQueue() = default;

  // Push one item with `weight`. Blocks while adding it would exceed capacity and
  // the queue is non-empty (a single item into an empty queue is always admitted,
  // so an over-weight item cannot deadlock). Returns false if the queue was
  // failed or cancelled (the item is not enqueued).
  bool push(T value, std::size_t weight)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(
      lock, [&] { return cancelled_ || failed_ || size_ == 0 || size_ + weight <= capacity_; });
    if (cancelled_ || failed_) {
      return false;
    }
    queue_.push_back(Item{std::move(value), weight});
    size_ += weight;
    not_empty_.notify_one();
    return true;
  }

  // Pop the next item in FIFO order into `out`. Blocks until an item is available.
  // Returns false (leaving `out` untouched) when the queue is cancelled/failed, or
  // closed AND fully drained.
  bool pop(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&] { return cancelled_ || failed_ || !queue_.empty() || closed_; });
    if (cancelled_ || failed_ || queue_.empty()) {
      return false;
    }
    Item item = std::move(queue_.front());
    queue_.pop_front();
    size_ -= item.weight;
    out = std::move(item.value);
    not_full_.notify_one();
    return true;
  }

  // Producer is done: pop() returns false once the queue has drained.
  void close()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  // Latch the first error (idempotent) and wake both ends so a blocked producer or
  // consumer unblocks; push()/pop() then return false. Used by the consumer to
  // surface an exception to the producer/finish().
  void fail(std::exception_ptr error)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!error_) {
      error_ = std::move(error);
    }
    failed_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  // Hard stop: abandon any pending items and unblock both ends immediately. Used
  // when tearing down without a graceful drain (e.g. destruction mid-feed).
  void cancel()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    not_empty_.notify_all();
    not_full_.notify_all();
  }

  [[nodiscard]] std::exception_ptr error() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

private:
  struct Item
  {
    T value;
    std::size_t weight;
  };

  mutable std::mutex mutex_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<Item> queue_;
  std::size_t capacity_;
  std::size_t size_ = 0;
  bool closed_ = false;
  bool failed_ = false;
  bool cancelled_ = false;
  std::exception_ptr error_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__FRAME_FEED_QUEUE_HPP_
