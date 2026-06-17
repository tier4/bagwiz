// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__BOUNDED_MESSAGE_QUEUE_HPP_
#define BAGWIZ__CORE__PIPELINE__BOUNDED_MESSAGE_QUEUE_HPP_

#include "bagwiz/core/pipeline/owned_message.hpp"

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <mutex>
#include <utility>

namespace bagwiz::core::pipeline
{

// Default cap on the total payload bytes buffered between the read and write
// threads. Bounds peak RSS: the read thread blocks once the in-flight payload
// would cross this, so a fast reader cannot outrun a slow writer and balloon
// memory. 128 MiB is small next to a multi-GB bag yet large enough to keep the
// writer fed across read-stage stalls.
inline constexpr std::size_t kDefaultQueueBytes = 128UL * 1024UL * 1024UL;

// A single-producer / single-consumer blocking queue of OwnedMessages, bounded
// by total payload bytes. PipelinedBackend's read thread is the sole producer
// and its write thread the sole consumer; this queue is the ONLY object shared
// between them, which keeps the data-race surface tiny. FIFO order is preserved,
// so a single consumer emits messages in the producer's (reader's) emission
// order — that is what makes PipelinedBackend's output byte-identical to
// SequentialBackend.
//
// Shutdown is cooperative and deadlock-free: every wait predicate also wakes on
// close() (producer done) and fail() (a fatal error on either side), so neither
// end can block forever once the run is ending.
class BoundedMessageQueue
{
public:
  explicit BoundedMessageQueue(std::size_t max_bytes = kDefaultQueueBytes)
  : max_bytes_(max_bytes == 0 ? 1 : max_bytes)
  {
  }

  // Holds a mutex and condition variables: neither copyable nor movable.
  BoundedMessageQueue(const BoundedMessageQueue &) = delete;
  BoundedMessageQueue & operator=(const BoundedMessageQueue &) = delete;
  BoundedMessageQueue(BoundedMessageQueue &&) = delete;
  BoundedMessageQueue & operator=(BoundedMessageQueue &&) = delete;
  ~BoundedMessageQueue() = default;

  // Producer: block until the message fits under the byte cap (or the queue was
  // failed), then enqueue. Returns false iff the queue was failed (the consumer
  // died) — the producer should stop. A message larger than the whole cap is
  // still admitted when the queue is empty, so an oversized payload can never
  // deadlock the pipeline.
  bool push(OwnedMessage && msg)
  {
    const std::size_t bytes = msg.payload.size();
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(
      lock, [&] { return failed_ || cur_bytes_ == 0 || cur_bytes_ + bytes <= max_bytes_; });
    if (failed_) {
      return false;
    }
    cur_bytes_ += bytes;
    queue_.push_back(std::move(msg));
    not_empty_.notify_one();
    return true;
  }

  // Consumer: block until a message is available, the queue is closed and
  // drained, or it was failed. On success moves the front message into `out` and
  // returns true. Returns false when no message will ever come (closed+drained,
  // or failed) — the consumer should stop.
  bool pop(OwnedMessage & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&] { return failed_ || !queue_.empty() || closed_; });
    if (failed_ || queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop_front();
    cur_bytes_ -= out.payload.size();
    not_full_.notify_one();
    return true;
  }

  // Producer signals it will push no more. Wakes a consumer blocked on an empty
  // queue so it can drain the remainder and then stop.
  void close()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
  }

  // Signal a fatal error from either side. Latches the FIRST exception and wakes
  // both ends so neither blocks forever. Idempotent; later calls are ignored.
  void fail(std::exception_ptr eptr)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!failed_) {
        failed_ = true;
        error_ = std::move(eptr);
      }
    }
    not_full_.notify_all();
    not_empty_.notify_all();
  }

  // The first latched exception, or nullptr if the run completed cleanly.
  // Const (Con.2): a pure observer that locks only to read; mutex_ is mutable.
  std::exception_ptr error() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return error_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable not_full_;
  std::condition_variable not_empty_;
  std::deque<OwnedMessage> queue_;
  std::size_t cur_bytes_ = 0;
  const std::size_t max_bytes_;
  bool closed_ = false;
  bool failed_ = false;
  std::exception_ptr error_;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__BOUNDED_MESSAGE_QUEUE_HPP_
