// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/introspection/message_deserializer.hpp"

#include "bagwiz/core/introspection/introspection_loader.hpp"

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

namespace bagwiz::core
{

namespace
{

// Preview of the first bytes of the payload used in error messages so
// CDR header issues can be diagnosed without re-running with a debugger.
std::string hex_preview(std::span<const std::byte> payload, std::size_t max_bytes = 16)
{
  const std::size_t n = std::min(payload.size(), max_bytes);
  std::string out;
  out.reserve(n * 3 + 4);
  for (std::size_t i = 0; i < n; ++i) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02x ", static_cast<unsigned>(payload[i]));
    out += buf;
  }
  if (payload.size() > n) {
    out += "...";
  }
  return out;
}

}  // namespace

namespace
{

// RAII wrapper around posix_memalign / std::free. Holds an aligned buffer of
// members.size_of_ bytes so the constructor cannot leak it on partial-init
// throw paths.
struct PosixFree
{
  // posix_memalign requires std::free for release; this is the RAII deleter
  // paired with it via std::unique_ptr below.
  void operator()(void * p) const noexcept { std::free(p); }  // NOLINT(cppcoreguidelines-no-malloc)
};
using AlignedBuffer = std::unique_ptr<void, PosixFree>;

AlignedBuffer aligned_alloc_or_throw(std::size_t alignment, std::size_t size)
{
  void * p = nullptr;
  const std::size_t bytes = size == 0 ? 1 : size;
  if (::posix_memalign(&p, alignment, bytes) != 0 || p == nullptr) {
    throw std::bad_alloc();
  }
  return AlignedBuffer{p};
}

// RAII wrapper for rmw_serialized_message_t so init/fini cannot drift apart.
class SerializedMessage
{
public:
  explicit SerializedMessage(std::size_t capacity)
  {
    rcutils_allocator_t alloc = rcutils_get_default_allocator();
    if (rmw_serialized_message_init(&msg_, capacity, &alloc) != RMW_RET_OK) {
      throw std::runtime_error("rmw_serialized_message_init failed");
    }
  }
  ~SerializedMessage() { rmw_serialized_message_fini(&msg_); }

  SerializedMessage(const SerializedMessage &) = delete;
  SerializedMessage & operator=(const SerializedMessage &) = delete;
  SerializedMessage(SerializedMessage &&) = delete;
  SerializedMessage & operator=(SerializedMessage &&) = delete;

  rmw_serialized_message_t & get() noexcept { return msg_; }

private:
  rmw_serialized_message_t msg_ = rmw_get_zero_initialized_serialized_message();
};

// Dismissible guard: pairs members.init_function() with fini_function() so a
// throw between init and successful construction unwinds the partial init.
// Call release() on the success path to hand ownership over to the
// DeserializedMessage destructor.
class MessageInitGuard
{
public:
  MessageInitGuard(
    const rosidl_typesupport_introspection_cpp::MessageMembers * members, void * buffer)
  : members_(members), buffer_(buffer)
  {
    members_->init_function(buffer_, rosidl_runtime_cpp::MessageInitialization::ALL);
  }
  ~MessageInitGuard()
  {
    if (members_ != nullptr) {
      members_->fini_function(buffer_);
    }
  }

  MessageInitGuard(const MessageInitGuard &) = delete;
  MessageInitGuard & operator=(const MessageInitGuard &) = delete;
  MessageInitGuard(MessageInitGuard &&) = delete;
  MessageInitGuard & operator=(MessageInitGuard &&) = delete;

  void release() noexcept { members_ = nullptr; }

private:
  const rosidl_typesupport_introspection_cpp::MessageMembers * members_ = nullptr;
  void * buffer_ = nullptr;
};

}  // namespace

DeserializedMessage::DeserializedMessage(
  const IntrospectionLoad & introspection, std::span<const std::byte> cdr_payload)
{
  if (!introspection.ok()) {
    throw std::runtime_error("introspection not loaded");
  }
  members_ = introspection.members;

  // Allocate an aligned buffer. alignof(std::max_align_t) covers
  // std::string/vector alignment on all ROS 2 supported platforms.
  auto buffer = aligned_alloc_or_throw(alignof(std::max_align_t), members_->size_of_);

  // Default-construct the C++ fields. Guard fini-on-throw until we hand
  // ownership to *this on the success path.
  MessageInitGuard init_guard(members_, buffer.get());

  // Copy the payload into an rcutils-managed buffer so ownership semantics
  // are clean, regardless of what the RMW implementation chooses to do
  // with the bytes.
  SerializedMessage serialized(cdr_payload.size());
  std::memcpy(serialized.get().buffer, cdr_payload.data(), cdr_payload.size());
  serialized.get().buffer_length = cdr_payload.size();

  const rmw_ret_t rc = rmw_deserialize(&serialized.get(), introspection.typesupport, buffer.get());
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    std::string err = "rmw_deserialize failed (size=" + std::to_string(cdr_payload.size()) +
                      ", first bytes: " + hex_preview(cdr_payload) + "): ";
    err += s != nullptr ? s->message : "(no error message)";
    rcutils_reset_error();
    throw std::runtime_error(err);
  }

  // Success: transfer the buffer + init lifetime to *this. From here on,
  // the destructor owns cleanup.
  buffer_ = buffer.release();
  init_guard.release();
  initialized_ = true;
}

DeserializedMessage::~DeserializedMessage()
{
  if (initialized_ && members_ != nullptr && buffer_ != nullptr) {
    members_->fini_function(buffer_);
  }
  // Pairs with posix_memalign in aligned_alloc_or_throw above; std::free is
  // the required release call per POSIX.
  std::free(buffer_);  // NOLINT(cppcoreguidelines-no-malloc)
}

}  // namespace bagwiz::core
