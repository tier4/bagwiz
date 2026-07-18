// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__PIPELINE__OWNED_MESSAGE_HPP_
#define BAGWIZ__CORE__PIPELINE__OWNED_MESSAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::core::pipeline
{

// An owned copy of a routed message, decoupled from the reader's zero-copy
// io::RawMessage. io::RawMessage's payload span and topic pointer are
// invalidated by the next BagReader::next() call, so a backend that overlaps
// reading and writing on separate threads cannot hand a RawMessage across the
// thread boundary. The read thread instead fills an OwnedMessage — copying the
// payload bytes and the already-resolved output topic name — and the write
// thread consumes it after the reader has moved on. PipelinedBackend is the
// only user.
struct OwnedMessage
{
  // The resolved OUTPUT topic name (after routing/rename), owned so it outlives
  // the input TopicInfo the router viewed.
  std::string out_topic;
  std::int64_t timestamp_ns = 0;
  // Owned copy of the message payload bytes.
  std::vector<std::byte> payload;
};

}  // namespace bagwiz::core::pipeline

#endif  // BAGWIZ__CORE__PIPELINE__OWNED_MESSAGE_HPP_
