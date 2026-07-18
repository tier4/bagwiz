// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__DECODER__INTROSPECTION_DECODER_HPP_
#define BAGWIZ__CORE__DECODER__INTROSPECTION_DECODER_HPP_

// Internal header (not installed). Implementation detail of open_decoder().

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/introspection/introspection_loader.hpp"

#include <span>
#include <string_view>

namespace bagwiz::core::decoder
{

// Introspection-typesupport decoder backend. Caches the IntrospectionLoad
// (which keeps the dlopen handle alive — `load_introspection` documents
// that pointers must outlive all use of decoded messages, and the leak is
// acceptable for short-lived CLI runs).
//
// Each decode() call:
//   1. allocates a buffer aligned for the largest member type
//   2. runs members.init_function to default-construct the C++ fields
//   3. calls rmw_deserialize to populate them from the CDR payload
//   4. walks MessageMembers to materialise a cdr_walker::Value tree
//   5. calls members.fini_function and frees the buffer
//
// Steps 1-3 + 5 are what `core::DeserializedMessage` already does;
// step 4 is the new "introspection → Value" walker bagwiz needed before
// it could share consumers with the schema-driven path.
class IntrospectionDecoder : public Decoder
{
public:
  explicit IntrospectionDecoder(IntrospectionLoad introspection) noexcept
  : introspection_(introspection)
  {
  }

  DecodeResult decode(std::span<const std::byte> payload) const override;
  std::string_view backend() const noexcept override { return "introspection"; }

  // Returns OpenDecoderResult with non-null decoder on success, or an
  // OpenDecoderResult with empty decoder + populated `error` on failure
  // (typically: typesupport `.so` not on AMENT_PREFIX_PATH).
  static OpenDecoderResult open(const io::TopicInfo & topic);

private:
  IntrospectionLoad introspection_;
};

}  // namespace bagwiz::core::decoder

#endif  // BAGWIZ__CORE__DECODER__INTROSPECTION_DECODER_HPP_
