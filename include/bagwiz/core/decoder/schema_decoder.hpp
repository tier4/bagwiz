// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__DECODER__SCHEMA_DECODER_HPP_
#define BAGWIZ__CORE__DECODER__SCHEMA_DECODER_HPP_

// Internal header (not installed). Implementation detail of open_decoder()
// — clients should always go through the factory.

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <span>
#include <string_view>
#include <utility>

namespace bagwiz::core::decoder
{

// Schema-driven decoder backend. Front-loads schema parsing into open()
// so per-message decode is just the CDR walk.
//
// open() returns an error when the schema cannot be parsed or contains
// unsupported types (wstring, float128) so the factory can fall through
// to the introspection backend.
class SchemaDecoder : public Decoder
{
public:
  // Take ownership of the parsed model. Built by open() below.
  explicit SchemaDecoder(msg_schema::SchemaModel schema) noexcept : schema_(std::move(schema)) {}

  DecodeResult decode(std::span<const std::byte> payload) const override;
  std::string_view backend() const noexcept override { return "schema"; }

  // Returns OpenDecoderResult with non-null decoder on success, or an
  // OpenDecoderResult with empty decoder + populated `error` on failure
  // (parse error or unsupported type discovered).
  static OpenDecoderResult open(const io::TopicInfo & topic);

private:
  msg_schema::SchemaModel schema_;
};

}  // namespace bagwiz::core::decoder

#endif  // BAGWIZ__CORE__DECODER__SCHEMA_DECODER_HPP_
