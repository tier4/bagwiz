// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/decoder/introspection_decoder.hpp"
#include "bagwiz/core/decoder/schema_decoder.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

namespace bagwiz::core::decoder
{

namespace
{

constexpr const char * kLogger = "bagwiz.core.decoder";

// Read the BAGWIZ_DECODER environment variable. Empty string means
// unset, which the factory treats as "schema-first auto policy".
const char * decoder_override()
{
  const char * raw = std::getenv("BAGWIZ_DECODER");
  return raw == nullptr ? "" : raw;
}

bool override_forces_introspection(const char * raw)
{
  return std::strcmp(raw, "introspection") == 0;
}

}  // namespace

OpenDecoderResult open_decoder(const io::TopicInfo & topic)
{
  const char * override_value = decoder_override();
  const bool forced_introspection = override_forces_introspection(override_value);

  std::string schema_attempt_error;
  if (!forced_introspection) {
    auto schema_result = SchemaDecoder::open(topic);
    if (schema_result.ok()) {
      BAGWIZ_LOG_DEBUG(
        kLogger, "topic '%s' (type %s): backend=schema", topic.name.c_str(), topic.type.c_str());
      return schema_result;
    }
    // Capture for the combined error path; fall through to introspection.
    schema_attempt_error = std::move(schema_result.error);
  }

  auto intro_result = IntrospectionDecoder::open(topic);
  if (intro_result.ok()) {
    BAGWIZ_LOG_DEBUG(
      kLogger, "topic '%s' (type %s): backend=introspection%s", topic.name.c_str(),
      topic.type.c_str(), forced_introspection ? " (forced via BAGWIZ_DECODER)" : "");
    return intro_result;
  }

  OpenDecoderResult combined;
  combined.error = "no decoder available for topic '" + topic.name + "' (type " + topic.type + "):";
  if (!schema_attempt_error.empty()) {
    combined.error += " schema=" + schema_attempt_error + ";";
  } else if (forced_introspection) {
    combined.error += " schema=skipped (BAGWIZ_DECODER=introspection);";
  } else {
    combined.error += " schema=not attempted;";
  }
  combined.error += " introspection=" + intro_result.error;
  return combined;
}

}  // namespace bagwiz::core::decoder
