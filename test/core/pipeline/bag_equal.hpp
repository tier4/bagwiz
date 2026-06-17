// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__PIPELINE__BAG_EQUAL_HPP_
#define CORE__PIPELINE__BAG_EQUAL_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <filesystem>

namespace bagwiz::test
{

// Differential oracle shared across the pipeline backend tests: assert two bags
// carry the same (topic, timestamp, payload-bytes) sequence in emission order.
// This is the byte-identical bar a non-sequential backend must clear against
// the SequentialBackend reference. `inline` so multiple test TUs can include it
// without an ODR clash.
inline void expect_bags_equal(const std::filesystem::path & a, const std::filesystem::path & b)
{
  auto ra = bagwiz::io::open_read(a);
  auto rb = bagwiz::io::open_read(b);
  bagwiz::io::RawMessage ma;
  bagwiz::io::RawMessage mb;
  while (true) {
    const bool got_a = ra->next(ma);
    const bool got_b = rb->next(mb);
    ASSERT_EQ(got_a, got_b) << "bags differ in message count";
    if (!got_a) {
      break;
    }
    EXPECT_EQ(ma.topic->name, mb.topic->name);
    EXPECT_EQ(ma.timestamp_ns, mb.timestamp_ns);
    ASSERT_EQ(ma.payload.size(), mb.payload.size());
    EXPECT_EQ(0, std::memcmp(ma.payload.data(), mb.payload.data(), ma.payload.size()));
  }
}

}  // namespace bagwiz::test

#endif  // CORE__PIPELINE__BAG_EQUAL_HPP_
