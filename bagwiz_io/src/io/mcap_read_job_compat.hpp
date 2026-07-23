// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_READ_JOB_COMPAT_HPP_
#define IO__MCAP_READ_JOB_COMPAT_HPP_

#include <mcap/read_job_queue.hpp>

// Compatibility shim for mcap's read-job types.
//
// mcap 1.x (e.g. the Jazzy robostack build) places ReadJobQueue and its job
// types under mcap::internal, while mcap 0.x (e.g. Humble) places them
// directly under mcap. bagwiz_io's CMakeLists.txt sets
// BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE when it detects a 1.x header.
namespace bagwiz::io::detail::mcap_compat
{

#ifdef BAGWIZ_MCAP_READ_JOBS_INTERNAL_NAMESPACE
namespace job_ns = mcap::internal;
#else
namespace job_ns = mcap;
#endif

using DecompressChunkJob = job_ns::DecompressChunkJob;
using ReadMessageJob = job_ns::ReadMessageJob;
using ReadJobQueue = job_ns::ReadJobQueue;

}  // namespace bagwiz::io::detail::mcap_compat

#endif  // IO__MCAP_READ_JOB_COMPAT_HPP_
