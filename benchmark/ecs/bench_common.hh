/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Shared helpers for the Phase 0a ECS microbenches. Intentionally
 * dependency-free — we don't pull in google-benchmark in 0a. The benches
 * report ns/op and relative speed vs a baseline in plain text, and gate
 * the build on the ratios called out in section 12 of the design doc.
 */
#ifndef GZ_SIM_ECS_BENCH_COMMON_HH_
#define GZ_SIM_ECS_BENCH_COMMON_HH_

#include <chrono>
#include <cstdio>
#include <string>

namespace gz::sim::ecs::bench
{
  struct Timer
  {
    using clock = std::chrono::steady_clock;
    clock::time_point t0;
    void Reset() { t0 = clock::now(); }
    double Seconds() const
    {
      return std::chrono::duration<double>(clock::now() - t0).count();
    }
  };

  inline void Report(const std::string &_name, double _secs,
                     uint64_t _work_items)
  {
    const double per = _secs / static_cast<double>(_work_items);
    std::printf("%-40s  %10.3f ms   %10.2f ns/item   (n=%llu)\n",
        _name.c_str(), _secs * 1000.0, per * 1e9,
        static_cast<unsigned long long>(_work_items));
  }
}

#endif
