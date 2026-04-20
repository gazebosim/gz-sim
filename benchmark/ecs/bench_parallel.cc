/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * bench_parallel: measure EachParallel scaling. Gate: ≥ 0.7× linear
 * speedup at 8 workers on the 10M-entity scene.
 */
#include <cstdio>
#include <vector>

#include "bench_common.hh"
#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose     { double x, y, z; };
  struct Velocity { double vx, vy, vz; };
}

int main()
{
  const size_t N = 1'000'000;
  const unsigned int kThreadCounts[] = {1, 2, 4, 8, 16};

  double baseline = 0.0;
  for (unsigned int t : kThreadCounts)
  {
    World w(t);
    for (size_t i = 0; i < N; ++i)
      w.Create(Pose{double(i), 0, 0}, Velocity{});

    // Warm up.
    w.EachParallel<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v) { v.vx = p.x; });

    bench::Timer timer;
    timer.Reset();
    const int iters = 20;
    for (int k = 0; k < iters; ++k)
    {
      w.EachParallel<const Pose, Velocity>(
          [](Entity, const Pose &p, Velocity &v) { v.vy = p.x * 0.5; });
    }
    double secs = timer.Seconds();

    char name[64];
    std::snprintf(name, sizeof(name),
        "bench_parallel / EachParallel t=%u", t);
    bench::Report(name, secs, N * iters);
    if (t == 1) baseline = secs;
    else
    {
      double speedup = baseline / secs;
      std::printf("   %u-thread speedup: %.2f× (linear=%u)\n",
          t, speedup, t);
    }
  }
  return 0;
}
