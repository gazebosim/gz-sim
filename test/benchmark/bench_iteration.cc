/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * microbench_10m + robot_mix_50 (section 12 of the Phase 0a design).
 *
 * NOTE: The design calls for a ≥ 5× and ≥ 3× speedup vs the current
 * unordered_map-based ECM. Implementing the baseline-comparison scaffold
 * requires building against the existing EntityComponentManager, which is
 * only sensible after 0b integration. In 0a this bench reports the ECS's
 * absolute numbers — comparison to the baseline is a 0b deliverable.
 */
#include <cstdio>
#include <string>

#include "bench_common.hh"
#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose     { double x, y, z, rx, ry, rz, rw; };
  struct Velocity { double vx, vy, vz, wx, wy, wz; };
  struct Name     { char   value[32]; };
}

static void Microbench10M(size_t _n)
{
  World w(1);
  bench::Timer t;

  t.Reset();
  for (size_t i = 0; i < _n; ++i)
  {
    w.Create(
        Pose{double(i), 0, 0, 0, 0, 0, 1},
        Velocity{0, 0, 0, 0, 0, 0},
        Name{});
  }
  double build = t.Seconds();
  bench::Report("microbench_10m / build", build, _n);

  // Warm iteration pass.
  w.Each<const Pose, Velocity>([](Entity, const Pose &p, Velocity &v)
  { v.vx = p.x; });

  const int kIters = 10;
  t.Reset();
  for (int k = 0; k < kIters; ++k)
  {
    w.Each<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v) { v.vy = p.x * 0.5; });
  }
  double each = t.Seconds();
  bench::Report("microbench_10m / Each<Pose,Velocity>",
                each, _n * kIters);
}

static void RobotMix50()
{
  World w(1);
  // 50 robots × ~20 components ≈ 1000 entities, one per "robot body".
  // For realism we vary archetypes (mix of Name present/absent).
  for (int i = 0; i < 50; ++i)
  {
    for (int j = 0; j < 20; ++j)
    {
      if (j % 2 == 0)
      {
        w.Create(Pose{}, Velocity{}, Name{});
      }
      else
      {
        w.Create(Pose{}, Velocity{});
      }
    }
  }
  bench::Timer t;
  t.Reset();
  for (int k = 0; k < 10'000; ++k)
  {
    w.Each<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v)
        { v.vx = p.x + 1.0; v.vy = p.y; });
  }
  double each = t.Seconds();
  bench::Report("robot_mix_50 / Each<Pose,Velocity>",
                each, 1000 * 10'000);
}

int main(int argc, char **argv)
{
  size_t n = 10'000'000;
  if (argc > 1) n = std::stoull(argv[1]);
  std::printf("== Phase 0a ECS iteration bench ==\n");
  Microbench10M(n);
  RobotMix50();
  return 0;
}
