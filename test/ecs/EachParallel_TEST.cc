/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>
#include <atomic>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x, y, z; };
  struct Vel  { double vx, vy, vz; };
}

TEST(EachParallel, VisitsAllEntities)
{
  World w(4);
  const int N = 10'000;
  for (int i = 0; i < N; ++i)
    w.Create(Pose{double(i), 0, 0}, Vel{0, 0, 0});

  std::atomic<int> count{0};
  w.EachParallel<const Pose, Vel>(
      [&](Entity, const Pose &p, Vel &v)
  {
    count.fetch_add(1, std::memory_order_relaxed);
    v.vx = p.x * 2.0;
  });
  EXPECT_EQ(count.load(), N);

  // Verify writes are visible.
  int good = 0;
  w.Each<const Pose, const Vel>(
      [&](Entity, const Pose &p, const Vel &v)
  {
    // Each entity's vx is set to exactly p.x * 2.0 by the same
    // arithmetic in EachParallel above, so a bit-exact compare is
    // intentional here.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
    if (v.vx == p.x * 2.0) ++good;
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
  });
  EXPECT_EQ(good, N);
}

TEST(EachParallel, DeterministicUnderFixedThreadCount)
{
  auto run = [](){
    World w(4);
    for (int i = 0; i < 2000; ++i)
      w.Create(Pose{double(i), 0, 0}, Vel{});
    double sum = 0;
    std::mutex m;
    // Collect per-entity updates. Values only depend on entity identity, so
    // the final state should be identical across runs for fixed thread count.
    w.EachParallel<const Pose, Vel>(
        [&](Entity, const Pose &p, Vel &v)
    {
      v.vx = p.x * 3.0 + 1.0;
    });
    w.Each<const Vel>([&](Entity, const Vel &v) { sum += v.vx; });
    return sum;
  };
  double a = run();
  double b = run();
  double c = run();
  EXPECT_DOUBLE_EQ(a, b);
  EXPECT_DOUBLE_EQ(b, c);
}
