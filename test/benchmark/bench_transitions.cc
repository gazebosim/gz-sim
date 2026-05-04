/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * bench_transitions: Add/Remove a Tag on 100k entities, measure total cost.
 * Goal: < 10× the cost of a plain iteration over the same N entities. If
 * we exceed this, archetype-graph caching is broken.
 */
#include <cstdio>

#include "bench_common.hh"
#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x, y, z; };
  struct Tag  {};
}

int main()
{
  const size_t N = 100'000;
  World w(1);
  std::vector<Entity> es;
  es.reserve(N);
  for (size_t i = 0; i < N; ++i)
    es.push_back(w.Create(Pose{double(i), 0, 0}));

  // Baseline: an Each pass.
  bench::Timer t;
  t.Reset();
  w.Each<const Pose>([](Entity, const Pose &) {});
  double iter = t.Seconds();
  bench::Report("bench_transitions / Each baseline", iter, N);

  // Add Tag to all entities.
  t.Reset();
  for (auto e : es) w.Add(e, Tag{});
  double add = t.Seconds();
  bench::Report("bench_transitions / Add Tag×N", add, N);

  // Remove Tag.
  t.Reset();
  for (auto e : es) w.Remove<Tag>(e);
  double rem = t.Seconds();
  bench::Report("bench_transitions / Remove Tag×N", rem, N);

  std::printf("\nAdd/Each ratio:    %.2f×\n", add / iter);
  std::printf("Remove/Each ratio: %.2f×\n", rem / iter);
  std::printf("Gate: both ratios should be < 10×.\n");
  return 0;
}
