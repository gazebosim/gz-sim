/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Iteration microbenchmarks for the archetype ECS core
// (`gz::sim::ecs::World`). Two scenes:
//
//   * microbenchN — N entities, all sharing one archetype
//     ({Pose, Velocity, Name}). Stresses the inner Each<...> loop in
//     its best-case (single-archetype, dense chunks) shape.
//
//   * robot_mix_50 — 50 "robots" × 20 entities each, where half of
//     each robot's entities have Name and half don't. Two
//     archetypes, ~1000 entities total. Probes the archetype-graph
//     traversal cost when the query matches more than one archetype.

#include <benchmark/benchmark.h>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose     { double x, y, z, rx, ry, rz, rw; };
  struct Velocity { double vx, vy, vz, wx, wy, wz; };
  struct Name     { char   value[32]; };
}

// microbenchN / iteration. Build the world once outside the timing
// loop; the timed body is one Each<const Pose, Velocity> pass.
static void BM_ArchetypeMicrobenchEach(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  World w(1);
  for (int64_t i = 0; i < N; ++i)
  {
    w.Create(
        Pose{double(i), 0, 0, 0, 0, 0, 1},
        Velocity{0, 0, 0, 0, 0, 0},
        Name{});
  }
  // Warm the chunks into cache.
  w.Each<const Pose, Velocity>(
      [](Entity, const Pose &p, Velocity &v) { v.vx = p.x; });

  for (auto _ : _state)
  {
    w.Each<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v) { v.vy = p.x * 0.5; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeMicrobenchEach)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// microbenchN / build path. Each timed iteration constructs a fresh
// World and creates N entities — measures the per-entity cost of
// type-pack sort, archetype lookup, and column placement-construction.
static void BM_ArchetypeMicrobenchBuild(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    World w(1);
    for (int64_t i = 0; i < N; ++i)
    {
      w.Create(
          Pose{double(i), 0, 0, 0, 0, 0, 1},
          Velocity{0, 0, 0, 0, 0, 0},
          Name{});
    }
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeMicrobenchBuild)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// robot_mix_50: 50 robots × 20 entities ≈ 1000 entities, split across
// two archetypes ({Pose, Velocity, Name} and {Pose, Velocity}).
static void BM_ArchetypeRobotMix50(benchmark::State &_state)
{
  World w(1);
  for (int i = 0; i < 50; ++i)
  {
    for (int j = 0; j < 20; ++j)
    {
      if (j % 2 == 0)
        w.Create(Pose{}, Velocity{}, Name{});
      else
        w.Create(Pose{}, Velocity{});
    }
  }
  for (auto _ : _state)
  {
    w.Each<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v)
        { v.vx = p.x + 1.0; v.vy = p.y; });
  }
  _state.SetItemsProcessed(_state.iterations() * 1000);
}
BENCHMARK(BM_ArchetypeRobotMix50)->Unit(benchmark::kMicrosecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
