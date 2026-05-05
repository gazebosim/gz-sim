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

// Archetype-transition benchmarks. Adding or removing a component
// moves an entity to a different archetype, which involves looking
// up (or lazily creating) the destination archetype, copying every
// shared column across, and patching the EntityIndex. The bench
// also reports a baseline Each pass so the per-transition cost can
// be compared against per-iteration cost (the design's "< 10×"
// gate is checked via the absolute timings these benches print).

#include <benchmark/benchmark.h>

#include <vector>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x, y, z; };
  struct Tag  {};
}

// Baseline iteration over N entities — used as the reference cost
// for the Add / Remove benches above.
static void BM_ArchetypeEachBaseline(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  World w(1);
  for (int64_t i = 0; i < N; ++i)
    w.Create(Pose{double(i), 0, 0});

  for (auto _ : _state)
  {
    w.Each<const Pose>([](Entity, const Pose &) {});
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeEachBaseline)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMicrosecond);

// Add a Tag to N entities. Each entity transitions from
// {Pose} -> {Pose, Tag}. PauseTiming/ResumeTiming bracket the
// per-iteration setup so only the Add work is measured.
static void BM_ArchetypeAddTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    World w(1);
    std::vector<Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
      es.push_back(w.Create(Pose{double(i), 0, 0}));
    _state.ResumeTiming();

    for (auto e : es) w.Add(e, Tag{});
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeAddTag)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

// Remove a Tag from N entities. Each entity transitions from
// {Pose, Tag} -> {Pose}. Setup creates the entities WITH the tag
// already present so the timed body is purely the Remove work.
static void BM_ArchetypeRemoveTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    World w(1);
    std::vector<Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
    {
      auto e = w.Create(Pose{double(i), 0, 0});
      w.Add(e, Tag{});
      es.push_back(e);
    }
    _state.ResumeTiming();

    for (auto e : es) w.Remove<Tag>(e);
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeRemoveTag)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
