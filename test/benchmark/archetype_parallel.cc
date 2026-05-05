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

// EachParallel scaling benchmark. The argument selects how many
// worker threads the World's WorkerPool uses; running the bench
// across t = 1, 2, 4, 8, 16 lets us read the speedup curve.
//
// Note: state.range(0) feeds the thread count into the World
// constructor — this is *not* google-benchmark's `Threads()`
// modifier, which would replicate the body across N threads. We
// want one body running EachParallel internally on N workers.

#include <benchmark/benchmark.h>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose     { double x, y, z; };
  struct Velocity { double vx, vy, vz; };
}

static void BM_ArchetypeEachParallel(benchmark::State &_state)
{
  const unsigned int t = static_cast<unsigned int>(_state.range(0));
  const int64_t N = 1000000;
  World w(t);
  for (int64_t i = 0; i < N; ++i)
    w.Create(Pose{double(i), 0, 0}, Velocity{});

  // Warm: get worker threads spun up and the chunks into cache.
  w.EachParallel<const Pose, Velocity>(
      [](Entity, const Pose &p, Velocity &v) { v.vx = p.x; });

  for (auto _ : _state)
  {
    w.EachParallel<const Pose, Velocity>(
        [](Entity, const Pose &p, Velocity &v) { v.vy = p.x * 0.5; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeEachParallel)
  ->Arg(1)->Arg(2)->Arg(4)->Arg(8)->Arg(16)
  ->Unit(benchmark::kMillisecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
