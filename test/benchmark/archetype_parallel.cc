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

// EachParallel scaling benchmark for the archetype core, with a
// single-threaded baseline against the public
// `gz::sim::EntityComponentManager` for context.
//
// The argument on BM_ArchetypeEachParallel selects the worker
// count for the World's WorkerPool (1, 2, 4, 8, 16) — this is
// *not* google-benchmark's `Threads()` modifier, which would
// replicate the body across N threads. Reading the curve:
//   * 1 thread vs Legacy serial: pure storage-layout speedup.
//   * 1 thread vs N threads:     parallel scaling factor.
//
// What "Legacy" measures depends on the build flag:
//   * GZ_SIM_ARCHETYPE_ECM=OFF — BM_LegacyEach runs against the
//     legacy unordered_map ECM. Read this run for the true
//     legacy-vs-archetype-parallel comparison.
//   * GZ_SIM_ARCHETYPE_ECM=ON  — the public ECM is itself
//     archetype-backed, so BM_LegacyEach measures the facade on
//     top of the archetype core. Read this run for
//     facade-vs-direct comparison.

#include <benchmark/benchmark.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include "gz/sim/ecs/World.hh"

using gz::sim::components::LinearVelocity;
using gz::sim::components::Pose;

// ---------------------------------------------------------------------------
// Archetype core — parallel iteration, parameterized by worker count.
// ---------------------------------------------------------------------------

static void BM_ArchetypeEachParallel(benchmark::State &_state)
{
  const unsigned int t = static_cast<unsigned int>(_state.range(0));
  const int64_t N = 1000000;
  gz::sim::ecs::World w(t);
  for (int64_t i = 0; i < N; ++i)
  {
    w.Create(
        Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
        LinearVelocity(gz::math::Vector3d(0, 0, 0)));
  }
  // Warm: spin up the worker threads and pull chunks into cache.
  w.EachParallel<Pose, LinearVelocity>(
      [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
      { v.Data().X() = p.Data().Pos().X(); });

  for (auto _ : _state)
  {
    w.EachParallel<Pose, LinearVelocity>(
        [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
        { v.Data().X() = p.Data().Pos().X() * 0.5; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeEachParallel)
  ->Arg(1)->Arg(2)->Arg(4)->Arg(8)->Arg(16)
  ->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Legacy / public-facade comparison — single-threaded Each over the
// same workload. The legacy ECM has no parallel iteration primitive,
// so this is the natural reference point against which the
// archetype's parallel curve is read.
// ---------------------------------------------------------------------------

static void BM_LegacyEach(benchmark::State &_state)
{
  const int64_t N = 1000000;
  gz::sim::EntityComponentManager ecm;
  for (int64_t i = 0; i < N; ++i)
  {
    auto e = ecm.CreateEntity();
    ecm.CreateComponent<Pose>(e, Pose(
        gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
    ecm.CreateComponent<LinearVelocity>(e, LinearVelocity(
        gz::math::Vector3d(0, 0, 0)));
  }
  // Warm.
  ecm.Each<Pose, LinearVelocity>(
      [](const gz::sim::Entity &, Pose *p, LinearVelocity *v) -> bool
      { v->Data().X() = p->Data().Pos().X(); return true; });

  for (auto _ : _state)
  {
    ecm.Each<Pose, LinearVelocity>(
        [](const gz::sim::Entity &, Pose *p, LinearVelocity *v) -> bool
        { v->Data().X() = p->Data().Pos().X() * 0.5; return true; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_LegacyEach)->Unit(benchmark::kMillisecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
