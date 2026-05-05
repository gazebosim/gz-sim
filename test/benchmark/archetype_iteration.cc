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

// Iteration microbenchmarks comparing the archetype core
// (`gz::sim::ecs::World`) against the public
// `gz::sim::EntityComponentManager`. Both sides use the same
// in-tree gz::sim::components types so the comparison is
// apples-to-apples — same component data layout, same call
// pattern, only the storage strategy differs.
//
// Two scenes:
//
//   * microbench / N — N entities, all sharing one component set
//     ({Pose, LinearVelocity, Name}). Best case for the archetype
//     (single dense archetype) and a representative case for the
//     legacy ECM (its per-type maps see the same N keys).
//
//   * robot_mix_50 — 50 "robots" × 20 entities each, where half of
//     each robot's entities have a Name and half don't. Two
//     archetypes / two distinct key sets, ~1000 entities total.
//
// What "Legacy" measures depends on the build flag:
//   * GZ_SIM_ARCHETYPE_ECM=OFF — BM_Legacy* runs against the
//     legacy unordered_map ECM; BM_Archetype* runs against the
//     standalone gz-sim-ecs core. Read this run for true
//     legacy-vs-archetype comparison.
//   * GZ_SIM_ARCHETYPE_ECM=ON  — the public ECM is itself
//     archetype-backed, so BM_Legacy* measures the facade on top
//     of the archetype core, and BM_Archetype* measures the core
//     directly. Read this run for facade-vs-direct comparison.

#include <benchmark/benchmark.h>

#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "gz/sim/ecs/World.hh"

using gz::sim::components::LinearVelocity;
using gz::sim::components::Name;
using gz::sim::components::Pose;

// ---------------------------------------------------------------------------
// Archetype core (gz::sim::ecs::World)
// ---------------------------------------------------------------------------

// microbench / iteration. Build once, time Each.
static void BM_ArchetypeMicrobenchEach(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  gz::sim::ecs::World w(1);
  for (int64_t i = 0; i < N; ++i)
  {
    w.Create(
        Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
        LinearVelocity(gz::math::Vector3d(0, 0, 0)),
        Name(std::string("e")));
  }
  // Warm the chunks into cache.
  w.Each<Pose, LinearVelocity>(
      [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
      { v.Data().X() = p.Data().Pos().X(); });

  for (auto _ : _state)
  {
    w.Each<Pose, LinearVelocity>(
        [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
        { v.Data().X() = p.Data().Pos().X() * 0.5; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeMicrobenchEach)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// microbench / build path. Each timed iteration constructs a fresh
// World and creates N entities — measures the per-entity cost of
// type-pack sort, archetype lookup, and column placement.
static void BM_ArchetypeMicrobenchBuild(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    gz::sim::ecs::World w(1);
    for (int64_t i = 0; i < N; ++i)
    {
      w.Create(
          Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
          LinearVelocity(gz::math::Vector3d(0, 0, 0)),
          Name(std::string("e")));
    }
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeMicrobenchBuild)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// robot_mix_50 / iteration. ~1000 entities split across two
// archetypes ({Pose, LinearVelocity, Name} and {Pose, LinearVelocity}).
static void BM_ArchetypeRobotMix50(benchmark::State &_state)
{
  gz::sim::ecs::World w(1);
  for (int i = 0; i < 50; ++i)
  {
    for (int j = 0; j < 20; ++j)
    {
      if (j % 2 == 0)
        w.Create(Pose(), LinearVelocity(), Name(std::string("e")));
      else
        w.Create(Pose(), LinearVelocity());
    }
  }
  for (auto _ : _state)
  {
    w.Each<Pose, LinearVelocity>(
        [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
        {
          v.Data().X() = p.Data().Pos().X() + 1.0;
          v.Data().Y() = p.Data().Pos().Y();
        });
  }
  _state.SetItemsProcessed(_state.iterations() * 1000);
}
BENCHMARK(BM_ArchetypeRobotMix50)->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------------------
// Legacy / public-facade comparison benches
// ---------------------------------------------------------------------------

static void BM_LegacyMicrobenchEach(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  gz::sim::EntityComponentManager ecm;
  for (int64_t i = 0; i < N; ++i)
  {
    auto e = ecm.CreateEntity();
    ecm.CreateComponent<Pose>(e, Pose(
        gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
    ecm.CreateComponent<LinearVelocity>(e, LinearVelocity(
        gz::math::Vector3d(0, 0, 0)));
    ecm.CreateComponent<Name>(e, Name(std::string("e")));
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
BENCHMARK(BM_LegacyMicrobenchEach)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

static void BM_LegacyMicrobenchBuild(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    gz::sim::EntityComponentManager ecm;
    for (int64_t i = 0; i < N; ++i)
    {
      auto e = ecm.CreateEntity();
      ecm.CreateComponent<Pose>(e, Pose(
          gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
      ecm.CreateComponent<LinearVelocity>(e, LinearVelocity(
          gz::math::Vector3d(0, 0, 0)));
      ecm.CreateComponent<Name>(e, Name(std::string("e")));
    }
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_LegacyMicrobenchBuild)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

static void BM_LegacyRobotMix50(benchmark::State &_state)
{
  gz::sim::EntityComponentManager ecm;
  for (int i = 0; i < 50; ++i)
  {
    for (int j = 0; j < 20; ++j)
    {
      auto e = ecm.CreateEntity();
      ecm.CreateComponent<Pose>(e, Pose());
      ecm.CreateComponent<LinearVelocity>(e, LinearVelocity());
      if (j % 2 == 0)
        ecm.CreateComponent<Name>(e, Name(std::string("e")));
    }
  }
  for (auto _ : _state)
  {
    ecm.Each<Pose, LinearVelocity>(
        [](const gz::sim::Entity &, Pose *p, LinearVelocity *v) -> bool
        {
          v->Data().X() = p->Data().Pos().X() + 1.0;
          v->Data().Y() = p->Data().Pos().Y();
          return true;
        });
  }
  _state.SetItemsProcessed(_state.iterations() * 1000);
}
BENCHMARK(BM_LegacyRobotMix50)->Unit(benchmark::kMicrosecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
