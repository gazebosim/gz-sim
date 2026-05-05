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

// Head-to-head bench: gz::sim::EntityComponentManager (the public
// ECM facade) versus gz::sim::ecs::World (the archetype core
// directly), running the same workload.
//
// Under GZ_SIM_ARCHETYPE_ECM=ON the public facade is itself
// archetype-backed, so the comparison measures *facade overhead*
// (legacy↔core handle translation, shadow-store maintenance) on
// top of the same underlying storage. Under
// GZ_SIM_ARCHETYPE_ECM=OFF the archetype benches don't build, so
// this file always runs in the archetype configuration.

#include <benchmark/benchmark.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include "gz/sim/ecs/World.hh"

using gz::sim::components::LinearVelocity;
using gz::sim::components::Pose;

// Public ECM facade — iteration. Build once, measure Each.
static void BM_FacadeEach(benchmark::State &_state)
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
BENCHMARK(BM_FacadeEach)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// Public ECM facade — build path.
static void BM_FacadeBuild(benchmark::State &_state)
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
    }
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_FacadeBuild)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// Direct ecs::World use — iteration. Build once, measure Each.
static void BM_ArchetypeWorldEach(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  gz::sim::ecs::World w(1);
  for (int64_t i = 0; i < N; ++i)
  {
    w.Create(
        Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
        LinearVelocity(gz::math::Vector3d(0, 0, 0)));
  }
  // Warm.
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
BENCHMARK(BM_ArchetypeWorldEach)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

// Direct ecs::World use — build path.
static void BM_ArchetypeWorldBuild(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    gz::sim::ecs::World w(1);
    for (int64_t i = 0; i < N; ++i)
    {
      w.Create(
          Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
          LinearVelocity(gz::math::Vector3d(0, 0, 0)));
    }
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeWorldBuild)
  ->Arg(100000)
  ->Arg(1000000)
  ->Unit(benchmark::kMillisecond);

#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
