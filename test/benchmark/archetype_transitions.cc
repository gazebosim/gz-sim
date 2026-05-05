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

// Component add/remove benchmarks comparing the archetype core
// against the public `gz::sim::EntityComponentManager`.
//
// Adding or removing a component on the archetype side moves the
// entity to a different archetype (look up destination, copy each
// shared column across, swap-with-last on the source). On the legacy
// side, the same operation is just a hash insertion or erasure in
// the per-type map. Two qualitatively different write paths — this
// bench measures the per-call cost of each.
//
// `gz::sim::components::Visual` is used as the tag component; it's
// a `Component<NoData, ...>` with no payload, which matches the
// "tag" workload the original phase-0a bench was after.
//
// What "Legacy" measures depends on the build flag:
//   * GZ_SIM_ARCHETYPE_ECM=OFF — BM_Legacy* runs against the
//     legacy unordered_map ECM (Add = hash-insert, Remove =
//     hash-erase). Read this run for true legacy-vs-archetype.
//   * GZ_SIM_ARCHETYPE_ECM=ON  — the public ECM is itself
//     archetype-backed, so BM_Legacy* measures the facade
//     overhead on top of the archetype core. Read this run for
//     facade-vs-direct comparison.

#include <benchmark/benchmark.h>

#include <vector>

#include <gz/math/Pose3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

#include "gz/sim/ecs/World.hh"

using gz::sim::components::Pose;
using gz::sim::components::Visual;

// ---------------------------------------------------------------------------
// Archetype core (gz::sim::ecs::World)
// ---------------------------------------------------------------------------

// Baseline iteration over N entities — the reference cost the
// per-transition benches are read against.
static void BM_ArchetypeEachBaseline(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  gz::sim::ecs::World w(1);
  for (int64_t i = 0; i < N; ++i)
    w.Create(Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));

  for (auto _ : _state)
  {
    w.Each<Pose>([](gz::sim::ecs::Entity, Pose &) {});
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeEachBaseline)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMicrosecond);

// Add a Visual tag to N entities. Each entity transitions from
// {Pose} -> {Pose, Visual}. PauseTiming/ResumeTiming bracket the
// per-iteration setup so only the Add work is measured.
static void BM_ArchetypeAddTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    gz::sim::ecs::World w(1);
    std::vector<gz::sim::ecs::Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
      es.push_back(w.Create(
          Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0))));
    _state.ResumeTiming();

    for (auto e : es) w.Add(e, Visual{});
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeAddTag)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

// Remove a Visual tag from N entities. Setup creates the entities
// WITH the tag already present so the timed body is purely the
// Remove work.
static void BM_ArchetypeRemoveTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    gz::sim::ecs::World w(1);
    std::vector<gz::sim::ecs::Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
    {
      auto e = w.Create(Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
      w.Add(e, Visual{});
      es.push_back(e);
    }
    _state.ResumeTiming();

    for (auto e : es) w.Remove<Visual>(e);
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_ArchetypeRemoveTag)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------------------------
// Legacy / public-facade comparison benches
// ---------------------------------------------------------------------------

static void BM_LegacyEachBaseline(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  gz::sim::EntityComponentManager ecm;
  for (int64_t i = 0; i < N; ++i)
  {
    auto e = ecm.CreateEntity();
    ecm.CreateComponent<Pose>(e, Pose(
        gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
  }
  for (auto _ : _state)
  {
    ecm.Each<Pose>(
        [](const gz::sim::Entity &, Pose *) -> bool { return true; });
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_LegacyEachBaseline)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMicrosecond);

static void BM_LegacyAddTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    gz::sim::EntityComponentManager ecm;
    std::vector<gz::sim::Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
    {
      auto e = ecm.CreateEntity();
      ecm.CreateComponent<Pose>(e, Pose(
          gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
      es.push_back(e);
    }
    _state.ResumeTiming();

    for (auto e : es) ecm.CreateComponent<Visual>(e, Visual());
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_LegacyAddTag)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

static void BM_LegacyRemoveTag(benchmark::State &_state)
{
  const int64_t N = _state.range(0);
  for (auto _ : _state)
  {
    _state.PauseTiming();
    gz::sim::EntityComponentManager ecm;
    std::vector<gz::sim::Entity> es;
    es.reserve(N);
    for (int64_t i = 0; i < N; ++i)
    {
      auto e = ecm.CreateEntity();
      ecm.CreateComponent<Pose>(e, Pose(
          gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
      ecm.CreateComponent<Visual>(e, Visual());
      es.push_back(e);
    }
    _state.ResumeTiming();

    for (auto e : es) ecm.RemoveComponent(e, Visual::typeId);
  }
  _state.SetItemsProcessed(_state.iterations() * N);
}
BENCHMARK(BM_LegacyRemoveTag)
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
