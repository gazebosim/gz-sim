/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <benchmark/benchmark.h>

#include <memory>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Name.hh"

using namespace ignition;
using namespace gazebo;

constexpr const int kEachIterations {100};

class EntityComponentManagerFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State& _state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto matchingEntityCount = _state.range(0);
    auto nonmatchingEntityCount = _state.range(1);
    this->Populate(matchingEntityCount, nonmatchingEntityCount);
  }

  protected: void Populate(int _matchingEntityCount, int _nonmatchingEntityCount)
  {
    for (int i = 0; i < _matchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, components::World());
      mgr->CreateComponent(worldEntity, components::Name("world_name"));
    }

    for (int i = 0; i < _nonmatchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, components::Name("world_name"));
    }
  }

  std::unique_ptr<EntityComponentManager> mgr;
};

BENCHMARK_DEFINE_F(EntityComponentManagerFixture, EachNoCache)(benchmark::State& _st)
{
  for (auto _ : _st)
  {
    auto matchingEntityCount = _st.range(0);
    _st.counters.insert({
        {"matching", _st.range(0)},
        {"nonmatching", _st.range(1)}});

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->EachNoCache<components::World, components::Name>(
          [&](const Entity &, const components::World *,
            const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != matchingEntityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EntityComponentManagerFixture, EachCache)(benchmark::State& _st)
{
  for (auto _ : _st)
  {
    auto matchingEntityCount = _st.range(0);
    _st.counters.insert({
        {"matching", _st.range(0)},
        {"nonmatching", _st.range(1)}});

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::World, components::Name>(
          [&](const Entity &, const components::World *,
            const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != matchingEntityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

/// Method to generate test argument combinations.  google/benchmark does
// powers of 2 by default, which looks kind of ugly.
static void EachTestArgs(benchmark::internal::Benchmark* _b)
{
  int maxEntityCount = 1000;
  int step = maxEntityCount/5;

  for (int matchingEntityCount = 0; matchingEntityCount <= maxEntityCount;
       matchingEntityCount += step)
  {
    for (int nonmatchingEntityCount = 0;
         nonmatchingEntityCount <= maxEntityCount;
         nonmatchingEntityCount += step)
    {
      _b->Args({matchingEntityCount, nonmatchingEntityCount});
    }
  }
}

BENCHMARK_REGISTER_F(EntityComponentManagerFixture, EachNoCache)
  ->Unit(benchmark::kMillisecond)
  ->Apply(EachTestArgs);

BENCHMARK_REGISTER_F(EntityComponentManagerFixture, EachCache)
  ->Unit(benchmark::kMillisecond)
  ->Apply(EachTestArgs);

