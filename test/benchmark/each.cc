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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

#ifdef BENCHMARK_INTERNAL
using Benchmark = ::benchmark::internal::Benchmark;
#else
using Benchmark = ::benchmark::Benchmark;
#endif

using namespace gz;
using namespace sim;
using namespace components;

constexpr const int kEachIterations {100};

class EntityComponentManagerFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto matchingEntityCount = _state.range(0);
    auto nonmatchingEntityCount = _state.range(1);
    this->Populate(matchingEntityCount, nonmatchingEntityCount);
  }

  protected: void Populate(int _matchingEntityCount,
                           int _nonmatchingEntityCount)
  {
    for (int i = 0; i < _matchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, World());
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

BENCHMARK_DEFINE_F(EntityComponentManagerFixture, EachCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto matchingEntityCount = _st.range(0);
    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<World, components::Name>(
          [&](const Entity &, const World *, const components::Name *)->bool
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

class ManyComponentFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto entityCount = _state.range(0);
    this->Populate(entityCount);
  }

  protected: void Populate(int _entityCount)
  {
    for (int i = 0; i < _entityCount; ++i)
    {
      Entity entity = mgr->CreateEntity();
      mgr->CreateComponent(entity, components::Name("world_name"));
      mgr->CreateComponent(entity, AngularVelocity());
      mgr->CreateComponent(entity, WorldAngularVelocity());
      mgr->CreateComponent(entity, Inertial());
      mgr->CreateComponent(entity, LinearAcceleration());
      mgr->CreateComponent(entity, WorldLinearAcceleration());
      mgr->CreateComponent(entity, LinearVelocity());
      mgr->CreateComponent(entity, WorldLinearVelocity());
      mgr->CreateComponent(entity, Pose());
      mgr->CreateComponent(entity, WorldPose());
    }
  }

  std::unique_ptr<EntityComponentManager> mgr;

  protected: EntityComponentManager* Mgr()
  {
    return mgr.get();
  }

  protected: const EntityComponentManager* MgrConst()
  {
    return mgr.get();
  }
};

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name>(
          [&](const Entity &, const components::Name *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each5ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name,
                AngularVelocity,
                Inertial,
                LinearAcceleration,
                LinearVelocity>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const LinearVelocity *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each10ComponentCache)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name,
                AngularVelocity,
                WorldAngularVelocity,
                Inertial,
                LinearAcceleration,
                WorldLinearAcceleration,
                LinearVelocity,
                WorldLinearVelocity,
                Pose,
                WorldPose>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const WorldAngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const WorldLinearAcceleration *,
              const LinearVelocity *,
              const WorldLinearVelocity *,
              const Pose *,
              const WorldPose *)->bool
          {
            entitiesMatched++;
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each5ComponentConst)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      const auto manager = this->MgrConst();
      manager->Each<components::Name,
                AngularVelocity,
                Inertial,
                LinearAcceleration,
                LinearVelocity>(
          [&](const Entity &,
              const components::Name *,
              const AngularVelocity *,
              const Inertial *,
              const LinearAcceleration *,
              const LinearVelocity *)->bool
          {
            entitiesMatched++;
            return true;
          });
      // Flush the enqueued groups so following iterations are much faster
      this->mgr->CreatePendingGroups();

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

/// \brief Models a fleet of robots that each step write a few components and
/// mark them as changed, then end the step as SimulationRunner does. Each robot
/// has 12 entities (model, 4 links, 4 collisions, 3 joints), created in SDF
/// order, and marks the model Pose and two joint JointPositions per step.
class SetChangedFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    this->mgr = std::make_unique<EntityComponentManager>();
    this->models.clear();
    this->joints.clear();
    this->Populate(static_cast<int>(_state.range(0)));
    // Entities are not new after the first step.
    this->EndStep();
  }

  protected: void TearDown(const ::benchmark::State &) override
  {
    this->mgr.reset();
  }

  protected: void Populate(int _robotCount)
  {
    const Entity world = this->mgr->CreateEntity();
    this->mgr->CreateComponent(world, World());
    this->mgr->CreateComponent(world, components::Name("world"));
    for (int r = 0; r < _robotCount; ++r)
    {
      const Entity model = this->mgr->CreateEntity();
      this->mgr->CreateComponent(model, components::Model());
      this->mgr->CreateComponent(model,
          components::Name("robot" + std::to_string(r)));
      this->mgr->CreateComponent(model, Pose());
      this->mgr->CreateComponent(model, ParentEntity(world));
      this->models.push_back(model);
      for (int l = 0; l < 4; ++l)
      {
        const Entity link = this->mgr->CreateEntity();
        this->mgr->CreateComponent(link, components::Link());
        this->mgr->CreateComponent(link,
            components::Name("link" + std::to_string(l)));
        this->mgr->CreateComponent(link, Pose());
        this->mgr->CreateComponent(link, ParentEntity(model));
        const Entity collision = this->mgr->CreateEntity();
        this->mgr->CreateComponent(collision, components::Collision());
        this->mgr->CreateComponent(collision, components::Name("collision"));
        this->mgr->CreateComponent(collision, Pose());
        this->mgr->CreateComponent(collision, ParentEntity(link));
      }
      for (int j = 0; j < 3; ++j)
      {
        const Entity joint = this->mgr->CreateEntity();
        this->mgr->CreateComponent(joint, components::Joint());
        this->mgr->CreateComponent(joint,
            components::Name("joint" + std::to_string(j)));
        this->mgr->CreateComponent(joint, ParentEntity(model));
        this->mgr->CreateComponent(joint, JointPosition({0.0}));
        // The first two joints are the driven wheels.
        if (j < 2)
          this->joints.push_back(joint);
      }
    }
  }

  /// \brief What SimulationRunner does at the end of a step.
  protected: void EndStep()
  {
    this->mgr->ClearNewlyCreatedEntities();
    this->mgr->ProcessRemoveEntityRequests();
    this->mgr->ClearRemovedComponents();
    this->mgr->SetAllComponentsUnchanged();
  }

  /// \brief Write the data of the three components of every robot.
  protected: void Write(double _value)
  {
    const math::Pose3d pose(_value, _value, 0, 0, 0, _value);
    for (std::size_t r = 0; r < this->models.size(); ++r)
    {
      for (const Entity joint : {this->joints[2 * r], this->joints[2 * r + 1]})
        this->mgr->Component<JointPosition>(joint)->Data().assign(1, _value);
      this->mgr->Component<Pose>(this->models[r])->Data() = pose;
    }
  }

  /// \brief Mark the three components of every robot as periodically changed.
  protected: void Mark()
  {
    for (std::size_t r = 0; r < this->models.size(); ++r)
    {
      for (const Entity joint : {this->joints[2 * r], this->joints[2 * r + 1]})
      {
        this->mgr->SetChanged(joint, JointPosition::typeId,
            ComponentState::PeriodicChange);
      }
      this->mgr->SetChanged(this->models[r], Pose::typeId,
          ComponentState::PeriodicChange);
    }
  }

  /// \brief Write over a buffer of _megabytes to push the ECM out of the
  /// cache, as the work of the other systems does between two steps.
  protected: static void EvictCache(std::size_t _megabytes)
  {
    static std::vector<char> buffer(256u << 20);
    const std::size_t size = std::min(buffer.size(), _megabytes << 20);
    for (std::size_t i = 0; i < size; i += 64)
      buffer[i] += 1;
    benchmark::DoNotOptimize(buffer.data());
    benchmark::ClobberMemory();
  }

  protected: std::unique_ptr<EntityComponentManager> mgr;
  protected: std::vector<Entity> models;
  protected: std::vector<Entity> joints;
};

/// \brief One step that writes the components, marks them changed and ends the
/// step. Range 0 is the robot count, range 1 the megabytes written between
/// steps to evict the ECM from the cache (0 keeps it warm). The step time is
/// measured without the eviction. Items are marked components.
BENCHMARK_DEFINE_F(SetChangedFixture, WriteMarkEndStep)
(benchmark::State &_st)
{
  using Clock = std::chrono::steady_clock;
  const auto evictMb = static_cast<std::size_t>(_st.range(1));
  double value = 0.0;
  for (auto _ : _st)
  {
    if (evictMb > 0)
      EvictCache(evictMb);
    value += 1e-3;
    const auto start = Clock::now();
    this->Write(value);
    this->Mark();
    this->EndStep();
    const auto end = Clock::now();
    _st.SetIterationTime(std::chrono::duration<double>(end - start).count());
  }
  _st.SetItemsProcessed(
      3 * static_cast<int64_t>(this->models.size()) * _st.iterations());
}

/// \brief The same step without marking, as a reference for the writes.
BENCHMARK_DEFINE_F(SetChangedFixture, WriteEndStep)
(benchmark::State &_st)
{
  using Clock = std::chrono::steady_clock;
  const auto evictMb = static_cast<std::size_t>(_st.range(1));
  double value = 0.0;
  for (auto _ : _st)
  {
    if (evictMb > 0)
      EvictCache(evictMb);
    value += 1e-3;
    const auto start = Clock::now();
    this->Write(value);
    this->EndStep();
    const auto end = Clock::now();
    _st.SetIterationTime(std::chrono::duration<double>(end - start).count());
  }
  _st.SetItemsProcessed(
      3 * static_cast<int64_t>(this->models.size()) * _st.iterations());
}

class EntityByComponentsFixture: public benchmark::Fixture
{
  protected: void SetUp(const ::benchmark::State &_state) override
  {
    mgr = std::make_unique<EntityComponentManager>();
    auto nonmatchingEntityCount = _state.range(0);
    this->Populate(nonmatchingEntityCount);
  }

  protected: void Populate(int _nonmatchingEntityCount)
  {
    // We add them first so EntityByComponents won't return immediately
    std::vector<Entity> nonMatchingEntities;
    nonMatchingEntities.reserve(_nonmatchingEntityCount);

    for (int i = 0; i < _nonmatchingEntityCount; ++i)
    {
      Entity worldEntity = mgr->CreateEntity();
      mgr->CreateComponent(worldEntity, World());
      mgr->CreateComponent(worldEntity, Inertial());
      mgr->CreateComponent(worldEntity, components::Name("child"));
      nonMatchingEntities.push_back(worldEntity);
    }

    for (int i = 0; i < matchingEntityCount; ++i)
    {
      Entity e = mgr->CreateEntity();
      if (parentEntity == kNullEntity)
      {
        parentEntity = e;
      }
      mgr->CreateComponent(e, Inertial());
      mgr->CreateComponent(e, components::Name("target"));
    }

    // Now make one of the matching entities parent of all the non matching to have
    // a parametrizably large parent entity
    for (const auto& e : nonMatchingEntities)
    {
      mgr->SetParentEntity(e, parentEntity);
    }
  }

  static constexpr int matchingEntityCount = 1;
  Entity parentEntity = kNullEntity;
  std::unique_ptr<EntityComponentManager> mgr;
};

BENCHMARK_DEFINE_F(EntityByComponentsFixture, EntityByComponents)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      const auto e = mgr->EntityByComponents(components::Name("target"));
      this->mgr->CreatePendingGroups();
      if (e == kNullEntity)
      {
        _st.SkipWithError("Entity not found");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EntityByComponentsFixture, EntitiesByComponents)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      const auto e = mgr->EntitiesByComponents(components::Name("target"));
      // Flush the enqueued groups so following iterations are much faster
      this->mgr->CreatePendingGroups();

      if (e.size() != EntityByComponentsFixture::matchingEntityCount)
      {
        _st.SkipWithError("Wrong name of entities found");
      }
    }
  }
}

BENCHMARK_DEFINE_F(EntityByComponentsFixture, ChildrenByComponents)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      const auto e = mgr->ChildrenByComponents(parentEntity, components::Name("child"));
      this->mgr->CreatePendingGroups();
      if (static_cast<int64_t>(e.size()) != _st.range(0))
      {
        _st.SkipWithError("Wrong name of children found");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentGet9More)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    for (int eachIter = 0; eachIter < kEachIterations; eachIter++)
    {
      int entitiesMatched = 0;

      mgr->Each<components::Name>(
          [&](const Entity& e,
              const components::Name *)->bool
          {
            if (mgr->Component<AngularVelocity>(e) != nullptr &&
                mgr->Component<WorldAngularVelocity>(e) != nullptr &&
                mgr->Component<Inertial>(e) != nullptr &&
                mgr->Component<LinearAcceleration>(e) != nullptr &&
                mgr->Component<WorldLinearAcceleration>(e) != nullptr &&
                mgr->Component<LinearVelocity>(e) != nullptr &&
                mgr->Component<WorldLinearVelocity>(e) != nullptr &&
                mgr->Component<Pose>(e) != nullptr &&
                mgr->Component<WorldPose>(e) != nullptr)
            {
              entitiesMatched++;
            }
            return true;
          });

      if (entitiesMatched != entityCount)
      {
        _st.SkipWithError("Failed to match correct number of entities");
      }
    }
  }
}

BENCHMARK_DEFINE_F(ManyComponentFixture, Each1ComponentRemove9)
(benchmark::State &_st)
{
  for (auto _ : _st)
  {
    auto entityCount = _st.range(0);

    int entitiesMatched = 0;

    mgr->Each<components::Name>(
        [&](const Entity& e,
            const components::Name *)->bool
        {
          if (mgr->RemoveComponent<AngularVelocity>(e) &&
              mgr->RemoveComponent<WorldAngularVelocity>(e) &&
              mgr->RemoveComponent<Inertial>(e) &&
              mgr->RemoveComponent<LinearAcceleration>(e) &&
              mgr->RemoveComponent<WorldLinearAcceleration>(e) &&
              mgr->RemoveComponent<LinearVelocity>(e) &&
              mgr->RemoveComponent<WorldLinearVelocity>(e) &&
              mgr->RemoveComponent<Pose>(e) &&
              mgr->RemoveComponent<WorldPose>(e))
          {
            entitiesMatched++;
            mgr->CreateComponent(e, AngularVelocity());
            mgr->CreateComponent(e, WorldAngularVelocity());
            mgr->CreateComponent(e, Inertial());
            mgr->CreateComponent(e, LinearAcceleration());
            mgr->CreateComponent(e, WorldLinearAcceleration());
            mgr->CreateComponent(e, LinearVelocity());
            mgr->CreateComponent(e, WorldLinearVelocity());
            mgr->CreateComponent(e, Pose());
            mgr->CreateComponent(e, WorldPose());
          }
          return true;
        });

    if (entitiesMatched != entityCount)
    {
      _st.SkipWithError("Failed to match correct number of entities");
    }
  }
}

/// Method to generate test argument combinations.  google/benchmark does
/// powers of 2 by default, which looks kind of ugly.
static void EachTestArgs(Benchmark *_b)
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

BENCHMARK_REGISTER_F(EntityComponentManagerFixture, EachCache)
  ->Unit(benchmark::kMillisecond)
  ->Apply(EachTestArgs);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each1ComponentCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each5ComponentConst)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each5ComponentCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each10ComponentCache)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each1ComponentGet9More)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(ManyComponentFixture, Each1ComponentRemove9)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(EntityByComponentsFixture, EntityByComponents)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Arg(10000)
  ->Arg(100000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(EntityByComponentsFixture, EntitiesByComponents)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

BENCHMARK_REGISTER_F(EntityByComponentsFixture, ChildrenByComponents)
  ->Arg(10)
  ->Arg(100)
  ->Arg(1000)
  ->Unit(benchmark::kMillisecond);

// The step is a few tens of microseconds, but evicting the cache takes much
// longer, so the iteration counts are fixed instead of left to the time target.
// 48 MB evicts a 32 MB L3 cache, 256 MB evicts a 96 MB one.
BENCHMARK_REGISTER_F(SetChangedFixture, WriteMarkEndStep)
  ->Args({500, 0})
  ->Iterations(2000)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

BENCHMARK_REGISTER_F(SetChangedFixture, WriteMarkEndStep)
  ->Args({500, 48})
  ->Iterations(300)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

BENCHMARK_REGISTER_F(SetChangedFixture, WriteMarkEndStep)
  ->Args({500, 256})
  ->Iterations(100)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

BENCHMARK_REGISTER_F(SetChangedFixture, WriteEndStep)
  ->Args({500, 0})
  ->Iterations(2000)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

BENCHMARK_REGISTER_F(SetChangedFixture, WriteEndStep)
  ->Args({500, 48})
  ->Iterations(300)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

BENCHMARK_REGISTER_F(SetChangedFixture, WriteEndStep)
  ->Args({500, 256})
  ->Iterations(100)
  ->UseManualTime()
  ->Unit(benchmark::kMicrosecond);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
