#include <benchmark/benchmark.h>

#include <string>
#include <vector>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;

void PopulateDeepHierarchy(EntityComponentManager &_ecm, int _modelCount,
    std::vector<Entity> &_collisions)
{
  Entity world = _ecm.CreateEntity();
  _ecm.CreateComponent(world, components::World());
  _ecm.CreateComponent(world, components::Name("world"));

  for (int i = 0; i < _modelCount; ++i)
  {
    const std::string suffix = std::to_string(i);

    Entity outerModel = _ecm.CreateEntity();
    _ecm.CreateComponent(outerModel, components::Model());
    _ecm.CreateComponent(outerModel, components::Name("model_" + suffix));
    _ecm.CreateComponent(outerModel, components::ParentEntity(world));

    Entity innerModel = _ecm.CreateEntity();
    _ecm.CreateComponent(innerModel, components::Model());
    _ecm.CreateComponent(innerModel, components::Name("nested_" + suffix));
    _ecm.CreateComponent(innerModel, components::ParentEntity(outerModel));

    Entity link = _ecm.CreateEntity();
    _ecm.CreateComponent(link, components::Link());
    _ecm.CreateComponent(link, components::Name("link_" + suffix));
    _ecm.CreateComponent(link, components::ParentEntity(innerModel));

    Entity collision = _ecm.CreateEntity();
    _ecm.CreateComponent(collision, components::Collision());
    _ecm.CreateComponent(collision, components::Name("collision_" + suffix));
    _ecm.CreateComponent(collision, components::ParentEntity(link));

    _collisions.push_back(collision);
  }
}

void BM_ScopedNameDeepHierarchy(benchmark::State &_st)
{
  EntityComponentManager ecm;
  std::vector<Entity> collisions;
  PopulateDeepHierarchy(ecm, static_cast<int>(_st.range(0)), collisions);

  for (auto _ : _st)
  {
    for (const auto &collision : collisions)
    {
      benchmark::DoNotOptimize(scopedName(collision, ecm, "::", false));
    }
  }

  _st.SetItemsProcessed(
      static_cast<int64_t>(_st.iterations() * collisions.size()));
}

void BM_EntityTypeStr(benchmark::State &_st)
{
  EntityComponentManager ecm;
  std::vector<Entity> collisions;
  PopulateDeepHierarchy(ecm, static_cast<int>(_st.range(0)), collisions);

  for (auto _ : _st)
  {
    for (const auto &collision : collisions)
    {
      Entity e = collision;
      while (e != kNullEntity)
      {
        benchmark::DoNotOptimize(entityTypeStr(e, ecm));
        auto p = ecm.Component<components::ParentEntity>(e);
        e = p ? p->Data() : kNullEntity;
      }
    }
  }
}

BENCHMARK(BM_ScopedNameDeepHierarchy)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK(BM_EntityTypeStr)
    ->Arg(10)
    ->Arg(100)
    ->Arg(1000)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_MAIN();
