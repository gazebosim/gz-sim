#include <benchmark/benchmark.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

using namespace gz;
using namespace sim;

void BM_ECM_Each(benchmark::State& state) {
  EntityComponentManager ecm;
  int numEntities = state.range(0);

  for (int i = 0; i < numEntities; ++i) {
    auto entity = ecm.CreateEntity();
    ecm.CreateComponent(entity, components::Name("entity_" + std::to_string(i)));
    ecm.CreateComponent(entity, components::Pose(math::Pose3d(i, 0, 0, 0, 0, 0)));
  }

  // Warmup - ensure view is created
  ecm.Each<components::Name, components::Pose>(
      [&](const Entity &, const components::Name *, const components::Pose *) {
        return true;
      });

  for (auto _ : state) {
    ecm.Each<components::Name, components::Pose>(
      [&](const Entity &, const components::Name *name, const components::Pose *pose) {
        benchmark::DoNotOptimize(name);
        benchmark::DoNotOptimize(pose);
        return true;
      });
  }
}

BENCHMARK(BM_ECM_Each)->Range(100, 10000);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
BENCHMARK_MAIN();
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
