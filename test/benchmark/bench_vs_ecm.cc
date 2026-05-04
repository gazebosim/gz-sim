/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Head-to-head bench: gz::sim::EntityComponentManager (classic) vs
 * gz::sim::ecs::World (Phase 0a archetype ECS) on the same workload.
 *
 * Same component types, same entity count, same write pattern — just
 * different storage. Prints ns/entity for each and the speedup ratio.
 */
#include <cstdio>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include "bench_common.hh"
#include "gz/sim/ecs/World.hh"

using gz::sim::components::LinearVelocity;
using gz::sim::components::Pose;

namespace
{
  struct Result
  {
    double build_s;
    double iter_s;
    uint64_t iter_items;
  };

  Result RunEcm(size_t _n, int _iters)
  {
    gz::sim::EntityComponentManager ecm;
    gz::sim::ecs::bench::Timer t;

    t.Reset();
    for (size_t i = 0; i < _n; ++i)
    {
      auto e = ecm.CreateEntity();
      ecm.CreateComponent<Pose>(e, Pose(
          gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)));
      ecm.CreateComponent<LinearVelocity>(e, LinearVelocity(
          gz::math::Vector3d(0, 0, 0)));
    }
    double build = t.Seconds();

    // Warm.
    ecm.Each<Pose, LinearVelocity>(
        [](const gz::sim::Entity &, Pose *p, LinearVelocity *v) -> bool
    {
      v->Data().X() = p->Data().Pos().X();
      return true;
    });

    t.Reset();
    for (int k = 0; k < _iters; ++k)
    {
      ecm.Each<Pose, LinearVelocity>(
          [](const gz::sim::Entity &, Pose *p, LinearVelocity *v) -> bool
      {
        v->Data().X() = p->Data().Pos().X() * 0.5;
        return true;
      });
    }
    double iter = t.Seconds();
    return {build, iter, _n * static_cast<uint64_t>(_iters)};
  }

  Result RunArchetype(size_t _n, int _iters)
  {
    gz::sim::ecs::World w(1);
    gz::sim::ecs::bench::Timer t;

    t.Reset();
    for (size_t i = 0; i < _n; ++i)
    {
      w.Create(
          Pose(gz::math::Pose3d(double(i), 0, 0, 0, 0, 0)),
          LinearVelocity(gz::math::Vector3d(0, 0, 0)));
    }
    double build = t.Seconds();

    // Warm.
    w.Each<Pose, LinearVelocity>(
        [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
    {
      v.Data().X() = p.Data().Pos().X();
    });

    t.Reset();
    for (int k = 0; k < _iters; ++k)
    {
      w.Each<Pose, LinearVelocity>(
          [](gz::sim::ecs::Entity, Pose &p, LinearVelocity &v)
      {
        v.Data().X() = p.Data().Pos().X() * 0.5;
      });
    }
    double iter = t.Seconds();
    return {build, iter, _n * static_cast<uint64_t>(_iters)};
  }
}

int main(int argc, char **argv)
{
  size_t n = 1'000'000;
  int iters = 50;
  if (argc > 1) n = std::stoull(argv[1]);
  if (argc > 2) iters = std::stoi(argv[2]);

  std::printf("== Head-to-head: classic ECM vs Phase 0a archetype ECS ==\n");
  std::printf("   entities = %zu, iterations = %d\n", n, iters);

  auto ecm  = RunEcm(n, iters);
  auto arch = RunArchetype(n, iters);

  std::printf("\nBuild (Create+CreateComponent/Create):\n");
  std::printf("  ECM       : %.3f s  (%.1f ns/entity)\n",
      ecm.build_s,  ecm.build_s  / n * 1e9);
  std::printf("  Archetype : %.3f s  (%.1f ns/entity)\n",
      arch.build_s, arch.build_s / n * 1e9);
  std::printf("  Ratio (ECM build / Arch build): %.2f×\n",
      ecm.build_s / arch.build_s);

  std::printf("\nIteration (Each<Pose, LinearVelocity>, %d passes):\n", iters);
  std::printf("  ECM       : %.3f ms  (%.2f ns/entity)\n",
      ecm.iter_s  * 1000.0, ecm.iter_s  / ecm.iter_items  * 1e9);
  std::printf("  Archetype : %.3f ms  (%.2f ns/entity)\n",
      arch.iter_s * 1000.0, arch.iter_s / arch.iter_items * 1e9);

  double ecm_ns_per  = ecm.iter_s  / ecm.iter_items;
  double arch_ns_per = arch.iter_s / arch.iter_items;
  std::printf("  Speedup   : %.2f×  (gate: ≥ 5× for microbench_10m)\n",
      ecm_ns_per / arch_ns_per);
  return 0;
}
