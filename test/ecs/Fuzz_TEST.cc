/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Random-sequence fuzz test with an unordered_map-backed oracle. Any
 * divergence between the archetype-ECS result and the oracle fails.
 */
#include <gtest/gtest.h>
#include <map>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct A { double v; };
  struct B { int   v; };
  struct C { float v; };

  // Oracle: per-entity optional values for each component type.
  struct OracleEntity
  {
    std::optional<A> a;
    std::optional<B> b;
    std::optional<C> c;
  };
}

TEST(Fuzz, RandomSequenceMatchesOracle)
{
  // Make reproducible: seed printed on failure.
  constexpr uint32_t kSeed = 0xDEADBEEFu;
  std::mt19937 rng(kSeed);

  World w(1);
  std::unordered_map<uint64_t, OracleEntity> oracle;
  std::vector<Entity> alive;

  auto pick_alive = [&]() -> Entity
  {
    if (alive.empty()) return kNullEntity;
    std::uniform_int_distribution<size_t> d(0, alive.size() - 1);
    return alive[d(rng)];
  };

  const int kOps = 5000;
  for (int step = 0; step < kOps; ++step)
  {
    int op = std::uniform_int_distribution<int>(0, 9)(rng);
    switch (op)
    {
      case 0:   // Create{} + random subset
      case 1:
      case 2:
      {
        bool ha = std::bernoulli_distribution(0.7)(rng);
        bool hb = std::bernoulli_distribution(0.5)(rng);
        bool hc = std::bernoulli_distribution(0.3)(rng);
        Entity e;
        OracleEntity oe;
        if (ha && hb && hc) { A a{1.0}; B b{2}; C c{3.0f};
          e = w.Create(a, b, c); oe.a=a; oe.b=b; oe.c=c; }
        else if (ha && hb) { A a{1.0}; B b{2};
          e = w.Create(a, b); oe.a=a; oe.b=b; }
        else if (ha && hc) { A a{1.0}; C c{3.0f};
          e = w.Create(a, c); oe.a=a; oe.c=c; }
        else if (hb && hc) { B b{2}; C c{3.0f};
          e = w.Create(b, c); oe.b=b; oe.c=c; }
        else if (ha) { A a{1.0}; e = w.Create(a); oe.a=a; }
        else if (hb) { B b{2}; e = w.Create(b); oe.b=b; }
        else if (hc) { C c{3.0f}; e = w.Create(c); oe.c=c; }
        else         { e = w.CreateEmpty(); }
        alive.push_back(e);
        oracle[e.Raw()] = oe;
        break;
      }
      case 3:   // Add A
      case 4:
      {
        Entity e = pick_alive();
        if (e == kNullEntity) break;
        double val = std::uniform_real_distribution<double>()(rng);
        w.Add(e, A{val});
        oracle[e.Raw()].a = A{val};
        break;
      }
      case 5:   // Add B
      {
        Entity e = pick_alive();
        if (e == kNullEntity) break;
        int val = std::uniform_int_distribution<int>()(rng);
        w.Add(e, B{val});
        oracle[e.Raw()].b = B{val};
        break;
      }
      case 6:   // Remove A
      {
        Entity e = pick_alive();
        if (e == kNullEntity) break;
        w.Remove<A>(e);
        oracle[e.Raw()].a.reset();
        break;
      }
      case 7:   // Remove B
      {
        Entity e = pick_alive();
        if (e == kNullEntity) break;
        w.Remove<B>(e);
        oracle[e.Raw()].b.reset();
        break;
      }
      case 8:   // Destroy
      {
        if (alive.empty()) break;
        size_t i = std::uniform_int_distribution<size_t>(0,
            alive.size() - 1)(rng);
        Entity e = alive[i];
        w.Destroy(e);
        oracle.erase(e.Raw());
        alive[i] = alive.back();
        alive.pop_back();
        break;
      }
      case 9:   // Modify A
      {
        Entity e = pick_alive();
        if (e == kNullEntity) break;
        auto *p = w.Modify<A>(e);
        if (p)
        {
          p->v = 7.0;
          oracle[e.Raw()].a->v = 7.0;
        }
        break;
      }
    }
  }

  // Verify every remaining live entity matches the oracle.
  EXPECT_EQ(w.NumEntities(), oracle.size());
  for (auto &[raw, oe] : oracle)
  {
    Entity e = Entity::FromRaw(raw);
    ASSERT_TRUE(w.IsAlive(e)) << "seed=" << kSeed << " raw=" << raw;
    EXPECT_EQ(w.Has<A>(e), oe.a.has_value()) << "seed=" << kSeed;
    EXPECT_EQ(w.Has<B>(e), oe.b.has_value()) << "seed=" << kSeed;
    EXPECT_EQ(w.Has<C>(e), oe.c.has_value()) << "seed=" << kSeed;
    if (oe.a)
      EXPECT_DOUBLE_EQ(w.Component<A>(e)->v, oe.a->v) << "seed=" << kSeed;
    if (oe.b)
      EXPECT_EQ(w.Component<B>(e)->v, oe.b->v) << "seed=" << kSeed;
    if (oe.c)
      EXPECT_FLOAT_EQ(w.Component<C>(e)->v, oe.c->v) << "seed=" << kSeed;
  }
}
