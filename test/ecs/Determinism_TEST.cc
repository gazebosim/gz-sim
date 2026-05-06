/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */

// Determinism test suite for the archetype ECS core.
//
// doc/archetype_ecs_architecture.md ("Determinism" section) claims:
//   1. Stable entity IDs — fresh world + same creation order =>
//      identical Entity handles across runs.
//   2. Stable generation increments — destroy + recreate produces
//      a deterministic sequence of generations.
//   3. Stable archetype IDs — first-sighting order is preserved.
//   4. Iteration order — `Each<...>` walks archetypes / chunks /
//      rows in a deterministic order across runs.
//   5. Iteration order after SwapRemove — the swap-with-last shuffle
//      is itself deterministic given the same remove sequence.
//   6. Parallel determinism — `EachParallel` partitions and merges
//      deterministically given a fixed worker count.
//
// Each test below probes one of those claims by running the same
// scripted sequence twice (sometimes against fresh process state)
// and asserting bit-identical output.

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <set>
#include <thread>
#include <vector>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose     { double x, y, z; };
  struct Velocity { double vx, vy, vz; };
  struct Name     { char value[32]; };
  struct Tag      {};

  // Build a deterministic world from a scripted sequence. Returns
  // the entity handles in creation order.
  std::vector<Entity> ScriptedBuild(World &_w, int _n)
  {
    std::vector<Entity> out;
    out.reserve(_n);
    for (int i = 0; i < _n; ++i)
    {
      // Vary archetype to exercise multiple type sets:
      //   i % 3 == 0  ->  {Pose, Velocity, Name}
      //   i % 3 == 1  ->  {Pose, Velocity}
      //   i % 3 == 2  ->  {Pose}
      Entity e;
      switch (i % 3)
      {
        case 0:
          e = _w.Create(
              Pose{double(i), 0, 0},
              Velocity{double(i) * 0.5, 0, 0},
              Name{});
          break;
        case 1:
          e = _w.Create(
              Pose{double(i), 0, 0},
              Velocity{double(i) * 0.5, 0, 0});
          break;
        default:
          e = _w.Create(Pose{double(i), 0, 0});
          break;
      }
      out.push_back(e);
    }
    return out;
  }

  // FNV-1a digest over the world's iteration output. The hash
  // covers every (Entity, Pose, Velocity) triple in iteration
  // order — any change in iteration order, entity identity, or
  // component value flips the digest.
  uint64_t IterationDigest(World &_w)
  {
    uint64_t h = 0xcbf29ce484222325ull;
    auto mix = [&h](const void *_p, size_t _n)
    {
      auto *b = static_cast<const unsigned char *>(_p);
      for (size_t i = 0; i < _n; ++i)
      {
        h ^= b[i];
        h *= 0x100000001b3ull;
      }
    };
    _w.Each<const Pose, const Velocity>(
        [&](Entity _e, const Pose &_p, const Velocity &_v)
        {
          uint64_t er = _e.Raw();
          mix(&er, sizeof(er));
          mix(&_p, sizeof(_p));
          mix(&_v, sizeof(_v));
        });
    return h;
  }
}

// -----------------------------------------------------------------
// 1. Stable entity IDs — same creation order, fresh world, twice.
// -----------------------------------------------------------------
TEST(Determinism, EntityIdsStableAcrossFreshWorlds)
{
  auto run_once = []()
  {
    World w(1);
    return ScriptedBuild(w, 200);
  };

  auto a = run_once();
  auto b = run_once();
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_EQ(a[i].Raw(), b[i].Raw())
        << "entity " << i << " differs: a=" << a[i].Raw()
        << ", b=" << b[i].Raw();
  }
}

// -----------------------------------------------------------------
// 2. Generation counter — deterministic destroy + recreate sequence.
// -----------------------------------------------------------------
TEST(Determinism, EntityGenerationsStableAcrossRuns)
{
  auto run_once = []()
  {
    World w(1);
    std::vector<Entity> es;
    es.reserve(100);
    for (int i = 0; i < 100; ++i)
      es.push_back(w.Create(Pose{double(i), 0, 0}));
    // Destroy every other entity, then recreate. Free-list LIFO
    // means the recreated entities should reuse the most recently
    // freed slots in a deterministic order, with their generations
    // bumped.
    for (size_t i = 0; i < es.size(); i += 2)
      w.Destroy(es[i]);
    std::vector<Entity> es2;
    for (int i = 0; i < 50; ++i)
      es2.push_back(w.Create(Pose{double(1000 + i), 0, 0}));
    return es2;
  };

  auto a = run_once();
  auto b = run_once();
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_EQ(a[i].Raw(), b[i].Raw())
        << "recreated entity " << i << " differs (index "
        << "+ generation): a=" << a[i].Raw() << ", b=" << b[i].Raw();
  }
}

// -----------------------------------------------------------------
// 3. Stable archetype IDs — first-sighting order preserved.
// -----------------------------------------------------------------
TEST(Determinism, ArchetypeIdsStableAcrossFreshWorlds)
{
  auto run_once = []() -> std::vector<size_t>
  {
    World w(1);
    std::vector<size_t> archetype_count_progression;
    // Empty world: archetype 0 (the empty archetype) is pre-allocated.
    archetype_count_progression.push_back(w.NumArchetypes());
    // Trigger archetype creations in a known order.
    w.Create(Pose{}, Velocity{}, Name{});
    archetype_count_progression.push_back(w.NumArchetypes());
    w.Create(Pose{});
    archetype_count_progression.push_back(w.NumArchetypes());
    w.Create(Pose{}, Velocity{});
    archetype_count_progression.push_back(w.NumArchetypes());
    // Re-creating the same shape doesn't allocate a new archetype.
    w.Create(Pose{}, Velocity{}, Name{});
    archetype_count_progression.push_back(w.NumArchetypes());
    return archetype_count_progression;
  };

  auto a = run_once();
  auto b = run_once();
  EXPECT_EQ(a, b);
  // Sanity: the progression should be 1, 2, 3, 4, 4 (the last create
  // hits an existing archetype).
  ASSERT_EQ(a.size(), 5u);
  EXPECT_EQ(a[0], 1u);  // empty archetype only
  EXPECT_EQ(a[1], 2u);  // + {Pose, Velocity, Name}
  EXPECT_EQ(a[2], 3u);  // + {Pose}
  EXPECT_EQ(a[3], 4u);  // + {Pose, Velocity}
  EXPECT_EQ(a[4], 4u);  // re-hit
}

// -----------------------------------------------------------------
// 4. Iteration order — `Each` walks the world deterministically.
// -----------------------------------------------------------------
TEST(Determinism, IterationOrderStableAcrossFreshWorlds)
{
  auto run_once = []()
  {
    World w(1);
    ScriptedBuild(w, 500);
    return IterationDigest(w);
  };
  uint64_t a = run_once();
  uint64_t b = run_once();
  uint64_t c = run_once();
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
}

// -----------------------------------------------------------------
// 5. Iteration order after SwapRemove — destroy a sparse subset,
//    then iterate. The swap-with-last shuffle should produce a
//    deterministic post-remove iteration.
// -----------------------------------------------------------------
TEST(Determinism, IterationOrderStableAfterRemoves)
{
  auto run_once = []()
  {
    World w(1);
    auto es = ScriptedBuild(w, 500);
    // Destroy every 7th entity — exercises non-tail removes that
    // trigger the SwapRemove relocation path.
    for (size_t i = 0; i < es.size(); i += 7)
      w.Destroy(es[i]);
    return IterationDigest(w);
  };
  uint64_t a = run_once();
  uint64_t b = run_once();
  uint64_t c = run_once();
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
}

// -----------------------------------------------------------------
// 6a. Parallel determinism — pure-read EachParallel.
// -----------------------------------------------------------------
TEST(Determinism, EachParallelReadOnlyStable)
{
  // Build once, run EachParallel-with-write twice, compare digests.
  // The same World instance is used so we're measuring whether
  // concurrent execution introduces any per-run divergence.
  World w(4);
  ScriptedBuild(w, 5000);

  auto compute_pass = [&]()
  {
    w.EachParallel<const Pose, Velocity>(
        [](Entity, const Pose &_p, Velocity &_v)
        {
          // Pure entity-local computation — non-determinism would
          // have to come from the framework, not the user code.
          _v.vx = _p.x * 3.0 + 1.0;
          _v.vy = _p.x - 2.5;
          _v.vz = _p.x * _p.x;
        });
    return IterationDigest(w);
  };
  uint64_t a = compute_pass();
  uint64_t b = compute_pass();
  uint64_t c = compute_pass();
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
}

// -----------------------------------------------------------------
// 6b. Parallel determinism — across fresh Worlds.
//     Two fresh Worlds, identical scripted builds, identical
//     parallel updates: the output digest must match.
// -----------------------------------------------------------------
TEST(Determinism, EachParallelStableAcrossFreshWorlds)
{
  auto run_once = []()
  {
    World w(4);
    ScriptedBuild(w, 5000);
    w.EachParallel<const Pose, Velocity>(
        [](Entity, const Pose &_p, Velocity &_v)
        {
          _v.vx = _p.x * 3.0 + 1.0;
        });
    return IterationDigest(w);
  };
  uint64_t a = run_once();
  uint64_t b = run_once();
  uint64_t c = run_once();
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
}

// -----------------------------------------------------------------
// 6c. Parallel determinism with DEFERRED mutations.
//     Workers call Add() inside the EachParallel body while a phase
//     is open. Each worker's Add lands in its thread-local
//     CommandBuffer; Commit drains all buffers and applies them.
//     The doc claims Commit merges in `(thread_id, insertion_order)`
//     deterministically — exercise that path.
// -----------------------------------------------------------------
TEST(Determinism, EachParallelDeferredMutationsStable)
{
  auto run_once = []()
  {
    World w(4);
    auto es = ScriptedBuild(w, 5000);
    (void)es;
    // Open a phase. Subsequent mutations defer.
    w.BeginPhase();
    w.EachParallel<const Pose>(
        [&w](Entity _e, const Pose &_p)
        {
          // Add Tag to roughly half the entities, based on a
          // deterministic per-entity predicate so the resulting
          // set is the same across runs regardless of execution
          // order.
          if ((static_cast<int>(_p.x) % 2) == 0)
            w.Add(_e, Tag{});
        });
    w.Commit();

    // Digest covers (Entity, has-Tag) for every entity that still
    // carries a Pose. Iteration order is pinned by the Each walk.
    uint64_t h = 0xcbf29ce484222325ull;
    auto mix = [&h](uint64_t _v)
    {
      const auto *b = reinterpret_cast<const unsigned char *>(&_v);
      for (size_t i = 0; i < sizeof(_v); ++i)
      {
        h ^= b[i];
        h *= 0x100000001b3ull;
      }
    };
    w.Each<const Pose>(
        [&](Entity _e, const Pose &)
        {
          mix(_e.Raw());
          mix(w.Has(_e, ComponentTypeRegistry::Instance()
              .TypeIdOf<Tag>()) ? 1 : 0);
        });
    return h;
  };
  uint64_t a = run_once();
  uint64_t b = run_once();
  uint64_t c = run_once();
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
}

// -----------------------------------------------------------------
// 7. Comprehensive: scripted Add / Remove / Destroy sequence + final
//    iteration digest. Replays a non-trivial mutation sequence
//    twice and compares the entire resulting state.
// -----------------------------------------------------------------
TEST(Determinism, ScriptedSequenceProducesIdenticalState)
{
  auto run_once = []()
  {
    World w(1);
    auto es = ScriptedBuild(w, 1000);
    // Add Tag to half of them.
    for (size_t i = 0; i < es.size(); i += 2)
      w.Add(es[i], Tag{});
    // Remove Tag from a third of those.
    for (size_t i = 0; i < es.size(); i += 6)
      w.Remove<Tag>(es[i]);
    // Destroy a different sparse subset.
    for (size_t i = 5; i < es.size(); i += 11)
      w.Destroy(es[i]);
    // Recreate some of the destroyed slots.
    for (int k = 0; k < 50; ++k)
      w.Create(Pose{double(2000 + k), 0, 0}, Velocity{});
    return IterationDigest(w);
  };
  uint64_t a = run_once();
  uint64_t b = run_once();
  EXPECT_EQ(a, b);
}
