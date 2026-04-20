/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>
#include <set>
#include <string>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x{0}, y{0}, z{0}; };
  struct Vel  { double vx{0}, vy{0}; };
  struct Name { std::string value; };
  struct Tag  {};
}

TEST(World, CreateImmediateAndIterate)
{
  World w(1);
  w.Create(Pose{1, 2, 3}, Vel{4, 5});
  w.Create(Pose{10, 20, 30}, Vel{40, 50});
  w.Create(Pose{100, 200, 300}, Vel{400, 500});

  int count = 0;
  double sum = 0;
  w.Each<const Pose, Vel>([&](Entity, const Pose &p, Vel &v)
  {
    ++count;
    sum += p.x;
    v.vx *= 2.0;
  });
  EXPECT_EQ(count, 3);
  EXPECT_DOUBLE_EQ(sum, 111.0);
  // Verify write persisted.
  int verify = 0;
  w.Each<Vel>([&](Entity, Vel &v) { verify += (v.vx > 0) ? 1 : 0; });
  EXPECT_EQ(verify, 3);
}

TEST(World, ComponentAccessAndModify)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});
  auto *p = w.Component<Pose>(e);
  ASSERT_NE(p, nullptr);
  EXPECT_DOUBLE_EQ(p->x, 1.0);

  auto *pm = w.Modify<Pose>(e);
  ASSERT_NE(pm, nullptr);
  pm->x = 99.0;
  EXPECT_DOUBLE_EQ(w.Component<Pose>(e)->x, 99.0);
}

TEST(World, AddRemoveTransitionsArchetype)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});
  size_t arche_before = w.NumArchetypes();
  EXPECT_TRUE(w.Has<Pose>(e));
  EXPECT_FALSE(w.Has<Vel>(e));

  w.Add(e, Vel{10, 20});
  EXPECT_TRUE(w.Has<Vel>(e));
  EXPECT_DOUBLE_EQ(w.Component<Pose>(e)->x, 1.0);
  EXPECT_DOUBLE_EQ(w.Component<Vel>(e)->vx, 10.0);
  EXPECT_GE(w.NumArchetypes(), arche_before);

  w.Remove<Pose>(e);
  EXPECT_FALSE(w.Has<Pose>(e));
  EXPECT_TRUE(w.Has<Vel>(e));
  EXPECT_DOUBLE_EQ(w.Component<Vel>(e)->vx, 10.0);
}

TEST(World, DestroyReleasesSlot)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});
  EXPECT_TRUE(w.IsAlive(e));
  w.Destroy(e);
  EXPECT_FALSE(w.IsAlive(e));
  EXPECT_EQ(w.NumEntities(), 0u);
  EXPECT_EQ(w.Component<Pose>(e), nullptr);
}

TEST(World, NonTriviallyRelocatable_StringName)
{
  World w(1);
  Entity e = w.Create(Name{"hello"});
  ASSERT_NE(w.Component<Name>(e), nullptr);
  EXPECT_EQ(w.Component<Name>(e)->value, "hello");
  // Transition: add a Pose, Name should be preserved.
  w.Add(e, Pose{});
  ASSERT_NE(w.Component<Name>(e), nullptr);
  EXPECT_EQ(w.Component<Name>(e)->value, "hello");
  w.Destroy(e);
}

TEST(World, ManyEntitiesIntoSameArchetype)
{
  World w(1);
  const int N = 1000;
  std::set<Entity> seen;
  for (int i = 0; i < N; ++i)
    seen.insert(w.Create(Pose{double(i), 0, 0}, Vel{}));
  EXPECT_EQ(seen.size(), (size_t)N);

  int c = 0;
  w.Each<const Pose>([&](Entity, const Pose &) { ++c; });
  EXPECT_EQ(c, N);
}

TEST(World, DeferredCreateInPhase)
{
  World w(1);
  w.BeginPhase();
  Entity e_out;
  (void)w.Create(Pose{1, 2, 3});  // placeholder return
  w.Commit();
  EXPECT_EQ(w.NumEntities(), 1u);
  int c = 0;
  w.Each<const Pose>([&](Entity, const Pose &p)
  {
    ++c;
    EXPECT_DOUBLE_EQ(p.x, 1.0);
  });
  EXPECT_EQ(c, 1);
}

TEST(World, DeferredAddInPhase)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});
  w.BeginPhase();
  w.Add(e, Vel{5, 6});
  // Inside the phase, the add is NOT visible yet.
  EXPECT_FALSE(w.Has<Vel>(e));
  w.Commit();
  EXPECT_TRUE(w.Has<Vel>(e));
}
