/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Phase 0b parity test. Exercises the ECM-shaped surface of
 * ArchetypeBackedECM and verifies that it preserves ECM semantics for
 * the scripted mutation sequences systems actually use.
 *
 * A full side-by-side parity test against the legacy ECM is out of
 * scope until the full facade migration lands in a later milestone of
 * 0b. This test locks in the contract of the parts that do ship now.
 */
#include <gtest/gtest.h>
#include <string>

#include "gz/sim/ecs/ArchetypeBackedECM.hh"

using gz::sim::ecs::ArchetypeBackedECM;

namespace
{
  struct Pose { double x, y, z; };
  struct Velocity { double vx, vy, vz; };
  struct Name { std::string value; };
}

TEST(ArchetypeBackedECM, CreateAndHasEntity)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  EXPECT_NE(e, gz::sim::kNullEntity);
  EXPECT_TRUE(ecm.HasEntity(e));
  EXPECT_FALSE(ecm.HasEntity(12345));
}

TEST(ArchetypeBackedECM, CreateComponentThenReadBack)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  auto *p = ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});
  ASSERT_NE(p, nullptr);
  EXPECT_DOUBLE_EQ(p->x, 1.0);
  EXPECT_TRUE(ecm.EntityHasComponent<Pose>(e));

  auto *q = ecm.Component<Pose>(e);
  ASSERT_NE(q, nullptr);
  EXPECT_DOUBLE_EQ(q->x, 1.0);
}

TEST(ArchetypeBackedECM, RemoveComponent)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});
  EXPECT_TRUE(ecm.EntityHasComponent<Pose>(e));

  EXPECT_TRUE(ecm.RemoveComponent<Pose>(e));
  EXPECT_FALSE(ecm.EntityHasComponent<Pose>(e));
  EXPECT_EQ(ecm.Component<Pose>(e), nullptr);
}

TEST(ArchetypeBackedECM, EachSeesAllEntities)
{
  ArchetypeBackedECM ecm;
  for (int i = 0; i < 100; ++i)
  {
    auto e = ecm.CreateEntity();
    ecm.CreateComponent<Pose>(e, Pose{double(i), 0, 0});
    ecm.CreateComponent<Velocity>(e, Velocity{});
  }
  int count = 0;
  double sum = 0;
  ecm.Each<Pose, Velocity>(
      [&](gz::sim::Entity, Pose *p, Velocity *v) -> bool
  {
    ++count;
    sum += p->x;
    v->vx = p->x * 2;
    return true;
  });
  EXPECT_EQ(count, 100);
  EXPECT_DOUBLE_EQ(sum, 99.0 * 100.0 / 2.0);

  // Writes through the callback are visible on a second pass.
  int verified = 0;
  ecm.Each<Velocity>([&](gz::sim::Entity, Velocity *v) -> bool
  {
    if (v->vx > 0) ++verified;
    return true;
  });
  EXPECT_GT(verified, 0);
}

TEST(ArchetypeBackedECM, EachReturnFalseEarlyExit)
{
  ArchetypeBackedECM ecm;
  for (int i = 0; i < 10; ++i)
  {
    auto e = ecm.CreateEntity();
    ecm.CreateComponent<Pose>(e, Pose{double(i), 0, 0});
  }
  int seen = 0;
  ecm.Each<Pose>([&](gz::sim::Entity, Pose *) -> bool
  {
    ++seen;
    return seen < 3;  // stop after 3 callbacks
  });
  EXPECT_EQ(seen, 3);
}

TEST(ArchetypeBackedECM, RequestRemoveEntityDeferred)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});
  ecm.RequestRemoveEntity(e);
  // Not yet actually removed — processed in ProcessRemoveEntityRequests.
  EXPECT_TRUE(ecm.HasEntity(e));

  ecm.ProcessRemoveEntityRequests();
  EXPECT_FALSE(ecm.HasEntity(e));
  EXPECT_EQ(ecm.EntityCount(), 0u);
}

TEST(ArchetypeBackedECM, CommitPhaseIsNoOpOutsidePhase)
{
  ArchetypeBackedECM ecm;
  // Creates outside a phase apply immediately; CommitPhase should be safe
  // to call anyway (matches ECM where it's just a boundary marker).
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});
  ecm.CommitPhase();
  EXPECT_TRUE(ecm.HasEntity(e));
  EXPECT_NE(ecm.Component<Pose>(e), nullptr);
}

TEST(ArchetypeBackedECM, EachNewReportsNewEntities)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});

  int seen = 0;
  ecm.EachNew<Pose>([&](gz::sim::Entity, Pose *) -> bool
  { ++seen; return true; });
  EXPECT_EQ(seen, 1);

  ecm.ClearNewlyCreatedEntities();
  seen = 0;
  ecm.EachNew<Pose>([&](gz::sim::Entity, Pose *) -> bool
  { ++seen; return true; });
  EXPECT_EQ(seen, 0);
}

TEST(ArchetypeBackedECM, EachRemovedReportsRemoved)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 3});
  ecm.RequestRemoveEntity(e);
  ecm.ProcessRemoveEntityRequests();

  int seen = 0;
  ecm.EachRemoved<Pose>([&](gz::sim::Entity) -> bool
  { ++seen; return true; });
  EXPECT_EQ(seen, 1);
}

TEST(ArchetypeBackedECM, NonTriviallyRelocatableName)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Name>(e, Name{"robot_42"});
  ASSERT_NE(ecm.Component<Name>(e), nullptr);
  EXPECT_EQ(ecm.Component<Name>(e)->value, "robot_42");

  // Archetype transition via adding a Pose must preserve the string.
  ecm.CreateComponent<Pose>(e, Pose{});
  ASSERT_NE(ecm.Component<Name>(e), nullptr);
  EXPECT_EQ(ecm.Component<Name>(e)->value, "robot_42");
}
