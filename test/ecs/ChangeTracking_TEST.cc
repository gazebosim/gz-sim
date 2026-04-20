/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>

#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x{0}, y{0}, z{0}; };
  struct Vel  { double vx{0}, vy{0}; };
}

TEST(ChangeTracking, NewRowsTrackedAcrossCreate)
{
  World w(1);
  w.Create(Pose{1, 0, 0});
  w.Create(Pose{2, 0, 0});

  int seen = 0;
  w.EachNew<const Pose>([&](Entity, const Pose &) { ++seen; });
  EXPECT_EQ(seen, 2);

  w.ClearChangeBits();
  int seen2 = 0;
  w.EachNew<const Pose>([&](Entity, const Pose &) { ++seen2; });
  EXPECT_EQ(seen2, 0);
}

TEST(ChangeTracking, ModifyMarksDirty)
{
  World w(1);
  Entity e = w.Create(Pose{1, 0, 0});
  // Read-only access doesn't mark dirty.
  (void)w.Component<Pose>(e);
  int changed = 0;
  w.EachChanged<const Pose>([&](Entity, const Pose &) { ++changed; });
  EXPECT_EQ(changed, 0);

  auto *pm = w.Modify<Pose>(e);
  pm->x = 99.0;
  changed = 0;
  w.EachChanged<const Pose>([&](Entity, const Pose &) { ++changed; });
  EXPECT_EQ(changed, 1);

  w.ClearChangeBits();
  changed = 0;
  w.EachChanged<const Pose>([&](Entity, const Pose &) { ++changed; });
  EXPECT_EQ(changed, 0);
}

TEST(ChangeTracking, EachRemovedDrainsRecords)
{
  World w(1);
  Entity e1 = w.Create(Pose{1, 0, 0}, Vel{});
  Entity e2 = w.Create(Pose{2, 0, 0}, Vel{});
  w.Destroy(e1);
  int removed = 0;
  w.EachRemoved<Pose>([&](Entity) { ++removed; });
  EXPECT_EQ(removed, 1);
  w.ClearChangeBits();
  removed = 0;
  w.EachRemoved<Pose>([&](Entity) { ++removed; });
  EXPECT_EQ(removed, 0);
  (void)e2;
}
