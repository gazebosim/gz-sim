/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>

#include "gz/sim/ecs/detail/EntityIndex.hh"

using gz::sim::ecs::Entity;
using gz::sim::ecs::EntityIndex;
using gz::sim::ecs::EntityRecord;
using gz::sim::ecs::kNullEntity;

TEST(EntityIndex, AllocateAndAlive)
{
  EntityIndex idx;
  auto e = idx.Allocate();
  EXPECT_TRUE(idx.IsAlive(e));
  EXPECT_EQ(idx.AliveCount(), 1u);
}

TEST(EntityIndex, FreeInvalidatesHandle)
{
  EntityIndex idx;
  auto e = idx.Allocate();
  idx.Free(e);
  EXPECT_FALSE(idx.IsAlive(e));
  EXPECT_EQ(idx.AliveCount(), 0u);
}

TEST(EntityIndex, FreeListReusesSlot)
{
  EntityIndex idx;
  auto e1 = idx.Allocate();
  idx.Free(e1);
  auto e2 = idx.Allocate();
  EXPECT_EQ(e2.Index(), e1.Index());
  // Generation must differ so stale handles are detectable.
  EXPECT_NE(e2.Generation(), e1.Generation());
  EXPECT_FALSE(idx.IsAlive(e1));
  EXPECT_TRUE(idx.IsAlive(e2));
}

TEST(EntityIndex, SetAndGetRecord)
{
  EntityIndex idx;
  auto e = idx.Allocate();
  idx.Set(e, EntityRecord{7, 3, 42});
  auto rec = idx.Get(e);
  EXPECT_EQ(rec.archetype, 7u);
  EXPECT_EQ(rec.chunk_idx, 3u);
  EXPECT_EQ(rec.row, 42u);
}

TEST(EntityIndex, PatchByIndex)
{
  EntityIndex idx;
  auto e = idx.Allocate();
  idx.Set(e, EntityRecord{1, 2, 3});
  idx.PatchByIndex(e.Index(), EntityRecord{1, 2, 0});
  EXPECT_EQ(idx.Get(e).row, 0u);
}

TEST(EntityIndex, StaleHandleDetected)
{
  EntityIndex idx;
  auto e1 = idx.Allocate();
  idx.Free(e1);
  auto e2 = idx.Allocate();
  // e1's generation is stale.
  EXPECT_FALSE(idx.IsAlive(e1));
  EXPECT_TRUE(idx.IsAlive(e2));
}
