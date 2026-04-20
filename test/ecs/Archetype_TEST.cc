/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>

#include "gz/sim/ecs/detail/Archetype.hh"
#include "gz/sim/ecs/ComponentTypeRegistry.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x, y, z; };
  struct Vel  { float  vx, vy; };
}

TEST(Archetype, ConstructAndReflect)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *p = reg.Register<Pose>();
  auto *v = reg.Register<Vel>();

  std::vector<ComponentTypeId> types{p->id, v->id};
  std::sort(types.begin(), types.end());
  std::vector<const ComponentTypeInfo *> infos;
  auto &reg_ref = reg;
  for (auto t : types) infos.push_back(reg_ref.Get(t));

  Archetype a(0, types, infos);
  EXPECT_EQ(a.Types().size(), 2u);
  EXPECT_TRUE(a.Contains(p->id));
  EXPECT_TRUE(a.Contains(v->id));
  EXPECT_NE(a.ColumnIndexOf(p->id), static_cast<size_t>(-1));
  EXPECT_GT(a.ChunkCapacity(), 0u);
}

TEST(Archetype, AcquireRowSpillsToNewChunk)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *p = reg.Register<Pose>();
  std::vector<ComponentTypeId> types{p->id};
  std::vector<const ComponentTypeInfo *> infos{p};

  // Force tiny capacity: a chunk small enough to hold just a few rows.
  Archetype a(0, types, infos, /*chunk_size=*/256);
  size_t cap = a.ChunkCapacity();
  ASSERT_GT(cap, 0u);

  for (size_t i = 0; i < cap + 1; ++i)
    a.AcquireRow();
  EXPECT_GE(a.NumChunks(), 2u);
  EXPECT_EQ(a.TotalRows(), cap + 1);
}

TEST(Archetype, DirtyBitsMarkAndClear)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *p = reg.Register<Pose>();
  std::vector<ComponentTypeId> types{p->id};
  std::vector<const ComponentTypeInfo *> infos{p};

  Archetype a(0, types, infos);
  auto [ci, row] = a.AcquireRow();
  size_t col = a.ColumnIndexOf(p->id);

  EXPECT_FALSE(a.IsDirty(col, ci, row));
  a.MarkDirty(col, ci, row);
  EXPECT_TRUE(a.IsDirty(col, ci, row));
  a.ClearDirtyBits();
  EXPECT_FALSE(a.IsDirty(col, ci, row));
}
