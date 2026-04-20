/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>

#include "gz/sim/ecs/detail/Chunk.hh"
#include "gz/sim/ecs/detail/ColumnLayout.hh"
#include "gz/sim/ecs/ComponentTypeRegistry.hh"

using gz::sim::ecs::Chunk;
using gz::sim::ecs::ComponentTypeRegistry;
using gz::sim::ecs::detail::ComputeColumnLayout;

namespace
{
  struct Pose  { double x, y, z; };
  struct Vel   { float vx, vy; };
}

TEST(Chunk, AcquireAndReleaseRows)
{
  Chunk c(4096, 32);
  EXPECT_EQ(c.Count(), 0u);
  EXPECT_EQ(c.Capacity(), 32u);
  EXPECT_TRUE(c.Empty());
  EXPECT_FALSE(c.Full());

  for (int i = 0; i < 32; ++i) EXPECT_EQ(c.AcquireRow(), (uint32_t)i);
  EXPECT_TRUE(c.Full());
  EXPECT_EQ(c.AcquireRow(), Chunk::kInvalidRow);

  c.ReleaseLastRow();
  EXPECT_FALSE(c.Full());
}

TEST(Chunk, IsAligned)
{
  Chunk c(16 * 1024, 64);
  uintptr_t addr = reinterpret_cast<uintptr_t>(c.Entities());
  EXPECT_EQ(addr % Chunk::kAlignment, 0u);
}

TEST(ColumnLayout, PackedOffsetsAreAligned)
{
  auto &reg = ComponentTypeRegistry::Instance();
  std::vector<const gz::sim::ecs::ComponentTypeInfo *> infos = {
      reg.Register<Pose>(),
      reg.Register<Vel>(),
  };
  auto layout = ComputeColumnLayout(infos, 16 * 1024);
  EXPECT_GT(layout.capacity, 100u);
  ASSERT_EQ(layout.offsets.size(), 2u);
  EXPECT_EQ(layout.offsets[0] % 16, 0u);
  EXPECT_EQ(layout.offsets[1] % 16, 0u);
  EXPECT_LE(layout.bytes_used, 16 * 1024u);
}

TEST(ColumnLayout, ZeroSizeChunkYieldsZeroCapacity)
{
  auto &reg = ComponentTypeRegistry::Instance();
  std::vector<const gz::sim::ecs::ComponentTypeInfo *> infos = {
      reg.Register<Pose>(),
  };
  auto layout = ComputeColumnLayout(infos, 8);  // can't fit a row at all.
  EXPECT_EQ(layout.capacity, 0u);
}

TEST(Chunk, DirtyBitsAllocation)
{
  Chunk c(4096, 32);
  c.AllocDirtyBits(4);
  ASSERT_NE(c.DirtyBits(), nullptr);
  EXPECT_EQ(c.DirtyWordCount(), 4u);
  for (size_t i = 0; i < 4; ++i) EXPECT_EQ(c.DirtyBits()[i], 0u);
}
