/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>
#include <unordered_map>

#include "gz/sim/ecs/Entity.hh"

using gz::sim::ecs::Entity;
using gz::sim::ecs::kNullEntity;

TEST(Entity, DefaultIsNull)
{
  Entity e;
  EXPECT_EQ(e, kNullEntity);
  EXPECT_EQ(e.Index(), 0u);
  EXPECT_EQ(e.Generation(), 0u);
}

TEST(Entity, IndexGenerationSplit)
{
  Entity e(42u, 7u);
  EXPECT_EQ(e.Index(), 42u);
  EXPECT_EQ(e.Generation(), 7u);
}

TEST(Entity, Equality)
{
  EXPECT_EQ(Entity(1, 0), Entity(1, 0));
  EXPECT_NE(Entity(1, 0), Entity(1, 1));
  EXPECT_NE(Entity(1, 0), Entity(2, 0));
}

TEST(Entity, HashableAndOrderable)
{
  std::unordered_map<Entity, int> m;
  m[Entity(1, 0)] = 42;
  m[Entity(2, 0)] = 43;
  EXPECT_EQ(m[Entity(1, 0)], 42);
  EXPECT_EQ(m[Entity(2, 0)], 43);
  EXPECT_LT(Entity(1, 0), Entity(2, 0));
}

TEST(Entity, RawRoundTrip)
{
  Entity e(123, 456);
  Entity f = Entity::FromRaw(e.Raw());
  EXPECT_EQ(e, f);
}
