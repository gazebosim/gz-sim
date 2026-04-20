/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>
#include <string>

#include "gz/sim/ecs/ComponentTypeRegistry.hh"

using gz::sim::ecs::ComponentTypeRegistry;

namespace
{
  struct Pose { double x, y, z; };
  struct Velocity { double vx, vy, vz; };
  struct Tag {};
}

TEST(ComponentTypeRegistry, RegisterReturnsInfo)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *p = reg.Register<Pose>("Pose");
  ASSERT_NE(p, nullptr);
  EXPECT_EQ(p->size, sizeof(Pose));
  EXPECT_EQ(p->alignment, alignof(Pose));
  EXPECT_TRUE(p->trivially_relocatable);
  EXPECT_EQ(p->name, "Pose");
}

TEST(ComponentTypeRegistry, RegisterIsIdempotent)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *a = reg.Register<Velocity>("Velocity");
  auto *b = reg.Register<Velocity>("Velocity");
  EXPECT_EQ(a, b);
}

TEST(ComponentTypeRegistry, DistinctTypesHaveDistinctIds)
{
  auto &reg = ComponentTypeRegistry::Instance();
  auto *p = reg.Register<Pose>();
  auto *v = reg.Register<Velocity>();
  EXPECT_NE(p->id, v->id);
}

TEST(ComponentTypeRegistry, ZeroSizedTypeRegisters)
{
  auto &reg = ComponentTypeRegistry::Instance();
  // Zero-size types should still register — they still need a distinct id,
  // even though they contribute no bytes to any column.
  auto *t = reg.Register<Tag>("Tag");
  ASSERT_NE(t, nullptr);
  EXPECT_GT(t->size, 0u);  // C++ mandates sizeof(empty) >= 1.
}

TEST(ComponentTypeRegistry, ConstructDestructRoundTrip)
{
  struct Thing { int x = 99; };
  auto &reg = ComponentTypeRegistry::Instance();
  auto *info = reg.Register<Thing>("Thing");
  alignas(Thing) std::byte buf[sizeof(Thing)];
  info->construct(&buf);
  auto *obj = reinterpret_cast<Thing *>(&buf);
  EXPECT_EQ(obj->x, 99);
  info->destruct(&buf);
}

// Verifies the gz::sim convention: a type that publishes a static
// ComponentTypeId typeId member (set by GZ_SIM_REGISTER_COMPONENT in the
// real codebase) gets that id in preference to a name-derived fallback.
namespace
{
  struct FakeGzSimComponent
  {
    double v{0};
    static inline gz::sim::ComponentTypeId typeId{0};
  };
}

TEST(ComponentTypeRegistry, GzSimTypeIdIsPreferred)
{
  using namespace gz::sim::ecs;
  // Simulate GZ_SIM_REGISTER_COMPONENT: Factory writes the id into the
  // static member at static-init time.
  constexpr gz::sim::ComponentTypeId kFakeId = 0xABCDEF0123456789ULL;
  FakeGzSimComponent::typeId = kFakeId;

  auto &reg = ComponentTypeRegistry::Instance();
  auto id = reg.TypeIdOf<FakeGzSimComponent>();
  EXPECT_EQ(id, kFakeId);

  auto *info = reg.Register<FakeGzSimComponent>("fake.gz_sim.component");
  EXPECT_EQ(info->id, kFakeId);

  // And: if typeId has not been set (still 0), fall back to the name hash.
  FakeGzSimComponent::typeId = 0;
  auto fallback = reg.TypeIdOf<FakeGzSimComponent>();
  EXPECT_NE(fallback, 0u);
  // Restore for later tests.
  FakeGzSimComponent::typeId = kFakeId;
}
