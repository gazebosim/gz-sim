/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#include <gtest/gtest.h>
#include <cstring>

#include "gz/sim/ecs/CommandBuffer.hh"
#include "gz/sim/ecs/World.hh"

using namespace gz::sim::ecs;

namespace
{
  struct Pose { double x, y, z; };
  struct Vel  { double vx, vy; };
}

TEST(CommandBuffer, AppendPayloadStoresBytes)
{
  CommandBuffer buf;
  int payload = 42;
  auto off = buf.AppendPayload(&payload, sizeof(int));
  EXPECT_EQ(off, 0u);
  EXPECT_EQ(buf.Blob().size(), sizeof(int));
  int read = 0;
  std::memcpy(&read, buf.Blob().data() + off, sizeof(int));
  EXPECT_EQ(read, 42);
}

TEST(CommandBuffer, ResetClearsOps)
{
  CommandBuffer buf;
  Command c{};
  c.kind = CommandKind::DestroyEntity;
  buf.Ops().push_back(c);
  buf.AppendPayload("abcd", 4);
  EXPECT_FALSE(buf.Empty());
  buf.Reset();
  EXPECT_TRUE(buf.Empty());
  EXPECT_EQ(buf.Blob().size(), 0u);
}

TEST(World, MultiplePhaseAddRemoveSequence)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});

  w.BeginPhase();
  w.Add(e, Vel{10, 20});
  w.Remove<Pose>(e);
  w.Commit();

  EXPECT_FALSE(w.Has<Pose>(e));
  EXPECT_TRUE(w.Has<Vel>(e));
  EXPECT_DOUBLE_EQ(w.Component<Vel>(e)->vx, 10.0);
}

TEST(World, DeferredDestroyInPhase)
{
  World w(1);
  Entity e = w.Create(Pose{1, 2, 3});
  w.BeginPhase();
  w.Destroy(e);
  EXPECT_TRUE(w.IsAlive(e));  // Still alive in-phase.
  w.Commit();
  EXPECT_FALSE(w.IsAlive(e));
}
