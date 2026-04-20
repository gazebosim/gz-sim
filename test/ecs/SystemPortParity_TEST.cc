/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Phase 0c: smoke-test one system's typical mutation pattern against the
 * archetype-backed ECM facade. This doesn't instantiate a real System
 * (that would pull in SimulationRunner and the plugin loader); instead
 * it replays the mutation *shape* a diff_drive-style system would
 * perform in a single Update tick:
 *
 *   - Read a command component (LinearVelocityCmd).
 *   - Integrate into Pose over dt.
 *   - Write a JointVelocityCmd per wheel.
 *
 * If this runs correctly under deferred semantics it demonstrates the
 * pattern the real systems use is archetype-safe — the paper audit in
 * docs/design/0c-audit-checklist.md is backed by a runnable test.
 */
#include <cmath>

#include <gtest/gtest.h>

#include "gz/sim/ecs/ArchetypeBackedECM.hh"

using gz::sim::ecs::ArchetypeBackedECM;

namespace
{
  struct LinearVelocityCmd  { double v{0}; };
  struct AngularVelocityCmd { double w{0}; };
  struct Pose { double x{0}, y{0}, yaw{0}; };
  struct JointVelocityCmd   { double vel{0}; };

  // Stand-in for a system's "this is my entity" lookup result.
  struct DiffDriveEntities
  {
    gz::sim::Entity body{gz::sim::kNullEntity};
    gz::sim::Entity left_wheel{gz::sim::kNullEntity};
    gz::sim::Entity right_wheel{gz::sim::kNullEntity};
  };
}

// Mimics the inner loop of a diff_drive-style system.
// Crucially: does NOT cache any component pointer across the call.
// Re-lookup is cheap under archetypes and the only archetype-safe
// pattern.
static void DiffDriveTick(ArchetypeBackedECM &_ecm,
                          DiffDriveEntities _e,
                          double _dt,
                          double _wheelRadius,
                          double _halfTrack)
{
  auto *vlin = _ecm.Component<LinearVelocityCmd>(_e.body);
  auto *vang = _ecm.Component<AngularVelocityCmd>(_e.body);
  auto *pose = _ecm.Component<Pose>(_e.body);
  if (!vlin || !vang || !pose) return;

  pose->x   += vlin->v * std::cos(pose->yaw) * _dt;
  pose->y   += vlin->v * std::sin(pose->yaw) * _dt;
  pose->yaw += vang->w * _dt;

  auto *left  = _ecm.Component<JointVelocityCmd>(_e.left_wheel);
  auto *right = _ecm.Component<JointVelocityCmd>(_e.right_wheel);
  if (!left || !right) return;
  left->vel  = (vlin->v - vang->w * _halfTrack) / _wheelRadius;
  right->vel = (vlin->v + vang->w * _halfTrack) / _wheelRadius;
}

TEST(SystemPortParity, DiffDriveTickRunsUnderArchetypeFacade)
{
  ArchetypeBackedECM ecm;
  DiffDriveEntities dde;
  dde.body        = ecm.CreateEntity();
  dde.left_wheel  = ecm.CreateEntity();
  dde.right_wheel = ecm.CreateEntity();

  ecm.CreateComponent<LinearVelocityCmd>(dde.body,  {1.0});
  ecm.CreateComponent<AngularVelocityCmd>(dde.body, {0.5});
  ecm.CreateComponent<Pose>(dde.body, {0, 0, 0});
  ecm.CreateComponent<JointVelocityCmd>(dde.left_wheel, {});
  ecm.CreateComponent<JointVelocityCmd>(dde.right_wheel, {});

  // Run 100 ticks at 1 kHz.
  const double dt = 0.001;
  const double R = 0.1;   // wheel radius
  const double H = 0.2;   // half-track
  for (int k = 0; k < 100; ++k)
    DiffDriveTick(ecm, dde, dt, R, H);

  auto *p = ecm.Component<Pose>(dde.body);
  ASSERT_NE(p, nullptr);
  EXPECT_GT(p->x, 0.0);       // moved forward
  EXPECT_GT(p->yaw, 0.0);     // yawed positive

  auto *l = ecm.Component<JointVelocityCmd>(dde.left_wheel);
  auto *r = ecm.Component<JointVelocityCmd>(dde.right_wheel);
  ASSERT_NE(l, nullptr);
  ASSERT_NE(r, nullptr);
  EXPECT_LT(l->vel, r->vel);  // turning left → right wheel spins faster
}

// Deferred-mutation contract: queued Add inside a phase is NOT visible
// until CommitPhase. The audit tool flags mutate-then-read patterns
// because of this.
TEST(SystemPortParity, DeferredMutationHidden_UntilCommit)
{
  ArchetypeBackedECM ecm;
  auto e = ecm.CreateEntity();
  ecm.CreateComponent<Pose>(e, Pose{1, 2, 0});

  ecm.BeginPhase();
  // Simulate a system that queues a component add.
  // Note: ArchetypeBackedECM::CreateComponent uses world.Add which is
  // deferred inside a phase. The mutation is not visible until commit.
  ecm.CreateComponent<LinearVelocityCmd>(e, {3.0});
  EXPECT_FALSE(ecm.EntityHasComponent<LinearVelocityCmd>(e));

  ecm.CommitPhase();
  EXPECT_TRUE(ecm.EntityHasComponent<LinearVelocityCmd>(e));
  EXPECT_DOUBLE_EQ(ecm.Component<LinearVelocityCmd>(e)->v, 3.0);
}
