/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <string>

#include <gz/common/Filesystem.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "test_config.hh"  // NOLINT(build/include)
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

// Adapted from peachtree0222's reproduction in
// https://github.com/gazebosim/gz-sim/issues/3979.
class PhysicsLateRemoval
    : public InternalFixture<::testing::TestWithParam<bool>>
{
};

namespace
{
class RemoveGroundSystem :
    public System,
    public ISystemConfigurePriority,
    public ISystemPreUpdate,
    public ISystemUpdate,
    public ISystemReset
{
  public: RemoveGroundSystem(Entity _ground, bool _duringUpdate)
      : ground(_ground), duringUpdate(_duringUpdate)
  {
  }

  public: System::PriorityType ConfigurePriority() final
  {
    this->priorityConfigured = true;
    // Added in memory so this priority is visible independently of dynamic
    // plugin interface registration. Run after Physics even at its default
    // priority (see #3976).
    return 1000;
  }

  public: void PreUpdate(const UpdateInfo &,
      EntityComponentManager &_ecm) final
  {
    if (!this->duringUpdate)
      this->Remove(_ecm);
  }

  public: void Update(const UpdateInfo &,
      EntityComponentManager &_ecm) final
  {
    if (this->duringUpdate)
      this->Remove(_ecm);
  }

  public: void Reset(const UpdateInfo &, EntityComponentManager &) final
  {
  }

  public: void RequestRemoval()
  {
    this->requestRemoval = true;
  }

  private: void Remove(EntityComponentManager &_ecm)
  {
    if (this->requestRemoval)
    {
      _ecm.RequestRemoveEntity(this->ground);
      // Repeated requests must not cause duplicate backend removal.
      _ecm.RequestRemoveEntity(this->ground);
      this->requestRemoval = false;
    }
  }

  public: bool priorityConfigured{false};

  private: Entity ground{kNullEntity};
  private: bool duringUpdate{false};
  private: bool requestRemoval{true};
};

struct RemovalOutcome
{
  bool modelAbsent{false};
  bool descendantsAbsent{false};
  std::optional<math::Pose3d> spherePose;
};

RemovalOutcome RunGroundScenario(
    const std::optional<bool> &_requestDuringUpdate, bool _parallel,
    bool _paused = false, bool _reset = false)
{
  ServerConfig serverConfig;
  sdf::Root root;
  EXPECT_TRUE(root.Load(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "physics_late_removal.sdf")).empty());
  root.WorldByIndex(0)->Element()->GetElement("gz:policies")
      ->GetElement("parallel_postupdates")->Set(_parallel);
  serverConfig.SetSdfRoot(root);

  Server server(serverConfig);
  server.SetUpdatePeriod(std::chrono::nanoseconds(0));

  const auto ground = server.EntityByName("ground");
  EXPECT_TRUE(ground.has_value());

  std::optional<math::Pose3d> spherePose;
  Entity groundLink{kNullEntity};
  Entity groundCollision{kNullEntity};
  bool descendantsAbsent = false;
  bool removedHierarchyObserved = false;
  std::shared_ptr<RemoveGroundSystem> remover;
  if (_requestDuringUpdate.has_value())
  {
    remover = std::make_shared<RemoveGroundSystem>(
        ground.value_or(kNullEntity), *_requestDuringUpdate);
    EXPECT_TRUE(server.AddSystem(remover).value_or(false));
  }

  test::Relay observer;
  observer.OnPreUpdate([&](const UpdateInfo &,
      EntityComponentManager &_ecm)
  {
    if (groundLink == kNullEntity)
    {
      groundLink = _ecm.EntityByComponents(
          components::Link(), components::Name("ground_link"));
      groundCollision = _ecm.EntityByComponents(
          components::Collision(), components::Name("ground_collision"));
    }
    else
    {
      descendantsAbsent = !_ecm.HasEntity(groundLink) &&
          !_ecm.HasEntity(groundCollision);
    }
  });
  observer.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    _ecm.EachRemoved<components::Model>(
        [&](const Entity &_entity, const components::Model *)
        {
          if (_entity == ground.value_or(kNullEntity))
          {
            // Other PostUpdate readers must retain access to the hierarchy
            // while Physics removes its privately owned backend objects.
            removedHierarchyObserved =
                _ecm.Component<components::Link>(groundLink) != nullptr &&
                _ecm.Component<components::Collision>(groundCollision) !=
                    nullptr;
          }
          return true;
        });
    const auto sphere = _ecm.EntityByComponents(
        components::Model(), components::Name("sphere"));
    if (kNullEntity != sphere)
    {
      const auto pose = _ecm.Component<components::Pose>(sphere);
      if (nullptr != pose)
        spherePose = pose->Data();
    }
  });
  EXPECT_TRUE(server.AddSystem(observer.systemPtr).value_or(false));

  EXPECT_TRUE(server.RunOnce(_paused));
  EXPECT_NE(kNullEntity, groundLink);
  EXPECT_NE(kNullEntity, groundCollision);
  if (remover)
  {
    EXPECT_TRUE(remover->priorityConfigured);
    EXPECT_TRUE(removedHierarchyObserved);
  }
  const bool modelAbsent = !server.HasEntity("ground");
  EXPECT_TRUE(server.Run(true, 2000, false));
  if (_reset && remover)
  {
    EXPECT_TRUE(modelAbsent);
    server.ResetAll();
    EXPECT_TRUE(server.Run(true, 1000, false));
    EXPECT_TRUE(server.HasEntity("ground"));
    EXPECT_EQ(ground, server.EntityByName("ground"));
    EXPECT_TRUE(spherePose.has_value());
    if (spherePose)
    {
      EXPECT_NEAR(spherePose->Pos().Z(), 0.5, 1e-3);
    }
    remover->RequestRemoval();
    EXPECT_TRUE(server.RunOnce(false));
    EXPECT_FALSE(server.HasEntity("ground"));
    EXPECT_TRUE(server.Run(true, 2000, false));
  }

  if (spherePose)
  {
    ::testing::Test::RecordProperty("final_sphere_z",
        std::to_string(spherePose->Pos().Z()));
  }
  return {modelAbsent, descendantsAbsent, spherePose};
}
}

TEST_P(PhysicsLateRemoval, RemovalRequestedBeforePhysicsRemovesBackendObject)
{
  const auto outcome = RunGroundScenario(false, GetParam());
  EXPECT_TRUE(outcome.modelAbsent);
  EXPECT_TRUE(outcome.descendantsAbsent);
  ASSERT_TRUE(outcome.spherePose.has_value());
  EXPECT_LT(outcome.spherePose->Pos().Z(), 0.0);
}

TEST_P(PhysicsLateRemoval, RemovalRequestedAfterPhysicsMustRemoveBackendObject)
{
  const auto outcome = RunGroundScenario(true, GetParam());
  EXPECT_TRUE(outcome.modelAbsent);
  EXPECT_TRUE(outcome.descendantsAbsent);
  ASSERT_TRUE(outcome.spherePose.has_value());
  EXPECT_LT(outcome.spherePose->Pos().Z(), 0.0);
}

TEST_P(PhysicsLateRemoval, RetainedGroundStopsSphere)
{
  const auto outcome = RunGroundScenario(std::nullopt, GetParam());
  EXPECT_FALSE(outcome.modelAbsent);
  ASSERT_TRUE(outcome.spherePose.has_value());
  EXPECT_NEAR(outcome.spherePose->Pos().Z(), 0.5, 1e-3);
}

TEST_P(PhysicsLateRemoval, RemovalRequestedWhilePaused)
{
  const auto outcome = RunGroundScenario(true, GetParam(), true);
  EXPECT_TRUE(outcome.modelAbsent);
  EXPECT_TRUE(outcome.descendantsAbsent);
  ASSERT_TRUE(outcome.spherePose.has_value());
  EXPECT_LT(outcome.spherePose->Pos().Z(), 0.0);
}

TEST_P(PhysicsLateRemoval, RemoveRestoredModelAgain)
{
  const auto outcome = RunGroundScenario(true, GetParam(), false, true);
  EXPECT_TRUE(outcome.modelAbsent);
  EXPECT_TRUE(outcome.descendantsAbsent);
  ASSERT_TRUE(outcome.spherePose.has_value());
  EXPECT_LT(outcome.spherePose->Pos().Z(), 0.0);
}

INSTANTIATE_TEST_SUITE_P(PostUpdateModes, PhysicsLateRemoval,
    ::testing::Bool());
