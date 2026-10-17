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
 *
 */

#include "JointStateWriter.hh"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PhysicsOwner.hh"

using namespace gz;
using namespace sim;
using namespace systems::physics_system;

namespace
{
const char *kOther = "gz::sim::systems::Other";

/// \brief Two models with one joint each. The first model has no owner. The
/// second belongs to another provider.
struct Scene
{
  EntityComponentManager ecm;
  Entity ownJoint{kNullEntity};
  Entity otherJoint{kNullEntity};

  Scene()
  {
    this->ownJoint = this->AddJoint("");
    this->otherJoint = this->AddJoint(kOther);
    this->ecm.SetAllComponentsUnchanged();
  }

  Entity AddJoint(const std::string &_owner)
  {
    const auto model = this->ecm.CreateEntity();
    this->ecm.CreateComponent(model, components::Model());
    if (!_owner.empty())
      this->ecm.CreateComponent(model, components::PhysicsOwner(_owner));

    const auto joint = this->ecm.CreateEntity();
    this->ecm.CreateComponent(joint, components::Joint());
    this->ecm.CreateComponent(joint, components::ParentEntity(model));
    this->ecm.CreateComponent(joint, components::JointPosition());
    this->ecm.CreateComponent(joint, components::JointVelocity());
    return joint;
  }
};
}

/////////////////////////////////////////////////
TEST(JointStateWriter, WritesOnlyOwnedJoints)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  std::vector<Entity> visited;
  auto state = [&](const Entity &_joint, std::vector<double> &_state)
  {
    visited.push_back(_joint);
    _state = {1.5, 2.5};
    return true;
  };

  WriteJointPositions(scene.ecm, physics, state);
  ASSERT_EQ(1u, visited.size());
  EXPECT_EQ(scene.ownJoint, visited[0]);

  auto *own = scene.ecm.Component<components::JointPosition>(scene.ownJoint);
  ASSERT_NE(nullptr, own);
  EXPECT_EQ((std::vector<double>{1.5, 2.5}), own->Data());
  EXPECT_EQ(ComponentState::PeriodicChange,
      scene.ecm.ComponentState(scene.ownJoint,
          components::JointPosition::typeId));

  auto *other =
      scene.ecm.Component<components::JointPosition>(scene.otherJoint);
  ASSERT_NE(nullptr, other);
  EXPECT_TRUE(other->Data().empty());
  EXPECT_EQ(ComponentState::NoChange,
      scene.ecm.ComponentState(scene.otherJoint,
          components::JointPosition::typeId));
}

/////////////////////////////////////////////////
TEST(JointStateWriter, AcceptAllFilterWritesEveryJoint)
{
  Scene scene;

  std::vector<Entity> visited;
  WriteJointPositions(scene.ecm, OwnerFilter(),
      [&](const Entity &_joint, std::vector<double> &_state)
      {
        visited.push_back(_joint);
        _state = {1.0};
        return true;
      });
  EXPECT_EQ(2u, visited.size());
  EXPECT_EQ((std::vector<double>{1.0}),
      scene.ecm.Component<components::JointPosition>(
          scene.otherJoint)->Data());
}

/////////////////////////////////////////////////
TEST(JointStateWriter, WritesOtherProviderJoints)
{
  Scene scene;
  const auto other = OwnerFilter::ForProvider(
      scene.ecm, kOther, false);

  WriteJointPositions(scene.ecm, other,
      [](const Entity &, std::vector<double> &_state)
      {
        _state = {3.0};
        return true;
      });

  EXPECT_TRUE(scene.ecm.Component<components::JointPosition>(
      scene.ownJoint)->Data().empty());
  EXPECT_EQ((std::vector<double>{3.0}),
      scene.ecm.Component<components::JointPosition>(
          scene.otherJoint)->Data());
}

/////////////////////////////////////////////////
TEST(JointStateWriter, UnsimulatedJointIsLeftAlone)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  WriteJointPositions(scene.ecm, physics,
      [](const Entity &, std::vector<double> &) { return false; });

  EXPECT_EQ(ComponentState::NoChange,
      scene.ecm.ComponentState(scene.ownJoint,
          components::JointPosition::typeId));
}

/////////////////////////////////////////////////
TEST(JointStateWriter, Velocities)
{
  Scene scene;
  const auto physics = OwnerFilter::ForProvider(
      scene.ecm, "Default", true);

  WriteJointVelocities(scene.ecm, physics,
      [](const Entity &, std::vector<double> &_state)
      {
        _state.resize(1);
        _state[0] = -4.0;
        return true;
      });

  EXPECT_EQ((std::vector<double>{-4.0}),
      scene.ecm.Component<components::JointVelocity>(
          scene.ownJoint)->Data());
  EXPECT_TRUE(scene.ecm.Component<components::JointVelocity>(
      scene.otherJoint)->Data().empty());

  // Velocities are not marked as changed, as in the Physics system.
  EXPECT_EQ(ComponentState::NoChange,
      scene.ecm.ComponentState(scene.ownJoint,
          components::JointVelocity::typeId));
}
