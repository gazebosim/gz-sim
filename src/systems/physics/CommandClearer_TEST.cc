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

#include "CommandClearer.hh"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/components/AngularVelocityReset.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPositionLimitsCmd.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointVelocityLimitsCmd.hh"
#include "gz/sim/components/JointVelocityReset.hh"
#include "gz/sim/components/LinearVelocityCmd.hh"
#include "gz/sim/components/LinearVelocityReset.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"

using namespace gz;
using namespace sim;
using namespace systems::physics_system;

namespace
{
const char *kOther = "gz::sim::systems::Other";

/// \brief Give an entity every component that ClearCommands touches.
void AddCommands(EntityComponentManager &_ecm, const Entity &_entity)
{
  msgs::Wrench wrench;
  wrench.mutable_force()->set_x(5);

  _ecm.CreateComponent(_entity, components::JointPositionReset(
      std::vector<double>{1.0}));
  _ecm.CreateComponent(_entity, components::JointVelocityReset(
      std::vector<double>{1.0}));
  _ecm.CreateComponent(_entity, components::WorldLinearVelocityReset(
      math::Vector3d(1, 0, 0)));
  _ecm.CreateComponent(_entity, components::WorldAngularVelocityReset(
      math::Vector3d(1, 0, 0)));
  _ecm.CreateComponent(_entity,
      components::EnableContactSurfaceCustomization(true));
  _ecm.CreateComponent(_entity, components::JointVelocityCmd(
      std::vector<double>{1.0}));
  _ecm.CreateComponent(_entity, components::LinearVelocityCmd(
      math::Vector3d(1, 0, 0)));
  _ecm.CreateComponent(_entity, components::AngularVelocityCmd(
      math::Vector3d(1, 0, 0)));
  _ecm.CreateComponent(_entity, components::JointForceCmd(
      std::vector<double>{2.0, 3.0}));
  _ecm.CreateComponent(_entity, components::SlipComplianceCmd(
      std::vector<double>{0.5, 0.5}));
  _ecm.CreateComponent(_entity, components::ExternalWorldWrenchCmd(wrench));
  const std::vector<math::Vector2d> limits{{-1, 1}};
  _ecm.CreateComponent(_entity, components::JointPositionLimitsCmd(limits));
  _ecm.CreateComponent(_entity, components::JointVelocityLimitsCmd(limits));
  _ecm.CreateComponent(_entity, components::JointEffortLimitsCmd(limits));
}

/// \brief Check that ClearCommands removed an entity's commands.
void ExpectCleared(const EntityComponentManager &_ecm, const Entity &_entity)
{
  EXPECT_EQ(nullptr, _ecm.Component<components::JointPositionReset>(_entity));
  EXPECT_EQ(nullptr, _ecm.Component<components::JointVelocityReset>(_entity));
  EXPECT_EQ(nullptr,
      _ecm.Component<components::WorldLinearVelocityReset>(_entity));
  EXPECT_EQ(nullptr,
      _ecm.Component<components::WorldAngularVelocityReset>(_entity));
  EXPECT_EQ(nullptr,
      _ecm.Component<components::EnableContactSurfaceCustomization>(_entity));
  EXPECT_EQ(nullptr, _ecm.Component<components::JointVelocityCmd>(_entity));
  EXPECT_EQ(nullptr, _ecm.Component<components::LinearVelocityCmd>(_entity));
  EXPECT_EQ(nullptr, _ecm.Component<components::AngularVelocityCmd>(_entity));

  // These stay, but hold no command.
  EXPECT_EQ((std::vector<double>{0.0, 0.0}),
      _ecm.Component<components::JointForceCmd>(_entity)->Data());
  EXPECT_EQ((std::vector<double>{0.0, 0.0}),
      _ecm.Component<components::SlipComplianceCmd>(_entity)->Data());
  EXPECT_DOUBLE_EQ(0.0, _ecm.Component<components::ExternalWorldWrenchCmd>(
      _entity)->Data().force().x());
  EXPECT_TRUE(_ecm.Component<components::JointPositionLimitsCmd>(
      _entity)->Data().empty());
  EXPECT_TRUE(_ecm.Component<components::JointVelocityLimitsCmd>(
      _entity)->Data().empty());
  EXPECT_TRUE(_ecm.Component<components::JointEffortLimitsCmd>(
      _entity)->Data().empty());
}

/// \brief Check that ClearCommands did not touch an entity.
void ExpectUntouched(const EntityComponentManager &_ecm, const Entity &_entity)
{
  EXPECT_NE(nullptr, _ecm.Component<components::JointPositionReset>(_entity));
  EXPECT_NE(nullptr, _ecm.Component<components::JointVelocityReset>(_entity));
  EXPECT_NE(nullptr,
      _ecm.Component<components::WorldLinearVelocityReset>(_entity));
  EXPECT_NE(nullptr,
      _ecm.Component<components::WorldAngularVelocityReset>(_entity));
  EXPECT_NE(nullptr,
      _ecm.Component<components::EnableContactSurfaceCustomization>(_entity));
  EXPECT_NE(nullptr, _ecm.Component<components::JointVelocityCmd>(_entity));
  EXPECT_NE(nullptr, _ecm.Component<components::LinearVelocityCmd>(_entity));
  EXPECT_NE(nullptr, _ecm.Component<components::AngularVelocityCmd>(_entity));
  EXPECT_EQ((std::vector<double>{2.0, 3.0}),
      _ecm.Component<components::JointForceCmd>(_entity)->Data());
  EXPECT_EQ((std::vector<double>{0.5, 0.5}),
      _ecm.Component<components::SlipComplianceCmd>(_entity)->Data());
  EXPECT_DOUBLE_EQ(5.0, _ecm.Component<components::ExternalWorldWrenchCmd>(
      _entity)->Data().force().x());
  EXPECT_EQ(1u, _ecm.Component<components::JointPositionLimitsCmd>(
      _entity)->Data().size());
  EXPECT_EQ(1u, _ecm.Component<components::JointVelocityLimitsCmd>(
      _entity)->Data().size());
  EXPECT_EQ(1u, _ecm.Component<components::JointEffortLimitsCmd>(
      _entity)->Data().size());
}

/// \brief Add a model and a child entity, which carries the commands.
Entity AddEntityWithCommands(EntityComponentManager &_ecm,
    const std::string &_owner)
{
  const auto model = _ecm.CreateEntity();
  _ecm.CreateComponent(model, components::Model());
  if (!_owner.empty())
    _ecm.CreateComponent(model, components::PhysicsOwner(_owner));

  const auto entity = _ecm.CreateEntity();
  _ecm.CreateComponent(entity, components::ParentEntity(model));
  AddCommands(_ecm, entity);
  return entity;
}
}

/////////////////////////////////////////////////
TEST(CommandClearer, ClearsEverythingWithoutOwners)
{
  EntityComponentManager ecm;
  const auto entity = AddEntityWithCommands(ecm, "");
  ClearCommands(ecm, OwnerFilter::ForProvider(ecm, "Default", true));
  ExpectCleared(ecm, entity);
}

/////////////////////////////////////////////////
TEST(CommandClearer, ClearsOnlyOwnedEntities)
{
  EntityComponentManager ecm;
  const auto own = AddEntityWithCommands(ecm, "");
  const auto other = AddEntityWithCommands(ecm, kOther);

  ClearCommands(ecm, OwnerFilter::ForProvider(ecm, "Default", true));
  ExpectCleared(ecm, own);
  ExpectUntouched(ecm, other);

  // The other provider clears its own entities.
  ClearCommands(ecm, OwnerFilter::ForProvider(ecm, kOther, false));
  ExpectCleared(ecm, other);
}

/////////////////////////////////////////////////
TEST(CommandClearer, AcceptAllFilterClearsOwnedEntitiesToo)
{
  EntityComponentManager ecm;
  const auto own = AddEntityWithCommands(ecm, "");
  const auto other = AddEntityWithCommands(ecm, kOther);

  ClearCommands(ecm, OwnerFilter());
  ExpectCleared(ecm, own);
  ExpectCleared(ecm, other);
}

/////////////////////////////////////////////////
TEST(CommandClearer, EmptyEcm)
{
  EntityComponentManager ecm;
  ClearCommands(ecm, OwnerFilter::ForProvider(ecm, "Default", true));
}
