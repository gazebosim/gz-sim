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

#include "FrameDataWriter.hh"

#include <gtest/gtest.h>

#include <optional>
#include <utility>
#include <vector>

#include <gz/math/eigen3/Conversions.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"

using namespace gz;
using namespace sim;
using namespace systems::physics_system;

namespace
{
/// \brief Frame data of a link that is yawed by 90 degrees, so that body and
/// world vectors differ.
physics::FrameData3d YawedFrameData()
{
  physics::FrameData3d data;
  data.pose = math::eigen3::convert(
      math::Pose3d(1, 2, 3, 0, 0, GZ_PI_2));
  data.linearVelocity = Eigen::Vector3d(1, 0, 0);
  data.angularVelocity = Eigen::Vector3d(0, 0, 2);
  data.linearAcceleration = Eigen::Vector3d(0, 3, 0);
  data.angularAcceleration = Eigen::Vector3d(0, 4, 0);
  return data;
}

/// \brief Add all frame data components to an entity.
void AddFrameDataComponents(EntityComponentManager &_ecm, const Entity &_entity)
{
  _ecm.CreateComponent(_entity, components::WorldPose());
  _ecm.CreateComponent(_entity, components::WorldLinearVelocity());
  _ecm.CreateComponent(_entity, components::WorldAngularVelocity());
  _ecm.CreateComponent(_entity, components::WorldLinearAcceleration());
  _ecm.CreateComponent(_entity, components::WorldAngularAcceleration());
  _ecm.CreateComponent(_entity, components::LinearVelocity());
  _ecm.CreateComponent(_entity, components::AngularVelocity());
  _ecm.CreateComponent(_entity, components::LinearAcceleration());
  _ecm.CreateComponent(_entity, components::AngularAcceleration());
}

/// \brief Check the state of all frame data components of an entity.
void ExpectState(const EntityComponentManager &_ecm, const Entity &_entity,
    ComponentState _state)
{
  for (const auto &typeId : {
      components::WorldPose::typeId,
      components::WorldLinearVelocity::typeId,
      components::WorldAngularVelocity::typeId,
      components::WorldLinearAcceleration::typeId,
      components::WorldAngularAcceleration::typeId,
      components::LinearVelocity::typeId,
      components::AngularVelocity::typeId,
      components::LinearAcceleration::typeId,
      components::AngularAcceleration::typeId})
  {
    EXPECT_EQ(_state, _ecm.ComponentState(_entity, typeId)) << typeId;
  }
}
}

/////////////////////////////////////////////////
TEST(FrameDataWriter, Equality)
{
  EXPECT_TRUE(FrameDataVectorEqual({1, 2, 3}, {1, 2, 3}));
  EXPECT_TRUE(FrameDataVectorEqual({1, 2, 3}, {1 + 1e-8, 2, 3}));
  EXPECT_FALSE(FrameDataVectorEqual({1, 2, 3}, {1 + 1e-3, 2, 3}));

  const math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  EXPECT_TRUE(FrameDataPoseEqual(pose, pose));
  EXPECT_TRUE(FrameDataPoseEqual(pose,
      math::Pose3d(1 + 1e-8, 2, 3, 0.1, 0.2, 0.3)));
  EXPECT_FALSE(FrameDataPoseEqual(pose,
      math::Pose3d(1 + 1e-3, 2, 3, 0.1, 0.2, 0.3)));
  EXPECT_FALSE(FrameDataPoseEqual(pose,
      math::Pose3d(1, 2, 3, 0.1, 0.2, 0.4)));
}

/////////////////////////////////////////////////
TEST(FrameDataWriter, LinkFrameData)
{
  EntityComponentManager ecm;
  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, components::Link());
  AddFrameDataComponents(ecm, link);
  ecm.SetAllComponentsUnchanged();

  WriteLinkFrameData(ecm, link, YawedFrameData());

  const math::Pose3d expectedPose(1, 2, 3, 0, 0, GZ_PI_2);
  EXPECT_TRUE(FrameDataPoseEqual(expectedPose,
      ecm.Component<components::WorldPose>(link)->Data()));
  EXPECT_EQ(math::Vector3d(1, 0, 0),
      ecm.Component<components::WorldLinearVelocity>(link)->Data());
  EXPECT_EQ(math::Vector3d(0, 0, 2),
      ecm.Component<components::WorldAngularVelocity>(link)->Data());
  EXPECT_EQ(math::Vector3d(0, 3, 0),
      ecm.Component<components::WorldLinearAcceleration>(link)->Data());
  EXPECT_EQ(math::Vector3d(0, 4, 0),
      ecm.Component<components::WorldAngularAcceleration>(link)->Data());

  // A body that is yawed by 90 degrees sees the world x axis along its -y
  // axis, and the world y axis along its x axis.
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(0, -1, 0),
      ecm.Component<components::LinearVelocity>(link)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(0, 0, 2),
      ecm.Component<components::AngularVelocity>(link)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(3, 0, 0),
      ecm.Component<components::LinearAcceleration>(link)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(4, 0, 0),
      ecm.Component<components::AngularAcceleration>(link)->Data()));

  ExpectState(ecm, link, ComponentState::PeriodicChange);

  // The same data again does not mark anything as changed.
  ecm.SetAllComponentsUnchanged();
  WriteLinkFrameData(ecm, link, YawedFrameData());
  ExpectState(ecm, link, ComponentState::NoChange);

  // A change below the tolerance is not a change.
  ecm.SetAllComponentsUnchanged();
  auto data = YawedFrameData();
  data.linearVelocity.x() += 1e-9;
  WriteLinkFrameData(ecm, link, data);
  ExpectState(ecm, link, ComponentState::NoChange);

  // A change in one value marks the world and body components that depend on
  // it, and nothing else.
  ecm.SetAllComponentsUnchanged();
  data = YawedFrameData();
  data.linearVelocity.x() = 5;
  WriteLinkFrameData(ecm, link, data);
  EXPECT_EQ(ComponentState::PeriodicChange, ecm.ComponentState(link,
      components::WorldLinearVelocity::typeId));
  EXPECT_EQ(ComponentState::PeriodicChange, ecm.ComponentState(link,
      components::LinearVelocity::typeId));
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(link,
      components::WorldPose::typeId));
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(link,
      components::WorldAngularVelocity::typeId));
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(link,
      components::AngularVelocity::typeId));
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(link,
      components::LinearAcceleration::typeId));
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(link,
      components::WorldLinearAcceleration::typeId));
}

/////////////////////////////////////////////////
TEST(FrameDataWriter, LinkFrameDataOnlyIfPresent)
{
  EntityComponentManager ecm;

  // A link where only one reader exists.
  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, components::Link());
  ecm.CreateComponent(link, components::WorldLinearVelocity());

  // A link without readers.
  const auto bareLink = ecm.CreateEntity();
  ecm.CreateComponent(bareLink, components::Link());

  WriteLinkFrameData(ecm, link, YawedFrameData());
  WriteLinkFrameData(ecm, bareLink, YawedFrameData());

  EXPECT_EQ(math::Vector3d(1, 0, 0),
      ecm.Component<components::WorldLinearVelocity>(link)->Data());

  // No component is created on behalf of other systems.
  EXPECT_EQ(nullptr, ecm.Component<components::WorldPose>(link));
  EXPECT_EQ(nullptr, ecm.Component<components::LinearVelocity>(link));
  EXPECT_EQ(nullptr, ecm.Component<components::WorldPose>(bareLink));
  EXPECT_EQ(nullptr,
      ecm.Component<components::WorldLinearVelocity>(bareLink));
  EXPECT_EQ(nullptr, ecm.Component<components::AngularVelocity>(bareLink));

  // Writing to an entity that does not exist is harmless.
  WriteLinkFrameData(ecm, kNullEntity, YawedFrameData());
}

/////////////////////////////////////////////////
TEST(FrameDataWriter, ChildFrameData)
{
  EntityComponentManager ecm;

  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, components::Link());
  const auto otherLink = ecm.CreateEntity();
  ecm.CreateComponent(otherLink, components::Link());

  // A sensor on the link, offset by 1 m along the link x axis.
  const auto sensor = ecm.CreateEntity();
  ecm.CreateComponent(sensor, components::ParentEntity(link));
  ecm.CreateComponent(sensor, components::Pose(math::Pose3d(1, 0, 0, 0, 0, 0)));
  AddFrameDataComponents(ecm, sensor);

  // A collision with only some of the readers.
  const auto collision = ecm.CreateEntity();
  ecm.CreateComponent(collision, components::ParentEntity(link));
  ecm.CreateComponent(collision,
      components::Pose(math::Pose3d(0, 1, 0, 0, 0, 0)));
  ecm.CreateComponent(collision, components::WorldPose());
  ecm.CreateComponent(collision, components::LinearVelocity());

  // A sensor on a link that the callback does not know about.
  const auto foreign = ecm.CreateEntity();
  ecm.CreateComponent(foreign, components::ParentEntity(otherLink));
  ecm.CreateComponent(foreign,
      components::Pose(math::Pose3d(1, 0, 0, 0, 0, 0)));
  AddFrameDataComponents(ecm, foreign);

  // An entity with frame data components, but no parent.
  const auto orphan = ecm.CreateEntity();
  ecm.CreateComponent(orphan, components::Pose());
  ecm.CreateComponent(orphan, components::WorldPose());

  // The link is yawed by 90 degrees and moves along its y axis in the world.
  // The offset is therefore along the world y axis.
  std::vector<std::pair<Entity, math::Pose3d>> calls;
  auto atOffset = [&](const Entity &_link, const math::Pose3d &_offset)
      -> std::optional<physics::FrameData3d>
  {
    calls.emplace_back(_link, _offset);
    if (_link != link)
      return std::nullopt;

    physics::FrameData3d data;
    data.pose = math::eigen3::convert(
        math::Pose3d(10, 0, 0, 0, 0, GZ_PI_2) * _offset);
    data.linearVelocity = Eigen::Vector3d(0, 2, 0);
    data.angularVelocity = Eigen::Vector3d(0, 0, 1);
    data.linearAcceleration = Eigen::Vector3d(0, 3, 0);
    data.angularAcceleration = Eigen::Vector3d(0, 0, 4);
    return data;
  };

  ecm.SetAllComponentsUnchanged();
  WriteChildFrameData(ecm, atOffset);

  // The sensor pose is the link pose composed with the offset.
  EXPECT_TRUE(FrameDataPoseEqual(math::Pose3d(10, 1, 0, 0, 0, GZ_PI_2),
      ecm.Component<components::WorldPose>(sensor)->Data()));
  EXPECT_EQ(math::Vector3d(0, 2, 0),
      ecm.Component<components::WorldLinearVelocity>(sensor)->Data());
  EXPECT_EQ(math::Vector3d(0, 0, 1),
      ecm.Component<components::WorldAngularVelocity>(sensor)->Data());
  EXPECT_EQ(math::Vector3d(0, 3, 0),
      ecm.Component<components::WorldLinearAcceleration>(sensor)->Data());
  EXPECT_EQ(math::Vector3d(0, 0, 4),
      ecm.Component<components::WorldAngularAcceleration>(sensor)->Data());

  // Body-fixed vectors are the world vectors rotated by the inverse of the
  // sensor's world orientation.
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(2, 0, 0),
      ecm.Component<components::LinearVelocity>(sensor)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(0, 0, 1),
      ecm.Component<components::AngularVelocity>(sensor)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(3, 0, 0),
      ecm.Component<components::LinearAcceleration>(sensor)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(0, 0, 4),
      ecm.Component<components::AngularAcceleration>(sensor)->Data()));

  // The collision has only the components that exist.
  EXPECT_TRUE(FrameDataPoseEqual(math::Pose3d(9, 0, 0, 0, 0, GZ_PI_2),
      ecm.Component<components::WorldPose>(collision)->Data()));
  EXPECT_TRUE(FrameDataVectorEqual(math::Vector3d(2, 0, 0),
      ecm.Component<components::LinearVelocity>(collision)->Data()));
  EXPECT_EQ(nullptr,
      ecm.Component<components::WorldLinearVelocity>(collision));
  EXPECT_EQ(nullptr, ecm.Component<components::AngularVelocity>(collision));

  // Entities on links that the callback rejects keep their values.
  EXPECT_EQ(math::Pose3d::Zero,
      ecm.Component<components::WorldPose>(foreign)->Data());
  EXPECT_EQ(math::Vector3d::Zero,
      ecm.Component<components::WorldLinearVelocity>(foreign)->Data());
  EXPECT_EQ(math::Vector3d::Zero,
      ecm.Component<components::AngularAcceleration>(foreign)->Data());

  // Entities without a parent are skipped.
  EXPECT_EQ(math::Pose3d::Zero,
      ecm.Component<components::WorldPose>(orphan)->Data());

  // The callback is asked about the parent link and the offset in the link
  // frame.
  EXPECT_FALSE(calls.empty());
  bool sawSensor{false};
  bool sawCollision{false};
  for (const auto &[callLink, offset] : calls)
  {
    EXPECT_TRUE(callLink == link || callLink == otherLink);
    if (callLink == link && offset == math::Pose3d(1, 0, 0, 0, 0, 0))
      sawSensor = true;
    if (callLink == link && offset == math::Pose3d(0, 1, 0, 0, 0, 0))
      sawCollision = true;
  }
  EXPECT_TRUE(sawSensor);
  EXPECT_TRUE(sawCollision);

  // Child components are set directly. They are not compared with the
  // previous value or marked as changed.
  EXPECT_EQ(ComponentState::NoChange, ecm.ComponentState(sensor,
      components::WorldPose::typeId));
}
