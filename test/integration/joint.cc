/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointEffortLimitsCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityLimitsCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/ThreadPitch.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class JointIntegrationTest : public InternalFixture<::testing::Test>
{
};

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Joint joint;
    EXPECT_FALSE(joint.Valid(ecm));
  }

  // Missing joint component
  {
    auto id = ecm.CreateEntity();
    Joint joint(id);
    EXPECT_FALSE(joint.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Joint>(id, components::Joint());

    Joint joint(id);
    EXPECT_TRUE(joint.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No name
  EXPECT_EQ(std::nullopt, joint.Name(ecm));

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("joint_name"));
  EXPECT_EQ("joint_name", joint.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, JointNames)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No parent or child joint names
  EXPECT_EQ(std::nullopt, joint.ParentLinkName(ecm));
  EXPECT_EQ(std::nullopt, joint.ChildLinkName(ecm));

  // Add parent joint name
  ecm.CreateComponent<components::ParentLinkName>(id,
    components::ParentLinkName("parent_joint_name"));
  EXPECT_EQ("parent_joint_name", joint.ParentLinkName(ecm));

  // Add child joint name
  ecm.CreateComponent<components::ChildLinkName>(id,
    components::ChildLinkName("child_joint_name"));
  EXPECT_EQ("child_joint_name", joint.ChildLinkName(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Pose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No pose
  EXPECT_EQ(std::nullopt, joint.Pose(ecm));

  // Add pose
  math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  ecm.CreateComponent<components::Pose>(id,
      components::Pose(pose));
  EXPECT_EQ(pose, joint.Pose(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Type)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No joint type
  EXPECT_EQ(std::nullopt, joint.Type(ecm));

  // Add type
  sdf::JointType jointType = sdf::JointType::PRISMATIC;
  ecm.CreateComponent<components::JointType>(id,
      components::JointType(jointType));
  EXPECT_EQ(jointType, joint.Type(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Axis)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No joint axis
  EXPECT_EQ(std::nullopt, joint.Axis(ecm));

  // Add axis
  sdf::JointAxis jointAxis;
  auto errors = jointAxis.SetXyz(math::Vector3d(0, 1, 1));
  EXPECT_TRUE(errors.empty());
  ecm.CreateComponent<components::JointAxis>(id,
      components::JointAxis(jointAxis));
  EXPECT_EQ(jointAxis.Xyz(), (*joint.Axis(ecm))[0].Xyz());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, ThreadPitch)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Joint>(id, components::Joint());

  Joint joint(id);

  // No name
  EXPECT_EQ(std::nullopt, joint.ThreadPitch(ecm));

  // Add name
  ecm.CreateComponent<components::ThreadPitch>(id,
      components::ThreadPitch(1.23));
  EXPECT_DOUBLE_EQ(1.23, *joint.ThreadPitch(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SensorByName)
{
  EntityComponentManager ecm;

  // Joint
  auto eJoint = ecm.CreateEntity();
  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());
  EXPECT_EQ(0u, joint.SensorCount(ecm));

  // Sensor
  auto eSensor = ecm.CreateEntity();
  ecm.CreateComponent<components::Sensor>(eSensor, components::Sensor());
  ecm.CreateComponent<components::ParentEntity>(eSensor,
      components::ParentEntity(eJoint));
  ecm.CreateComponent<components::Name>(eSensor,
      components::Name("sensor_name"));

  // Check joint
  EXPECT_EQ(eSensor, joint.SensorByName(ecm, "sensor_name"));
  EXPECT_EQ(1u, joint.SensorCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SetVelocity)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointVelocityCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointVelocityCmd>(eJoint));

  std::vector<double> velCmd{1};
  joint.SetVelocity(ecm, velCmd);

  // velocity cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointVelocityCmd>(eJoint));
  EXPECT_EQ(velCmd,
    ecm.Component<components::JointVelocityCmd>(eJoint)->Data());

  // Make sure the velocity cmd is updated
  std::vector<double> velCmd2{0};
  joint.SetVelocity(ecm, velCmd2);
  EXPECT_EQ(velCmd2,
    ecm.Component<components::JointVelocityCmd>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SetForce)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointForceCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointForceCmd>(eJoint));

  std::vector<double> forceCmd{10};
  joint.SetForce(ecm, forceCmd);

  // force cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointForceCmd>(eJoint));
  EXPECT_EQ(forceCmd,
    ecm.Component<components::JointForceCmd>(eJoint)->Data());

  // Make sure the force cmd is updated
  std::vector<double> forceCmd2{1};
  joint.SetForce(ecm, forceCmd2);
  EXPECT_EQ(forceCmd2,
    ecm.Component<components::JointForceCmd>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SetVelocityLimits)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointVelocityLimitsCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointVelocityLimitsCmd>(eJoint));

  std::vector<math::Vector2d> velLimitsCmd{math::Vector2d(0.1, 1.1)};
  joint.SetVelocityLimits(ecm, velLimitsCmd);

  // velocity limits cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointVelocityLimitsCmd>(eJoint));
  EXPECT_EQ(velLimitsCmd,
    ecm.Component<components::JointVelocityLimitsCmd>(eJoint)->Data());

  // Make sure the velocity limits cmd is updated
  std::vector<math::Vector2d> velLimitsCmd2{math::Vector2d(-0.2, 2.4)};
  joint.SetVelocityLimits(ecm, velLimitsCmd2);
  EXPECT_EQ(velLimitsCmd2,
    ecm.Component<components::JointVelocityLimitsCmd>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SetEffortLimits)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointEffortLimitsCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointEffortLimitsCmd>(eJoint));

  std::vector<math::Vector2d> effortLimitsCmd{math::Vector2d(9, 9.9)};
  joint.SetEffortLimits(ecm, effortLimitsCmd);

  // effort limits cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointEffortLimitsCmd>(eJoint));
  EXPECT_EQ(effortLimitsCmd,
    ecm.Component<components::JointEffortLimitsCmd>(eJoint)->Data());

  // Make sure the effort limits cmd is updated
  std::vector<math::Vector2d> effortLimitsCmd2{math::Vector2d(5.2, 5.4)};
  joint.SetEffortLimits(ecm, effortLimitsCmd2);
  EXPECT_EQ(effortLimitsCmd2,
    ecm.Component<components::JointEffortLimitsCmd>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, SetPositionLimits)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointPositionLimitsCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointPositionLimitsCmd>(eJoint));

  std::vector<math::Vector2d> positionLimitsCmd{math::Vector2d(-0.1, 0.1)};
  joint.SetPositionLimits(ecm, positionLimitsCmd);

  // position limits cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointPositionLimitsCmd>(eJoint));
  EXPECT_EQ(positionLimitsCmd,
    ecm.Component<components::JointPositionLimitsCmd>(eJoint)->Data());

  // Make sure the position limits cmd is updated
  std::vector<math::Vector2d> positionLimitsCmd2{math::Vector2d(-0.2, 0.4)};
  joint.SetPositionLimits(ecm, positionLimitsCmd2);
  EXPECT_EQ(positionLimitsCmd2,
    ecm.Component<components::JointPositionLimitsCmd>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, ResetVelocity)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointVelocityReset should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointVelocityReset>(eJoint));

  std::vector<double> vel{1};
  joint.ResetVelocity(ecm, vel);

  // velocity reset should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointVelocityReset>(eJoint));
  EXPECT_EQ(vel,
    ecm.Component<components::JointVelocityReset>(eJoint)->Data());

  // Make sure the velocity reset is updated
  std::vector<double> vel2{0};
  joint.ResetVelocity(ecm, vel2);
  EXPECT_EQ(vel2,
    ecm.Component<components::JointVelocityReset>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, ResetPosition)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // No JointPositionReset should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::JointPositionReset>(eJoint));

  std::vector<double> pos{1};
  joint.ResetPosition(ecm, pos);

  // position reset should exist
  EXPECT_NE(nullptr, ecm.Component<components::JointPositionReset>(eJoint));
  EXPECT_EQ(pos,
    ecm.Component<components::JointPositionReset>(eJoint)->Data());

  // Make sure the position reset is updated
  std::vector<double> pos2{0};
  joint.ResetPosition(ecm, pos2);
  EXPECT_EQ(pos2,
    ecm.Component<components::JointPositionReset>(eJoint)->Data());
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Velocity)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // Before enabling, velocity should return nullopt
  EXPECT_EQ(std::nullopt, joint.Velocity(ecm));

  // After enabling, velocity should return default values
  joint.EnableVelocityCheck(ecm);

  EXPECT_NE(nullptr, ecm.Component<components::JointVelocity>(eJoint));

  std::vector<double> velOut = *joint.Velocity(ecm);
  EXPECT_TRUE(velOut.empty());

  // With custom velocities
  std::vector<double> vel{0.3, 0.4};
  ecm.SetComponentData<components::JointVelocity>(eJoint, vel);
  velOut = *joint.Velocity(ecm);
  EXPECT_EQ(2u, velOut.size());
  EXPECT_DOUBLE_EQ(vel[0], velOut[0]);
  EXPECT_DOUBLE_EQ(vel[1], velOut[1]);

  // Disabling velocities goes back to nullopt
  joint.EnableVelocityCheck(ecm, false);
  EXPECT_EQ(std::nullopt, joint.Velocity(ecm));
  EXPECT_EQ(nullptr, ecm.Component<components::JointVelocity>(eJoint));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, Position)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // Before enabling, position should return nullopt
  EXPECT_EQ(std::nullopt, joint.Position(ecm));

  // After enabling, position should return default values
  joint.EnablePositionCheck(ecm);

  EXPECT_NE(nullptr, ecm.Component<components::JointPosition>(eJoint));

  std::vector<double> posOut = *joint.Position(ecm);
  EXPECT_TRUE(posOut.empty());

  // With custom positions
  std::vector<double> pos{-0.3, -0.4};
  ecm.SetComponentData<components::JointPosition>(eJoint, pos);
  posOut = *joint.Position(ecm);
  EXPECT_EQ(2u, posOut.size());
  EXPECT_DOUBLE_EQ(pos[0], posOut[0]);
  EXPECT_DOUBLE_EQ(pos[1], posOut[1]);

  // Disabling positions goes back to nullopt
  joint.EnablePositionCheck(ecm, false);
  EXPECT_EQ(std::nullopt, joint.Position(ecm));
  EXPECT_EQ(nullptr, ecm.Component<components::JointPosition>(eJoint));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, TransmittedWrench)
{
  EntityComponentManager ecm;

  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());

  ASSERT_TRUE(joint.Valid(ecm));

  // Before enabling, wrench should return nullopt
  EXPECT_EQ(std::nullopt, joint.TransmittedWrench(ecm));

  // After enabling, wrench should return default values
  joint.EnableTransmittedWrenchCheck(ecm);

  EXPECT_NE(nullptr, ecm.Component<components::JointTransmittedWrench>(eJoint));

  std::vector<msgs::Wrench> wrenchOut = *joint.TransmittedWrench(ecm);
  // todo(anyone) Unlike Velocity and Position functions that return an
  // empty vector if it has not been populated yet, the wrench vector
  // will contain one empty wrench msg. This is because the TransmittedWrench
  // API workarounds the fact that the TransmittedWrench component contains
  // only one wrench reading instead of a wrench vector like positions and
  // velocities.
  // EXPECT_TRUE(wrenchOut.empty());

  // With custom wrench
  msgs::Wrench wrenchMsg;
  msgs::Set(wrenchMsg.mutable_force(),
      math::Vector3d(0.2, 3.2, 0.1));

  std::vector<msgs::Wrench> wrench{wrenchMsg};
  ecm.SetComponentData<components::JointTransmittedWrench>(eJoint, wrench[0]);
  wrenchOut = *joint.TransmittedWrench(ecm);
  EXPECT_EQ(1u, wrenchOut.size());
  EXPECT_DOUBLE_EQ(wrench[0].force().x(), wrenchOut[0].force().x());
  EXPECT_DOUBLE_EQ(wrench[0].force().y(), wrenchOut[0].force().y());
  EXPECT_DOUBLE_EQ(wrench[0].force().z(), wrenchOut[0].force().z());

  // Disabling wrench goes back to nullopt
  joint.EnableTransmittedWrenchCheck(ecm, false);
  EXPECT_EQ(std::nullopt, joint.TransmittedWrench(ecm));
  EXPECT_EQ(nullptr, ecm.Component<components::JointTransmittedWrench>(eJoint));
}

//////////////////////////////////////////////////
TEST_F(JointIntegrationTest, ParentModel)
{
  EntityComponentManager ecm;

  // Model
  auto eModel = ecm.CreateEntity();
  ecm.CreateComponent(eModel, components::Model());
  auto eJoint = ecm.CreateEntity();
  ecm.CreateComponent(eJoint, components::Joint());

  Joint joint(eJoint);
  EXPECT_EQ(eJoint, joint.Entity());
  EXPECT_FALSE(joint.ParentModel(ecm).has_value());

  ecm.CreateComponent<components::ParentEntity>(eJoint,
      components::ParentEntity(eModel));

  ASSERT_TRUE(joint.Valid(ecm));

  // Check parent model
  EXPECT_EQ(eModel, ecm.ParentEntity(eJoint));
  auto parentModel = joint.ParentModel(ecm);
  ASSERT_TRUE(parentModel.has_value());
  EXPECT_TRUE(parentModel->Valid(ecm));
  EXPECT_EQ(eModel, parentModel->Entity());
}
