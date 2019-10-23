/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/Link.hh>

using namespace ignition;
using namespace gazebo;

class LinkIntegrationTest : public ::testing::Test
{
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Link link;
    EXPECT_FALSE(link.Valid(ecm));
  }

  // Missing link component
  {
    auto id = ecm.CreateEntity();
    Link link(id);
    EXPECT_FALSE(link.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Link>(id, components::Link());

    Link link(id);
    EXPECT_TRUE(link.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, ResetEntity)
{
  EntityComponentManager ecm;
  auto eLink1 = ecm.CreateEntity();
  ecm.CreateComponent(eLink1, components::Link());
  auto eLink2 = ecm.CreateEntity();
  ecm.CreateComponent(eLink2, components::Link());

  Link link(eLink1);
  EXPECT_EQ(eLink1, link.Entity());

  link.ResetEntity(eLink2);
  EXPECT_EQ(eLink2, link.Entity());
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Link>(id, components::Link());

  Link link(id);

  // No name
  EXPECT_EQ(std::nullopt, link.Name(ecm));

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("link_name"));
  EXPECT_EQ("link_name", link.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, ParentModel)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  // Model
  auto eModel = ecm.CreateEntity();
  ecm.CreateComponent(eModel, components::Model());
  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());
  EXPECT_FALSE(link.ParentModel(ecm).has_value());

  creator.SetParent(eLink, eModel);
  ASSERT_TRUE(link.Valid(ecm));

  // Check parent model
  EXPECT_EQ(eModel, ecm.ParentEntity(eLink));
  auto parentModel = link.ParentModel(ecm);
  ASSERT_TRUE(parentModel.has_value());
  EXPECT_TRUE(parentModel->Valid(ecm));
  EXPECT_EQ(eModel, parentModel->Entity());
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkPoses)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // Before we add components, pose functions should return nullopt

  EXPECT_EQ(std::nullopt, link.WorldPose(ecm));
  EXPECT_EQ(std::nullopt, link.WorldInertialPose(ecm));

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(1.0, 0.0, 0.0, 0, 0, IGN_PI_4);
  math::Pose3d inertiaPose;
  // This is the pose of the inertia frame relative to its parent link frame
  inertiaPose.Set(1.0, 2.0, 3.0, 0, IGN_PI_2, 0);

  math::Inertiald linkInertial;
  linkInertial.SetPose(inertiaPose);

  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));
  ecm.CreateComponent(eLink, components::Inertial(linkInertial));

  EXPECT_EQ(linkWorldPose, link.WorldPose(ecm));
  EXPECT_EQ(linkWorldPose * inertiaPose, link.WorldInertialPose(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkVelocities)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // Before we add components, velocity functions should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(std::nullopt, link.WorldAngularVelocity(ecm));

  math::Pose3d pose;
  pose.Set(0, 0, 0, IGN_PI_2, 0, 0);
  math::Vector3d linVel{1.0, 0.0, 0.0};
  math::Vector3d angVel{0.0, 0.0, 2.0};
  ecm.CreateComponent(eLink, components::WorldPose(pose));
  ecm.CreateComponent(eLink, components::WorldLinearVelocity(linVel));
  ecm.CreateComponent(eLink, components::WorldAngularVelocity(angVel));

  EXPECT_EQ(linVel, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(angVel, link.WorldAngularVelocity(ecm));

  // Linear velocity at offset
  math::Vector3d offset{0.0, 1.0, 0.0};
  math::Vector3d angVelBody = pose.Rot().RotateVectorReverse(angVel);
  auto expLinVel = linVel + pose.Rot().RotateVector(angVelBody.Cross(offset));
  EXPECT_EQ(expLinVel, link.WorldLinearVelocity(ecm, offset));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkAccelerations)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // Before we add components, velocity functions should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldLinearAcceleration(ecm));

  math::Vector3d linAccel{1.0, 0.0, 0.0};
  math::Vector3d angAccel{0.0, 0.0, 2.0};
  ecm.CreateComponent(eLink, components::WorldLinearAcceleration(linAccel));

  EXPECT_EQ(linAccel, link.WorldLinearAcceleration(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkInertiaMatrix)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // Before we add components, pose functions should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldInertiaMatrix(ecm));

  math::MassMatrix3d linkMassMatrix(1.0, {0.4, 0.4, 0.4}, {0.02, 0.02, 0.02});
  math::Pose3d linkComPose;
  linkComPose.Set(0.2, 0.1, 0.0, IGN_PI_4, 0, 0);
  math::Inertiald linkInertial{linkMassMatrix, linkComPose};

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(0.0, 0.1, 0.2, 0.0, IGN_PI_4, IGN_PI_2);

  ecm.CreateComponent(eLink, components::Inertial(linkInertial));
  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));

  math::Matrix3d comWorldRot(linkWorldPose.Rot() * linkComPose.Rot());
  math::Matrix3d expMoi =
      comWorldRot * linkMassMatrix.Moi() * comWorldRot.Transposed();
  EXPECT_EQ(expMoi, *link.WorldInertiaMatrix(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkKineticEnergy)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // Before we add components, pose functions should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldKineticEnergy(ecm));

  math::MassMatrix3d linkMassMatrix(2.0, {2.0, 1.5, 1.0}, {0.0, 0.0, 0.0});
  math::Pose3d linkComPose;
  linkComPose.Set(0.2, 0.0, 0.0, IGN_PI_2, 0, 0);
  math::Inertiald linkInertial{linkMassMatrix, linkComPose};

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(0.0, 0.1, 0.2, 0.0, 0.0, IGN_PI_2);

  // initially zero velocity
  math::Vector3d linkWorldAngularVelocity;
  math::Vector3d linkWorldLinearVelocity;

  ecm.CreateComponent(eLink, components::Inertial(linkInertial));
  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));
  ecm.CreateComponent(eLink,
      components::WorldAngularVelocity(linkWorldAngularVelocity));
  ecm.CreateComponent(eLink,
      components::WorldLinearVelocity(linkWorldLinearVelocity));

  // initially zero velocity
  EXPECT_DOUBLE_EQ(0.0, *link.WorldKineticEnergy(ecm));

  // update velocity and expect different kinetic energy
  const auto angularVel =
      ecm.Component<components::WorldAngularVelocity>(eLink);

  // the link and inertial rotations do the following:
  // xyz in world frame <-> zxy in inertial frame
  // the inertial pose is along y axis in world frame

  // T: kinetic energy
  // m: mass
  // w: world angular velocity

  // w = [1, 0, 0]
  // T = 0.5 * (Izz + m * cx^2)
  // T = 0.5 * (1 + 2 * 0.2^2)
  // T = 0.54
  *angularVel = components::WorldAngularVelocity({math::Vector3d(1, 0, 0)});
  EXPECT_DOUBLE_EQ(0.54, *link.WorldKineticEnergy(ecm));

  // w = [0, 1, 0]
  // T = 0.5 * Ixx
  // T = 0.5 * 2.0
  // T = 1.0
  *angularVel = components::WorldAngularVelocity({math::Vector3d(0, 1, 0)});
  EXPECT_DOUBLE_EQ(1.0, *link.WorldKineticEnergy(ecm));

  // w = [0, 0, 1]
  // T = 0.5 * (Iyy + m * cx^2)
  // T = 0.5 * (1.5 + 2 * 0.2^2)
  // T = 0.79
  *angularVel = components::WorldAngularVelocity({math::Vector3d(0, 0, 1)});
  EXPECT_DOUBLE_EQ(0.79, *link.WorldKineticEnergy(ecm));

  // reset angular velocity to zeros
  *angularVel = components::WorldAngularVelocity({math::Vector3d(0, 0, 0)});
  EXPECT_DOUBLE_EQ(0.0, *link.WorldKineticEnergy(ecm));

  // set linear velocity
  const auto linearVel =
      ecm.Component<components::WorldLinearVelocity>(eLink);

  *linearVel = components::WorldLinearVelocity({math::Vector3d(1, 0, 0)});
  EXPECT_DOUBLE_EQ(1.0, *link.WorldKineticEnergy(ecm));

  *linearVel = components::WorldLinearVelocity({math::Vector3d(10, 0, 0)});
  EXPECT_DOUBLE_EQ(100.0, *link.WorldKineticEnergy(ecm));
}
