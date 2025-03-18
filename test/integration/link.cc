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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Link.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class LinkIntegrationTest : public InternalFixture<::testing::Test>
{
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
TEST_F(LinkIntegrationTest, VisualByName)
{
  EntityComponentManager ecm;

  // Link
  auto eLink = ecm.CreateEntity();
  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());
  EXPECT_EQ(0u, link.VisualCount(ecm));

  // Visual
  auto eVisual = ecm.CreateEntity();
  ecm.CreateComponent<components::Visual>(eVisual, components::Visual());
  ecm.CreateComponent<components::ParentEntity>(eVisual,
      components::ParentEntity(eLink));
  ecm.CreateComponent<components::Name>(eVisual,
      components::Name("visual_name"));

  // Check link
  EXPECT_EQ(eVisual, link.VisualByName(ecm, "visual_name"));
  EXPECT_EQ(1u, link.VisualCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, CollisionByName)
{
  EntityComponentManager ecm;

  // Link
  auto eLink = ecm.CreateEntity();
  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());
  EXPECT_EQ(0u, link.CollisionCount(ecm));

  // Collision
  auto eCollision = ecm.CreateEntity();
  ecm.CreateComponent<components::Collision>(eCollision,
      components::Collision());
  ecm.CreateComponent<components::ParentEntity>(eCollision,
      components::ParentEntity(eLink));
  ecm.CreateComponent<components::Name>(eCollision,
      components::Name("collision_name"));

  // Check link
  EXPECT_EQ(eCollision, link.CollisionByName(ecm, "collision_name"));
  EXPECT_EQ(1u, link.CollisionCount(ecm));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, IsCanonical)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Link>(id, components::Link());

  Link link(id);

  EXPECT_FALSE(link.IsCanonical(ecm));

  ecm.CreateComponent<components::CanonicalLink>(id,
      components::CanonicalLink());
  EXPECT_TRUE(link.IsCanonical(ecm));
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

  EXPECT_EQ(std::nullopt, link.WorldInertialPose(ecm));

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(1.0, 0.0, 0.0, 0, 0, GZ_PI_4);
  math::Pose3d inertiaPose;
  // This is the pose of the inertia frame relative to its parent link frame
  inertiaPose.Set(1.0, 2.0, 3.0, 0, GZ_PI_2, 0);

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

  // Before enabling, velocity functions should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(std::nullopt, link.WorldAngularVelocity(ecm));

  // After enabling, velocity functions should return default values
  link.EnableVelocityChecks(ecm);

  EXPECT_NE(nullptr, ecm.Component<components::WorldPose>(eLink));
  EXPECT_NE(nullptr, ecm.Component<components::WorldLinearVelocity>(eLink));
  EXPECT_NE(nullptr, ecm.Component<components::WorldAngularVelocity>(eLink));

  EXPECT_EQ(math::Vector3d::Zero, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(math::Vector3d::Zero, link.WorldAngularVelocity(ecm));

  // With custom velocities
  math::Pose3d pose;
  pose.Set(0, 0, 0, GZ_PI_2, 0, 0);
  math::Vector3d linVel{1.0, 0.0, 0.0};
  math::Vector3d angVel{0.0, 0.0, 2.0};
  ecm.SetComponentData<components::WorldPose>(eLink, pose);
  ecm.SetComponentData<components::WorldLinearVelocity>(eLink, linVel);
  ecm.SetComponentData<components::WorldAngularVelocity>(eLink, angVel);

  EXPECT_EQ(linVel, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(angVel, link.WorldAngularVelocity(ecm));

  // Linear velocity at offset
  math::Vector3d offset{0.0, 1.0, 0.0};
  math::Vector3d angVelBody = pose.Rot().RotateVectorReverse(angVel);
  auto expLinVel = linVel + pose.Rot().RotateVector(angVelBody.Cross(offset));
  EXPECT_EQ(expLinVel, link.WorldLinearVelocity(ecm, offset));

  // Disabling velocities goes back to nullopt
  link.EnableVelocityChecks(ecm, false);

  EXPECT_EQ(std::nullopt, link.WorldLinearVelocity(ecm));
  EXPECT_EQ(std::nullopt, link.WorldAngularVelocity(ecm));
  EXPECT_EQ(std::nullopt, link.WorldLinearVelocity(ecm, offset));
  EXPECT_EQ(nullptr, ecm.Component<components::WorldPose>(eLink));
  EXPECT_EQ(nullptr, ecm.Component<components::WorldLinearVelocity>(eLink));
  EXPECT_EQ(nullptr, ecm.Component<components::WorldAngularVelocity>(eLink));
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

  // Before we enable acceleration, acceleration should return nullopt
  EXPECT_EQ(std::nullopt, link.WorldLinearAcceleration(ecm));
  EXPECT_EQ(std::nullopt, link.WorldAngularAcceleration(ecm));

  // After enabling, they should return default values
  link.EnableAccelerationChecks(ecm);
  EXPECT_EQ(math::Vector3d::Zero, link.WorldLinearAcceleration(ecm));
  EXPECT_EQ(math::Vector3d::Zero, link.WorldAngularAcceleration(ecm));
  EXPECT_NE(nullptr, ecm.Component<components::WorldLinearAcceleration>(eLink));
  EXPECT_NE(nullptr,
      ecm.Component<components::WorldAngularAcceleration>(eLink));

  // After setting acceleration, we get the value
  math::Vector3d linAccel{1.0, 0.0, 0.0};
  ecm.SetComponentData<components::WorldLinearAcceleration>(eLink, linAccel);
  EXPECT_EQ(linAccel, link.WorldLinearAcceleration(ecm));

  math::Vector3d angAccel{0.0, 1.0, 0.0};
  ecm.SetComponentData<components::WorldAngularAcceleration>(eLink, angAccel);
  EXPECT_EQ(angAccel, link.WorldAngularAcceleration(ecm));

  // Disabling accelerations goes back to nullopt
  link.EnableAccelerationChecks(ecm, false);

  EXPECT_EQ(std::nullopt, link.WorldLinearAcceleration(ecm));
  EXPECT_EQ(std::nullopt, link.WorldAngularAcceleration(ecm));
  EXPECT_EQ(nullptr, ecm.Component<components::WorldLinearAcceleration>(eLink));
  EXPECT_EQ(nullptr,
      ecm.Component<components::WorldAngularAcceleration>(eLink));
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkInertia)
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
  EXPECT_EQ(std::nullopt, link.WorldInertial(ecm));

  math::MassMatrix3d linkMassMatrix(10.0, {0.4, 0.4, 0.4}, {0.02, 0.02, 0.02});
  math::Pose3d linkWorldPose;
  linkWorldPose.Set(1.0, 0.0, 0.0, 0, 0, GZ_PI_4);
  // This is the pose of the inertia frame relative to its parent link frame
  math::Pose3d inertiaPose;
  inertiaPose.Set(1.0, 2.0, 3.0, 0, GZ_PI_2, 0);
  math::Inertiald linkInertial{linkMassMatrix, inertiaPose};

  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));
  ecm.CreateComponent(eLink, components::Inertial(linkInertial));

  ASSERT_TRUE(link.WorldInertial(ecm));
  EXPECT_EQ(10.0, link.WorldInertial(ecm).value().MassMatrix().Mass());
  EXPECT_EQ(linkWorldPose * inertiaPose,
    link.WorldInertial(ecm).value().Pose());
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
  linkComPose.Set(0.2, 0.1, 0.0, GZ_PI_4, 0, 0);
  math::Inertiald linkInertial{linkMassMatrix, linkComPose};

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(0.0, 0.1, 0.2, 0.0, GZ_PI_4, GZ_PI_2);

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
  linkComPose.Set(0.2, 0.0, 0.0, GZ_PI_2, 0, 0);
  math::Inertiald linkInertial{linkMassMatrix, linkComPose};

  math::Pose3d linkWorldPose;
  linkWorldPose.Set(0.0, 0.1, 0.2, 0.0, 0.0, GZ_PI_2);

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

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkSetVelocity)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // No LinearVelocityCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::LinearVelocityCmd>(eLink));

  math::Vector3d linVel(1, 0, 0);
  link.SetLinearVelocity(ecm, linVel);

  // LinearVelocity cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::LinearVelocityCmd>(eLink));
  EXPECT_EQ(linVel,
    ecm.Component<components::LinearVelocityCmd>(eLink)->Data());

  // Make sure the linear velocity is updated
  math::Vector3d linVel2(0, 0, 0);
  link.SetLinearVelocity(ecm, linVel2);
  EXPECT_EQ(linVel2,
    ecm.Component<components::LinearVelocityCmd>(eLink)->Data());

  // No AngularVelocityCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::AngularVelocityCmd>(eLink));

  math::Vector3d angVel(0, 0, 1);
  link.SetAngularVelocity(ecm, angVel);

  // AngularVelocity cmd should exist
  EXPECT_NE(nullptr, ecm.Component<components::AngularVelocityCmd>(eLink));
  EXPECT_EQ(angVel,
    ecm.Component<components::AngularVelocityCmd>(eLink)->Data());

  // Make sure the angular velocity is updated
  math::Vector3d angVel2(0, 0, 0);
  link.SetAngularVelocity(ecm, angVel2);
  EXPECT_EQ(angVel2,
    ecm.Component<components::AngularVelocityCmd>(eLink)->Data());
}

//////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, LinkAddWorldForce)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // No ExternalWorldWrenchCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::ExternalWorldWrenchCmd>(eLink));

  // Add force
  math::Vector3d force(0, 0, 1.0);
  link.AddWorldForce(ecm, force);

  // No WorldPose or Inertial component exists so command should not work
  EXPECT_EQ(nullptr, ecm.Component<components::ExternalWorldWrenchCmd>(eLink));

  // create WorldPose and Inertial component and try adding force again
  math::Pose3d linkWorldPose;
  linkWorldPose.Set(1.0, 0.0, 0.0, 0, 0, GZ_PI_4);
  math::Pose3d inertiaPose;
  inertiaPose.Set(1.0, 2.0, 3.0, 0, GZ_PI_2, 0);
  math::Inertiald linkInertial;
  linkInertial.SetPose(inertiaPose);
  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));
  ecm.CreateComponent(eLink, components::Inertial(linkInertial));
  link.AddWorldForce(ecm, force);

  // ExternalWorldWrenchCmd component should now be created
  auto wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);

  // verify wrench values
  auto wrenchMsg = wrenchComp->Data();

  math::Vector3 expectedTorque =
      linkWorldPose.Rot().RotateVector(inertiaPose.Pos()).Cross(force);
  EXPECT_EQ(force, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(expectedTorque, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // apply opposite force. Since the cmd is not processed yet, this should
  // cancel out the existing wrench cmd
  link.AddWorldForce(ecm, -force);
  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // Add world force at an offset
  math::Vector3d offset{0.0, 1.0, 0.0};
  link.AddWorldForce(ecm, force, offset);

  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  expectedTorque =
      linkWorldPose.Rot().RotateVector(offset + inertiaPose.Pos()).Cross(force);
  EXPECT_EQ(force, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(expectedTorque, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // apply opposite force again and verify the resulting wrench values are zero
  link.AddWorldForce(ecm, -force, offset);
  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // apply world force at an offset and world torque
  math::Vector3d torque{1.0, 0.0, 0.0};
  link.AddWorldWrench(ecm, force, torque, offset);

  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  math::Vector3d posComWorldCoord = linkWorldPose.Rot().RotateVector(offset);
  expectedTorque = torque + posComWorldCoord.Cross(force);
  EXPECT_EQ(force, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(expectedTorque, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // apply opposite wrench again and verify the resulting wrench values are zero
  link.AddWorldWrench(ecm, -force, -torque, offset);
  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));
}

TEST_F(LinkIntegrationTest, LinkAddForceInInertialFrame)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  SdfEntityCreator creator(ecm, eventMgr);

  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, components::Link());

  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());

  ASSERT_TRUE(link.Valid(ecm));

  // No ExternalWorldWrenchCmd should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::ExternalWorldWrenchCmd>(eLink));

  // Add force in Inertial Frame
  math::Vector3d force(0, 0, 1.0);
  link.AddForceInInertialFrame(ecm, force);

  // No WorldPose or Inertial component exists so command should not work
  EXPECT_EQ(nullptr, ecm.Component<components::ExternalWorldWrenchCmd>(eLink));

  // create WorldPose and Inertial component and try adding force again
  math::Pose3d linkWorldPose;
  linkWorldPose.Set(1.0, 0.0, 0.0, 0, 0, GZ_PI_4);
  math::Pose3d inertiaPose;
  inertiaPose.Set(1.0, 2.0, 3.0, 0, GZ_PI_2, 0);
  math::Inertiald linkInertial;
  linkInertial.SetPose(inertiaPose);
  ecm.CreateComponent(eLink, components::WorldPose(linkWorldPose));
  ecm.CreateComponent(eLink, components::Inertial(linkInertial));
  link.AddForceInInertialFrame(ecm, force);

  // ExternalWorldWrenchCmd component should now be created
  auto wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);

  // Summary of dynamic state at this time in the test:
  // - The link has a world pose with position (1, 0, 0)
  // and orientation (0, 0, π/4).
  // - The link has an inertial pose with position (1, 2, 3)
  // and orientation (0, π/2, 0).
  // - A force of (0, 0, 1) N is applied in the inertial frame.
  // Original force vector in inertial frame
  // force = [0, 0, 1]
  // Rotation matrix for inertiaPose (rotation about Y-axis by π/2 radians)
  // Ry = [ cos(π/2)   0    sin(π/2)
  //            0      1       0
  //        -sin(π/2)  0    cos(π/2) ]
  // Ry = [  0   0   1
  //         0   1   0
  //        -1   0   0 ]
  //
  // Calculate linkForce = Ry * force:
  // linkForce_x = (0 * 0) + (0 * 0) + (1 * 1) = 1
  // linkForce_y = (0 * 0) + (1 * 0) + (0 * 1) = 0
  // linkForce_z = (-1 * 0) + (0 * 0) + (0 * 1) = 0
  //
  // Thus, linkForce = [1, 0, 0]
  //
  // Rotation matrix for linkWorldPose (rotation about Z-axis by π/4 radians)
  // Rz = [ cos(π/4)   -sin(π/4)   0
  //        sin(π/4)    cos(π/4)   0
  //           0           0       1 ]
  //
  // Rz ≈ [ √2/2   -√2/2    0
  //         √2/2    √2/2    0
  //          0       0      1 ]
  //
  // Calculate worldForce = Rz * linkForce:
  // worldForce_x = (√2/2)*1 + (-√2/2)*0 + (0)*0 = √2/2 ≈ 0.7071
  // worldForce_y = (√2/2)*1 + (√2/2)*0 + (0)*0 = √2/2 ≈ 0.7071
  // worldForce_z = (0)*1 + (0)*0 + (1)*0 = 0
  //
  // Thus, worldForce ≈ [√2/2, √2/2, 0] ≈ [0.7071, 0.7071, 0]
  //
  // Expected torque in world frame
  // First, rotate the inertia pose position to the world frame
  // using the link's world pose rotation.
  // Inertia pose position: (1, 2, 3)
  // Link world pose rotation matrix (around Z-axis by π/4 radians):
  // R_world = [ √2/2   -√2/2    0
  //            √2/2    √2/2    0
  //             0       0      1]
  //
  // Rotate inertia pose position to world frame:
  // r_world = R_world * [1, 2, 3] = [-√2/2, 3√2/2, 3]
  //
  // Then, calculate the cross product with the world force
  // to get the expected torque.
  // World force: (√2/2, √2/2, 0)
  // Cross product:
  // expectedTorque = r_world × F_world
  // expectedTorque = (-3√2/2, 3√2/2, -2)
  //
  // After the calculations the expected wrench is
  const double expectedForceX = std::sqrt(2) / 2;
  const double expectedForceY = std::sqrt(2) / 2;
  const double expectedForceZ = 0;
  const double expectedTorqueX = -3 * std::sqrt(2)/2;
  const double expectedTorqueY = 3 * std::sqrt(2)/2;
  const double expectedTorqueZ = -2;

  // verify Wrench values
  auto wrenchMsg = wrenchComp->Data();

  // Looser tolerances are needed for the nonzero terms
  EXPECT_NEAR(expectedForceX, wrenchMsg.force().x(), 1e-2);
  EXPECT_NEAR(expectedForceY, wrenchMsg.force().y(), 1e-2);
  EXPECT_NEAR(expectedForceZ, wrenchMsg.force().z(), 1e-6);
  EXPECT_NEAR(expectedTorqueX, wrenchMsg.torque().x(), 1e-2);
  EXPECT_NEAR(expectedTorqueY, wrenchMsg.torque().y(), 1e-2);
  EXPECT_NEAR(expectedTorqueZ, wrenchMsg.torque().z(), 1e-2);

  // apply opposite force. Since the cmd is not processed yet, this should
  // cancel out the existing wrench cmd
  link.AddForceInInertialFrame(ecm, -force);
  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

  // Add force in Inertial Frame at an offset
  math::Vector3d offset{0.0, 1.0, 0.0};
  link.AddForceInInertialFrame(ecm, force, offset);

  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);

  // Summary of dynamic state at this time in the test:
  // - The link has a world pose with position (1, 0, 0)
  // and orientation (0, 0, π/4).
  // - The link has an inertial pose with position (1, 2, 3)
  // and orientation (0, π/2, 0).
  // - A force of (0, 0, 1) N is appled at an offset (0, 1, 0),
  // both force and offset are expressed in terms of the inertial frame.
  // Calculate the offset expressed in terms of link's coordinate frame
  // First, create a pose for the offset in the inertial frame.
  // offset = (0, 1, 0)
  // forceApplicationRelativeToInertialFrame =
  // Pose3d(offset, Quaterniond::Identity)
  // This pose represents a translation by the
  // offset vector without any rotation.
  // Transform this pose by the inertial pose to get the
  // offset in the link's frame.
  // Inertial pose: (1, 2, 3) with orientation (0, π/2, 0)
  // Inertial rotation matrix (around Y-axis by π/2 radians):
  // Ry = [ 0   0   1
  //        0   1   0
  //       -1   0   0 ]
  //
  // Inertial translation matrix:
  // T = [1 0n, 0 1]
  //     [0 1 0 2]
  //     [0 0 1 3]
  //     [0 0 0 1]
  //
  // Combine rotation and translation into a homogeneous transformation matrix:
  // H_inertial = [Ry | T] = [0  0  1 | 1]
  //              [0  1  0 | 2]
  //              [-1 0  0 | 3]
  //              [0  0  0 | 1]
  //
  // Apply this transformation to the offset pose:
  // offsetInLinkFrame = H_inertial * [0, 1, 0, 1]^T
  //                    = [0  0  1 | 1] * [0]
  //                      [0  1  0 | 2]   [1]
  //                      [-1 0  0 | 3]   [0]
  //                      [0  0  0 | 1]   [1]
  //                    = [1, 3, 3]
  // offsetInLInkFrame = [1, 3, 3]
  //
  // Calculate the force expressed in link's coordinate frame with offset
  // First, rotate the force by the inertia pose's rotation.
  // Inertia pose rotation matrix (around Y-axis by π/2 radians):
  // Ry = [ 0   0   1
  //        0   1   0
  //       -1   0   0 ]
  // Force in inertial frame: (0, 0, 1)
  // linkForceWithOffset = Ry * force = [1, 0, 0]
  // Then, rotate this force by the link's world pose
  // rotation to get the world force.
  // Link world pose rotation matrix (around Z-axis by π/4 radians):
  // Rz = [ √2/2   -√2/2    0
  //        √2/2    √2/2    0
  //          0       0      1 ]
  // worldForceWithOffset = Rz * linkForceWithOffset = [√2/2, √2/2, 0]
  //
  // Calculate the expected torque with offset
  // First, calculate the effective position vector in the world frame.
  // offsetInLinkFrame = [1, 3, 3]
  // inertiaPose.Pos() = [1, 2, 3]
  // effectivePosition = offsetInLinkFrame + inertiaPose.Pos() = [2, 5, 6]
  // Rotate this effective position by the link's world pose rotation:
  // effectivePositionWorld = Rz * effectivePosition
  //                        = [-3√2/2, 7√2/2, 6]
  // Calculate the cross product with the world force to get the torque.
  // worldForceWithOffset = [√2/2, √2/2, 0]
  // expectedTorqueWithOffset = effectivePositionWorld × worldForceWithOffset
  // expectedTorqueWithOffset = (-3√2, 3√2, -5)
  // After the calculations the expected wrench is
  const double expectedForceWithOffsetX = std::sqrt(2) / 2;
  const double expectedForceWithOffsetY = std::sqrt(2) / 2;
  const double expectedForceWithOffsetZ = 0;
  const double expectedTorqueWithOffsetX = -3 * std::sqrt(2);
  const double expectedTorqueWithOffsetY = 3 * std::sqrt(2);
  const double expectedTorqueWithOffsetZ = -5;

  wrenchMsg = wrenchComp->Data();

  // Looser tolerances are needed for the nonzero terms
  EXPECT_NEAR(expectedForceWithOffsetX, wrenchMsg.force().x(), 1e-2);
  EXPECT_NEAR(expectedForceWithOffsetY, wrenchMsg.force().y(), 1e-2);
  EXPECT_NEAR(expectedForceWithOffsetZ, wrenchMsg.force().z(), 1e-6);
  EXPECT_NEAR(expectedTorqueWithOffsetX, wrenchMsg.torque().x(), 1e-2);
  EXPECT_NEAR(expectedTorqueWithOffsetY, wrenchMsg.torque().y(), 1e-2);
  EXPECT_NEAR(expectedTorqueWithOffsetZ, wrenchMsg.torque().z(), 1e-2);

  // apply opposite force again and verify the resulting wrench values are zero
  link.AddForceInInertialFrame(ecm, -force, offset);
  wrenchComp = ecm.Component<components::ExternalWorldWrenchCmd>(eLink);
  EXPECT_NE(nullptr, wrenchComp);
  wrenchMsg = wrenchComp->Data();

  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.force().x(), wrenchMsg.force().y(), wrenchMsg.force().z()));
  EXPECT_EQ(math::Vector3d::Zero, math::Vector3d(
      wrenchMsg.torque().x(), wrenchMsg.torque().y(), wrenchMsg.torque().z()));

}
