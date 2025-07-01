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
#include <gz/sim/components/AxisAlignedBox.hh>
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
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Visual.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Link.hh>

#include <sdf/Collision.hh>
#include <sdf/Box.hh>
#include <sdf/Sphere.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Ellipsoid.hh>

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
TEST_F(LinkIntegrationTest, SensorByName)
{
  EntityComponentManager ecm;

  // Link
  auto eLink = ecm.CreateEntity();
  Link link(eLink);
  EXPECT_EQ(eLink, link.Entity());
  EXPECT_EQ(0u, link.SensorCount(ecm));

  // Sensor
  auto eSensor = ecm.CreateEntity();
  ecm.CreateComponent<components::Sensor>(eSensor, components::Sensor());
  ecm.CreateComponent<components::ParentEntity>(eSensor,
      components::ParentEntity(eLink));
  ecm.CreateComponent<components::Name>(eSensor,
      components::Name("sensor_name"));

  // Check link
  EXPECT_EQ(eSensor, link.SensorByName(ecm, "sensor_name"));
  EXPECT_EQ(1u, link.SensorCount(ecm));
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

/////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, AxisAlignedBoxInLink)
{
  // linkA
  //  - collisionAA (box)
  //  - collisionAB (sphere)
  //
  // linkB
  //  - collisionBA (ellipsoid)
  //  - collisionBB (mesh)
  //
  // linkC

  gz::sim::EntityComponentManager ecm;
  gz::sim::EventManager evm;
  gz::sim::SdfEntityCreator creator(ecm, evm);

  sdf::Collision collision;
  sdf::Geometry geom;
  sdf::ElementPtr sdf(new sdf::Element());

  // Link A
  auto eLinkA = ecm.CreateEntity();
  ecm.CreateComponent(eLinkA, gz::sim::components::Link());
  ecm.CreateComponent(eLinkA, gz::sim::components::Name("linkA"));

  // Create box collision AA - Child of Link A
  sdf::Box box;
  box.SetSize(gz::math::Vector3d(2, 2, 2));
  geom.SetType(sdf::GeometryType::BOX);
  geom.SetBoxShape(box);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(2.0, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkA);

  // Create sphere collision AB - Child of Link A
  sdf::Sphere sphere;
  sphere.SetRadius(2.0);
  geom.SetType(sdf::GeometryType::SPHERE);
  geom.SetSphereShape(sphere);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(-3.0, 2.0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkA);

  // LinkA - bounding box from collisions
  gz::sim::Link linkA(eLinkA);
  auto aabb = linkA.ComputeAxisAlignedBox(ecm);

  EXPECT_TRUE(aabb.has_value());
  EXPECT_EQ(-5.0, aabb->Min().X());
  EXPECT_EQ(-1.0, aabb->Min().Y());
  EXPECT_EQ(-2.0, aabb->Min().Z());
  EXPECT_EQ(3.0, aabb->Max().X());
  EXPECT_EQ(4.0, aabb->Max().Y());
  EXPECT_EQ(2.0, aabb->Max().Z());

  // By default, the bounding box checks are disabled.
  // Once enabled, the result must match the computed AABB.
  EXPECT_EQ(std::nullopt, linkA.AxisAlignedBox(ecm));
  linkA.EnableBoundingBoxChecks(ecm);
  EXPECT_EQ(aabb, linkA.AxisAlignedBox(ecm));

  // If ECM is updated with new link AABB, the ECM value is returned
  // instead of recomputing the AABB from the collisions
  math::AxisAlignedBox newAabb(-math::Vector3d::One, math::Vector3d::One);
  ecm.SetComponentData<components::AxisAlignedBox>(eLinkA, newAabb);
  EXPECT_NE(newAabb, aabb.value());
  EXPECT_EQ(newAabb, linkA.AxisAlignedBox(ecm));

  // If the bounding box checks are disabled and then re-enabled, the
  // bounding box should be recalculated from the collisions
  linkA.EnableBoundingBoxChecks(ecm, false);
  EXPECT_EQ(std::nullopt, linkA.AxisAlignedBox(ecm));
  linkA.EnableBoundingBoxChecks(ecm);
  EXPECT_EQ(aabb, linkA.AxisAlignedBox(ecm));

  // Link B
  auto eLinkB = ecm.CreateEntity();
  ecm.CreateComponent(eLinkB, gz::sim::components::Link());
  ecm.CreateComponent(eLinkB, gz::sim::components::Name("linkB"));

  // Create ellipsoid collision BA - Child of Link B
  sdf::Ellipsoid ellipsoid;
  ellipsoid.SetRadii(gz::math::Vector3d(0.5, 0.6, 0.1));
  geom.SetType(sdf::GeometryType::ELLIPSOID);
  geom.SetEllipsoidShape(ellipsoid);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(1, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkB);

  // Create mesh collision BB - Child of Link B
  sdf::Mesh mesh;
  mesh.SetUri("name://unit_box");
  geom.SetType(sdf::GeometryType::MESH);
  geom.SetMeshShape(mesh);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(-1, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkB);

  // LinkB - bounding box from collisions
  gz::sim::Link linkB(eLinkB);
  aabb = linkB.ComputeAxisAlignedBox(ecm);

  EXPECT_TRUE(aabb.has_value());
  EXPECT_EQ(-1.5, aabb->Min().X());
  EXPECT_EQ(-0.6, aabb->Min().Y());
  EXPECT_EQ(-0.5, aabb->Min().Z());
  EXPECT_EQ(1.5, aabb->Max().X());
  EXPECT_EQ(0.6, aabb->Max().Y());
  EXPECT_EQ(0.5, aabb->Max().Z());

  // Link C
  auto eLinkC = ecm.CreateEntity();
  ecm.CreateComponent(eLinkC, gz::sim::components::Link());
  ecm.CreateComponent(eLinkC, gz::sim::components::Name("linkC"));

  // LinkC - No collisions (AABB should be invalid if checks are enabled)
  gz::sim::Link linkC(eLinkC);
  EXPECT_EQ(std::nullopt, linkC.AxisAlignedBox(ecm));
  EXPECT_EQ(std::nullopt, linkC.ComputeAxisAlignedBox(ecm));
  linkC.EnableBoundingBoxChecks(ecm);
  EXPECT_EQ(math::AxisAlignedBox(), linkC.AxisAlignedBox(ecm));

  // Add invalid mesh collision to LinkC
  sdf::Mesh meshInvalid;
  meshInvalid.SetUri("invalid_uri");
  meshInvalid.SetFilePath("invalid_filepath");
  geom.SetType(sdf::GeometryType::MESH);
  geom.SetMeshShape(meshInvalid);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(0, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkC);

  // LinkC - Invalid mesh is the only collision present, bounding
  // box should be invalid (aab.Min > aab.Max) and a warning
  // should be printed
  linkC = gz::sim::Link(eLinkC);
  linkC.EnableBoundingBoxChecks(ecm, false);
  EXPECT_EQ(std::nullopt, linkC.AxisAlignedBox(ecm));
  EXPECT_EQ(math::AxisAlignedBox(), linkC.ComputeAxisAlignedBox(ecm));
  linkC.EnableBoundingBoxChecks(ecm);
  EXPECT_EQ(math::AxisAlignedBox(), linkC.AxisAlignedBox(ecm));

  // Add valid mesh collision to LinkC
  mesh.SetUri("name://unit_box");
  geom.SetMeshShape(mesh);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(0, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLinkC);

  // LinkC - Invalid mesh will be skipped with warning, bounding
  // box will be defined for the valid mesh only
  linkC = gz::sim::Link(eLinkC);
  aabb = linkC.ComputeAxisAlignedBox(ecm);

  EXPECT_TRUE(aabb.has_value());
  EXPECT_EQ(-0.5, aabb->Min().X());
  EXPECT_EQ(-0.5, aabb->Min().Y());
  EXPECT_EQ(-0.5, aabb->Min().Z());
  EXPECT_EQ(0.5, aabb->Max().X());
  EXPECT_EQ(0.5, aabb->Max().Y());
  EXPECT_EQ(0.5, aabb->Max().Z());

  // Reparenting invalid mesh collision from LinkC to LinkB
  // should have the same effect as above but without a warning
  auto eCollisions = ecm.ChildrenByComponents(
    eLinkC, gz::sim::components::Collision());
  creator.SetParent(eCollisions[0], eLinkB);

  linkC = gz::sim::Link(eLinkC);
  aabb = linkC.ComputeAxisAlignedBox(ecm);

  EXPECT_TRUE(aabb.has_value());
  EXPECT_EQ(-0.5, aabb->Min().X());
  EXPECT_EQ(-0.5, aabb->Min().Y());
  EXPECT_EQ(-0.5, aabb->Min().Z());
  EXPECT_EQ(0.5, aabb->Max().X());
  EXPECT_EQ(0.5, aabb->Max().Y());
  EXPECT_EQ(0.5, aabb->Max().Z());
}

/////////////////////////////////////////////////
TEST_F(LinkIntegrationTest, AxisAlignedBoxInWorld)
{
  // model
  //  - link
  //    - collision (mesh)

  gz::sim::EntityComponentManager ecm;
  gz::sim::EventManager evm;
  gz::sim::SdfEntityCreator creator(ecm, evm);

  sdf::Collision collision;
  sdf::Geometry geom;
  sdf::ElementPtr sdf(new sdf::Element());

  // Model with translation only pose wrt world
  auto modelPoseInWorld = gz::math::Pose3d(1023, -932, 378, 0, 0, 0);
  auto eModel = ecm.CreateEntity();
  ecm.CreateComponent(eModel, gz::sim::components::Model());
  ecm.CreateComponent(eModel, gz::sim::components::Name("model"));
  ecm.CreateComponent(eModel, gz::sim::components::Pose(modelPoseInWorld));

  // Link with translation only pose wrt model
  auto linkPoseInModel = gz::math::Pose3d(0.1, -0.2, 0.3, 0, 0, 0);
  auto eLink = ecm.CreateEntity();
  ecm.CreateComponent(eLink, gz::sim::components::Link());
  ecm.CreateComponent(eLink, gz::sim::components::Name("link"));
  ecm.CreateComponent(eLink, gz::sim::components::Pose(linkPoseInModel));
  creator.SetParent(eLink, eModel);

  // Create mesh collision - Child of Link
  sdf::Mesh mesh;
  mesh.SetUri("name://unit_box");
  geom.SetType(sdf::GeometryType::MESH);
  geom.SetMeshShape(mesh);
  collision.Load(sdf);
  collision.SetGeom(geom);
  collision.SetRawPose(gz::math::Pose3d(0, 0, 0, 0, 0, 0));
  creator.SetParent(creator.CreateEntities(&collision), eLink);

  // Model - bounding box from link collision
  gz::sim::Link link(eLink);
  link.EnableBoundingBoxChecks(ecm);
  auto aabbInWorld = link.WorldAxisAlignedBox(ecm);

  EXPECT_TRUE(aabbInWorld.has_value());
  EXPECT_EQ(1023 + 0.1 - 0.5, aabbInWorld->Min().X());
  EXPECT_EQ(1023 + 0.1 + 0.5, aabbInWorld->Max().X());
  EXPECT_EQ(-932 - 0.2 - 0.5, aabbInWorld->Min().Y());
  EXPECT_EQ(-932 - 0.2 + 0.5, aabbInWorld->Max().Y());
  EXPECT_EQ(378 + 0.3 - 0.5, aabbInWorld->Min().Z());
  EXPECT_EQ(378 + 0.3 + 0.5, aabbInWorld->Max().Z());
}
