/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/msgs/entity_factory.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Sphere.hh>
#include <sdf/Cylinder.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/SlipComplianceCmd.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test DiffDrive system
class WheelSlipTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
             {
               common::Console::SetVerbosity(4);
               ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                   (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
             }
  /// \brief Class to hold parameters for tire tests.
  public: class WheelSlipState
  {
    /// \brief Constructor.
    public: WheelSlipState()
    {
    }

    /// \brief Destructor.
    public: ~WheelSlipState() = default;

    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLateral = 0.0;

    /// \brief Axel force in lateral direction to expect.
    public: double axelForceLongitudinal = 0.0;

    /// \brief Description to print during test loop.
    public: std::string description;

    /// \brief Drum spin speed in rad/s.
    public: double drumSpeed = 0.0;

    /// \brief Steer angle to apply.
    public: ignition::math::Angle steer;

    /// \brief Suspension force to apply in N.
    public: double suspForce = 0.0;

    /// \brief Wheel slip compliance in lateral direction;
    public: double wheelSlipComplianceLateral = 0.01;

    /// \brief Wheel slip compliance in longitudinal direction;
    public: double wheelSlipComplianceLongitudinal = 0.01;

    /// \brief Wheel spin speed in rad/s.
    public: double wheelSpeed = 0.0;

    /// \brief P gain with wheel spin speed.
    public: double wheelSpeedGain = 0.0;

    /// \brief Wheel torque in Nm.
    public: double wheelTorque = 0.0;
  };
};

TEST_F(WheelSlipTest, TireDrum)
{
  const double metersPerMile = 1609.34;
  const double secondsPerHour = 3600.0;
  std::vector<std::string> linksToCheck{"wheel", "axle", "upright"};

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/tire_drum.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  gazebo::EntityComponentManager *ecm = nullptr;
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
        gazebo::EntityComponentManager &_ecm)
      {
      ecm = &_ecm;
      });

  // Create a system that records the vehicle poses
  std::vector<math::Pose3d> poses;
  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Get world and gravity
  Entity worldEntity =
    ecm->EntityByComponents(components::World());

  EXPECT_NE(gazebo::kNullEntity, worldEntity);

  // Get both models
  Entity tireEntity =
    ecm->EntityByComponents(components::Model(),
        components::Name("tire"));

  Entity drumEntity =
    ecm->EntityByComponents(components::Model(),
        components::Name("drum"));

  EXPECT_NE(gazebo::kNullEntity, tireEntity);
  EXPECT_NE(gazebo::kNullEntity, drumEntity);

  Entity wheelLinkEntity = ecm->EntityByComponents(
      components::ParentEntity(tireEntity),
      components::Name("wheel"),
      components::Link());

  Entity drumLinkEntity = ecm->EntityByComponents(
      components::ParentEntity(drumEntity),
      components::Name("link"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelLinkEntity);
  EXPECT_NE(gazebo::kNullEntity, drumLinkEntity);

  auto wheelInertialComp =
    ecm->Component<components::Inertial>(wheelLinkEntity);

  auto drumInertialComp =
    ecm->Component<components::Inertial>(drumLinkEntity);

  EXPECT_NE(nullptr, wheelInertialComp);
  EXPECT_NE(nullptr, drumInertialComp);

  const double wheelMass = wheelInertialComp->Data().MassMatrix().Mass();
  const double drumMass = drumInertialComp->Data().MassMatrix().Mass();

  EXPECT_DOUBLE_EQ(2.5, wheelMass);
  EXPECT_DOUBLE_EQ(10.0, drumMass);

  auto collisionWheelLinkEntity = ecm->EntityByComponents(
      components::ParentEntity(wheelLinkEntity),
      components::Name("collision"),
      components::Collision());

  auto collisionDrumLinkEntity = ecm->EntityByComponents(
      components::ParentEntity(drumLinkEntity),
      components::Name("collision"),
      components::Collision());

  EXPECT_NE(gazebo::kNullEntity, collisionWheelLinkEntity);
  EXPECT_NE(gazebo::kNullEntity, collisionDrumLinkEntity);

  auto wheelCollisionComp =
    ecm->Component<components::CollisionElement>(collisionWheelLinkEntity);

  auto drumCollisionComp =
    ecm->Component<components::CollisionElement>(collisionDrumLinkEntity);

  ASSERT_NE(nullptr, wheelCollisionComp);
  ASSERT_NE(nullptr, drumCollisionComp);

  ASSERT_TRUE(
      (wheelCollisionComp->Data().Geom()->Type() ==
       sdf::GeometryType::SPHERE) ||
      (wheelCollisionComp->Data().Geom()->Type() ==
       sdf::GeometryType::CYLINDER));

  ASSERT_TRUE(
      (drumCollisionComp->Data().Geom()->Type() ==
       sdf::GeometryType::SPHERE) ||
      (drumCollisionComp->Data().Geom()->Type() ==
       sdf::GeometryType::CYLINDER));

  double wheelRadius = 0.0;
  double drumRadius = 0.0;

  if (wheelCollisionComp->Data().Geom()->Type() == sdf::GeometryType::SPHERE)
    wheelRadius = wheelCollisionComp->Data().Geom()->SphereShape()->Radius();
  if (wheelCollisionComp->Data().Geom()->Type() == sdf::GeometryType::CYLINDER)
    wheelRadius = wheelCollisionComp->Data().Geom()->CylinderShape()->Radius();

  if (drumCollisionComp->Data().Geom()->Type() == sdf::GeometryType::SPHERE)
    drumRadius = drumCollisionComp->Data().Geom()->SphereShape()->Radius();
  if (drumCollisionComp->Data().Geom()->Type() == sdf::GeometryType::CYLINDER)
    drumRadius = drumCollisionComp->Data().Geom()->CylinderShape()->Radius();

  // TODO(anyone) enable below tests when kp is supported
  // auto elem = collisionComp->Data().Element();
  // ASSERT_TRUE(elem->HasElement("surface"));
  // auto surface = elem->GetElement("surface");
  // ASSERT_NE(nullptr, surface);
  // auto surfaceContact = surface->GetElement("contact");
  // auto surfaceContactOde = surfaceContact->GetElement("ode");
  // const double kp = surfaceContactOde->GetElement("kp")->Get<double>();
  // ASSERT_EQ(kp, 250e3);

  double modelMass = 0.0;
  for (const auto &linkName : linksToCheck)
  {
    Entity linkEntity = ecm->EntityByComponents(
        components::ParentEntity(tireEntity),
        components::Name(linkName),
        components::Link());
    EXPECT_NE(gazebo::kNullEntity, linkEntity);
    auto inertialComp = ecm->Component<components::Inertial>(linkEntity);

    EXPECT_NE(nullptr, inertialComp);

    modelMass += inertialComp->Data().MassMatrix().Mass();
  }

  // Get axle wheel and steer joint of wheel model
  Entity wheelAxleJointEntity = ecm->EntityByComponents(
      components::ParentEntity(tireEntity),
      components::Name("axle_wheel"),
      components::Joint());

  ASSERT_NE(gazebo::kNullEntity, wheelAxleJointEntity);

  Entity wheelSteerJointEntity = ecm->EntityByComponents(
      components::ParentEntity(tireEntity),
      components::Name("steer"),
      components::Joint());

  ASSERT_NE(gazebo::kNullEntity, wheelSteerJointEntity);

  const double wheelSpeed =
    -25.0 * metersPerMile / secondsPerHour / wheelRadius;

  double wheelNormalForce = 1000.0;
  double wheelSlip1 = 0.0;
  double wheelSlip2 = 0.0;
  double slipComplianceLateral = 0.1;
  double slipComplianceLongitudinal = 0.0;

  // Zero slip
  components::SlipComplianceCmd newSlipCmdComp({wheelSlip1, wheelSlip2});

  Entity tireCollisionEntity = ecm->EntityByComponents(
      components::ParentEntity(tireEntity),
      components::Name("collision"),
      components::Collision());

  auto currSlipCmdComp =
    ecm->Component<components::SlipComplianceCmd>(tireCollisionEntity);

  if (currSlipCmdComp)
    *currSlipCmdComp = newSlipCmdComp;
  else
    ecm->CreateComponent(tireCollisionEntity, newSlipCmdComp);

  // TODO(john): Complete below tests to be equivalent to Gazebo classic's tests
  WheelSlipState state0;
  state0.drumSpeed = -25.0 * metersPerMile / secondsPerHour / drumRadius;
  state0.wheelSpeed = -25.0 * metersPerMile / secondsPerHour / drumRadius;
  state0.wheelSpeedGain = 1e2;
  state0.suspForce = 1000.0;

  std::vector<WheelSlipState> states;
  {
    WheelSlipState state = state0;
    state.description = "Zero slip";
    state.steer.Degree(0.0);
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Lateral slip: low";
    state.steer.Degree(3.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = -state.suspForce *
        sin(state.steer.Radian()) / state.wheelSlipComplianceLateral;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Lateral slip: high";
    state.steer.Degree(10);
    state.wheelSpeed *= cos(state.steer.Radian());
    state.axelForceLateral = -state.suspForce;
    state.axelForceLongitudinal = 0.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Longitudinal torque control: low";
    state.wheelSpeed = -1.055 * state.drumSpeed * drumRadius / wheelRadius;
    state.wheelTorque = 0.25 * state.suspForce * wheelRadius;
    state.steer.Degree(0.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = -250.0;
    states.push_back(state);
  }
  {
    WheelSlipState state = state0;
    state.description = "Longitudinal torque control: moderate";
    state.wheelSpeed = -1.12 * state.drumSpeed * drumRadius / wheelRadius;
    state.wheelTorque = 0.5 * state.suspForce * wheelRadius;
    state.steer.Degree(0.0);
    state.wheelSlipComplianceLateral = 0.1;
    state.axelForceLateral = 0.0;
    state.axelForceLongitudinal = -600.0;
    states.push_back(state);
  }

  // Lateral slip: low
  wheelSlip1 = wheelSpeed / wheelNormalForce * slipComplianceLateral;
  wheelSlip2 = wheelSpeed / wheelNormalForce * slipComplianceLongitudinal;
  newSlipCmdComp = components::SlipComplianceCmd({wheelSlip1, wheelSlip2});
  ecm->CreateComponent(tireCollisionEntity, newSlipCmdComp);

  server.Run(true, 250, false);

  // Lateral slip: high
  slipComplianceLateral = 1.0;
  wheelSlip1 = wheelSpeed / wheelNormalForce * slipComplianceLateral;
  wheelSlip2 = wheelSpeed / wheelNormalForce * slipComplianceLongitudinal;
  newSlipCmdComp = components::SlipComplianceCmd({wheelSlip1, wheelSlip2});
  ecm->CreateComponent(tireCollisionEntity, newSlipCmdComp);

  server.Run(true, 250, false);
}

TEST_F(WheelSlipTest, TricyclesUphill)
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/trisphere_cycle_wheel_slip.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  gazebo::EntityComponentManager *ecm = nullptr;
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
        gazebo::EntityComponentManager &_ecm)
      {
      ecm = &_ecm;
      });

  // Create a system that records the vehicle poses
  std::vector<math::Pose3d> poses;
  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Get world and gravity
  Entity worldEntity =
    ecm->EntityByComponents(components::World());

  EXPECT_NE(gazebo::kNullEntity, worldEntity);

  auto gravity = ecm->Component<components::Gravity>(worldEntity);

  EXPECT_NE(nullptr, gravity);
  EXPECT_EQ(math::Vector3d(-2, 0, -9.8), gravity->Data());

  // Get both models
  Entity trisphereCycle0Entity =
    ecm->EntityByComponents(components::Model(),
        components::Name("trisphere_cycle0"));

  EXPECT_NE(gazebo::kNullEntity, trisphereCycle0Entity);


  Entity trisphereCycle1Entity =
    ecm->EntityByComponents(components::Model(),
        components::Name("trisphere_cycle1"));

  EXPECT_NE(gazebo::kNullEntity, trisphereCycle1Entity);

  // Check rear left wheel of first model
  Entity wheelRearLeftEntity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("wheel_rear_left"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelRearLeftEntity);

  Entity wheelRearLeftCollisionEntity = ecm->EntityByComponents(
      components::ParentEntity(wheelRearLeftEntity),
      components::Collision());

  auto collisionGeometry =
    ecm->Component<components::Geometry>(wheelRearLeftCollisionEntity);
  EXPECT_EQ(sdf::GeometryType::SPHERE, collisionGeometry->Data().Type());
  EXPECT_NE(nullptr, collisionGeometry->Data().SphereShape());

  const double wheelRadius = collisionGeometry->Data().SphereShape()->Radius();
  EXPECT_DOUBLE_EQ(0.15, wheelRadius);

  // Get rear wheel spins of both models
  Entity wheelRearLeftSpin0Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("wheel_rear_left_spin"),
      components::Joint());

  EXPECT_NE(gazebo::kNullEntity, wheelRearLeftSpin0Entity);

  Entity wheelRearRightSpin0Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("wheel_rear_right_spin"),
      components::Joint());

  EXPECT_NE(gazebo::kNullEntity, wheelRearRightSpin0Entity);

  Entity wheelRearLeftSpin1Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle1Entity),
      components::Name("wheel_rear_left_spin"),
      components::Joint());

  EXPECT_NE(gazebo::kNullEntity, wheelRearLeftSpin1Entity);

  Entity wheelRearRightSpin1Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle1Entity),
      components::Name("wheel_rear_right_spin"),
      components::Joint());

  EXPECT_NE(gazebo::kNullEntity, wheelRearRightSpin1Entity);

  // Set speed of both models
  const double angularSpeed = 6.0;
  ecm->CreateComponent(
      wheelRearLeftSpin0Entity,
      components::JointVelocityCmd({angularSpeed}));

  ecm->CreateComponent(
      wheelRearRightSpin0Entity,
      components::JointVelocityCmd({angularSpeed}));

  ecm->CreateComponent(
      wheelRearLeftSpin1Entity,
      components::JointVelocityCmd({angularSpeed}));

  ecm->CreateComponent(
      wheelRearRightSpin1Entity,
      components::JointVelocityCmd({angularSpeed}));

  // Get frame link of first model
  Entity trisphere0FrameEntity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("frame"),
      components::Link());

  // Get frame link of second model
  Entity trisphere1FrameEntity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle1Entity),
      components::Name("frame"),
      components::Link());

  auto worldVelTrisphere0 =
    ecm->Component<components::WorldLinearVelocity>(trisphere0FrameEntity);

  if (!worldVelTrisphere0)
  {
    ecm->CreateComponent(trisphere0FrameEntity,
        components::WorldLinearVelocity());
    worldVelTrisphere0 =
      ecm->Component<components::WorldLinearVelocity>(trisphere0FrameEntity);
  }

  auto worldVelTrisphere1 =
    ecm->Component<components::WorldLinearVelocity>(trisphere1FrameEntity);

  if (!worldVelTrisphere1)
  {
    ecm->CreateComponent(trisphere1FrameEntity,
        components::WorldLinearVelocity());
    worldVelTrisphere1 =
      ecm->Component<components::WorldLinearVelocity>(trisphere1FrameEntity);
  }

  test::Relay testSlipSystem;
  testSlipSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
        gazebo::EntityComponentManager &)
      {
      auto wheelRearLeftVelocity0Cmd =
        ecm->Component<components::JointVelocityCmd>(wheelRearLeftSpin0Entity);
      auto wheelRearRightVelocity0Cmd =
        ecm->Component<components::JointVelocityCmd>(wheelRearRightSpin0Entity);
      auto wheelRearLeftVelocity1Cmd =
        ecm->Component<components::JointVelocityCmd>(wheelRearLeftSpin1Entity);
      auto wheelRearRightVelocity1Cmd =
        ecm->Component<components::JointVelocityCmd>(wheelRearRightSpin1Entity);

      *wheelRearLeftVelocity0Cmd =
        components::JointVelocityCmd({angularSpeed});
      *wheelRearRightVelocity0Cmd =
        components::JointVelocityCmd({angularSpeed});
      *wheelRearLeftVelocity1Cmd =
        components::JointVelocityCmd({angularSpeed});
      *wheelRearRightVelocity1Cmd =
        components::JointVelocityCmd({angularSpeed});
      });
  server.AddSystem(testSlipSystem.systemPtr);
  server.Run(true, 2000, false);

  // compute expected slip
  // normal force as passed to Wheel Slip in test world
  const double wheelNormalForce = 32;
  const double mass = 14.5;
  const double forceRatio =
    (mass/2) * std::abs(gravity->Data().X()) / wheelNormalForce;
  const double noSlipLinearSpeed = wheelRadius * angularSpeed;

  auto wheelRearLeftVelocity =
    ecm->Component<components::JointVelocity>(wheelRearLeftSpin0Entity);
  auto wheelRearRightVelocity =
    ecm->Component<components::JointVelocity>(wheelRearRightSpin0Entity);
  worldVelTrisphere0 =
    ecm->Component<components::WorldLinearVelocity>(trisphere0FrameEntity);

  EXPECT_NE(nullptr, wheelRearLeftVelocity);
  EXPECT_NE(nullptr, wheelRearRightVelocity);
  EXPECT_NE(nullptr, worldVelTrisphere0);

  EXPECT_NEAR(angularSpeed, wheelRearLeftVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(angularSpeed, wheelRearRightVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(noSlipLinearSpeed - worldVelTrisphere0->Data()[0], 0.0, 5e-3);

  wheelRearLeftVelocity =
    ecm->Component<components::JointVelocity>(wheelRearLeftSpin1Entity);
  wheelRearRightVelocity =
    ecm->Component<components::JointVelocity>(wheelRearRightSpin1Entity);
  worldVelTrisphere1 =
    ecm->Component<components::WorldLinearVelocity>(trisphere1FrameEntity);

  EXPECT_NE(nullptr, wheelRearLeftVelocity);
  EXPECT_NE(nullptr, wheelRearRightVelocity);
  EXPECT_NE(nullptr, worldVelTrisphere1);

  EXPECT_NEAR(angularSpeed, wheelRearLeftVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(angularSpeed, wheelRearRightVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(noSlipLinearSpeed - worldVelTrisphere1->Data()[0],
      noSlipLinearSpeed * forceRatio, 5e-3);
}
