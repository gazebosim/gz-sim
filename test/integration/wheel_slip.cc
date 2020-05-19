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
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include <sdf/Sphere.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test DiffDrive system
class WheelSlipTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};
/*
TEST_F(WheelSlipTest, TireDrum)
{
  const double metersPerMile = 1609.34;
  const double secondsPerHour = 3600.0;

  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "test/worlds/tire_drum.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
}
*/
TEST_F(WheelSlipTest, TricyclesUphill)
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "examples/worlds/trisphere_cycle_wheel_slip.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  gazebo::EntityComponentManager *ecm = nullptr;
  Relay testSystem;
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

  auto collisionGeometry = ecm->Component<components::Geometry>(wheelRearLeftCollisionEntity);
  EXPECT_EQ(sdf::GeometryType::SPHERE, collisionGeometry->Data().Type());
  EXPECT_NE(nullptr, collisionGeometry->Data().SphereShape());

  const double wheelRadius = collisionGeometry->Data().SphereShape()->Radius();
  EXPECT_DOUBLE_EQ(0.15, wheelRadius);
  
  // Get rear wheel spins of both models
  Entity wheelRearLeftSpin0Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("wheel_rear_left_spin"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelRearLeftSpin0Entity);
  
  Entity wheelRearRightSpin0Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle0Entity),
      components::Name("wheel_rear_right_spin"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelRearRightSpin0Entity);
  
  Entity wheelRearLeftSpin1Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle1Entity),
      components::Name("wheel_rear_left_spin"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelRearLeftSpin1Entity);
  
  Entity wheelRearRightSpin1Entity = ecm->EntityByComponents(
      components::ParentEntity(trisphereCycle1Entity),
      components::Name("wheel_rear_right_spin"),
      components::Link());

  EXPECT_NE(gazebo::kNullEntity, wheelRearRightSpin1Entity);

  // Set speed of both models
  const double angularSpeed = 6.0;

  ecm->CreateComponent(
        wheelRearLeftSpin0Entity,
        components::JointVelocityCmd({angularSpeed})
      );
  
  ecm->CreateComponent(
        wheelRearRightSpin0Entity,
        components::JointVelocityCmd({angularSpeed})
      );

  ecm->CreateComponent(
        wheelRearLeftSpin1Entity,
        components::JointVelocityCmd({angularSpeed})
      );
  
  ecm->CreateComponent(
        wheelRearRightSpin1Entity,
        components::JointVelocityCmd({angularSpeed})
      );

  server.Run(true, 2000, false);

  // compute expected slip
  // normal force as passed to Wheel Slip in test world
  const double wheelNormalForce = 32;
  const double mass = 14.5;
  const double forceRatio = (mass/2) * std::abs(gravity->Data().X()) / wheelNormalForce;
  const double noSlipLinearSpeed = wheelRadius * angularSpeed;

  auto wheelRearLeftVelocity = ecm->Component<components::JointVelocity>(wheelRearLeftSpin0Entity);
  auto wheelRearRightVelocity = ecm->Component<components::JointVelocity>(wheelRearRightSpin0Entity);
  auto worldVel = ecm->Component<components::WorldLinearVelocity>(trisphereCycle0Entity);

  EXPECT_NE(nullptr, wheelRearLeftVelocity);
  EXPECT_NE(nullptr, wheelRearRightVelocity);
  EXPECT_NE(nullptr, worldVel);

  EXPECT_NEAR(angularSpeed, wheelRearLeftVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(angularSpeed, wheelRearRightVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(noSlipLinearSpeed - worldVel->Data()[0], 0.0, 5e-3);
  
  wheelRearLeftVelocity = ecm->Component<components::JointVelocity>(wheelRearLeftSpin1Entity);
  wheelRearRightVelocity = ecm->Component<components::JointVelocity>(wheelRearRightSpin1Entity);
  worldVel = ecm->Component<components::WorldLinearVelocity>(trisphereCycle1Entity);

  EXPECT_NE(nullptr, wheelRearLeftVelocity);
  EXPECT_NE(nullptr, wheelRearRightVelocity);
  EXPECT_NE(nullptr, worldVel);

  EXPECT_NEAR(angularSpeed, wheelRearLeftVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(angularSpeed, wheelRearRightVelocity->Data()[0], 3e-3);
  EXPECT_NEAR(noSlipLinearSpeed - worldVel->Data()[0], noSlipLinearSpeed * forceRatio, 5e-3);
}
