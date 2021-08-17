/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/msgs/twist.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

using namespace std::chrono_literals;

//////////////////////////////////////////////////
// Test sending a command and verify that model reacts accordingly
TEST(ExampleTests, Command)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture
  ignition::gazebo::TestFixture fixture("../command.sdf");

  // Get the link that we'll be inspecting
  bool configured{false};
  ignition::gazebo::Link link;
  fixture.OnConfigure(
    [&link, &configured](const ignition::gazebo::Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager &/*_eventMgr*/)
    {
      ignition::gazebo::World world(_worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "commanded");
      EXPECT_NE(ignition::gazebo::kNullEntity, modelEntity);

      auto model = ignition::gazebo::Model(modelEntity);

      auto linkEntity = model.LinkByName(_ecm, "link");
      EXPECT_NE(ignition::gazebo::kNullEntity, linkEntity);

      link = ignition::gazebo::Link(linkEntity);
      EXPECT_TRUE(link.Valid(_ecm));

      // Tell Gazebo that we want to observe the link's velocity and acceleration
      link.EnableVelocityChecks(_ecm, true);
      link.EnableAccelerationChecks(_ecm, true);

      configured = true;
    })
  // Configure must be set before finalize, but other callbacks can come after
  .Finalize();

  EXPECT_TRUE(configured);

  // Check that link is falling due to gravity
  int iterations{0};
  ignition::math::Vector3d linVel;
  ignition::math::Vector3d linAccel;
  fixture.OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      linVel = link.WorldLinearVelocity(_ecm).value();
      EXPECT_DOUBLE_EQ(0.0, linVel.Y());

      linAccel = link.WorldLinearAcceleration(_ecm).value();
      EXPECT_DOUBLE_EQ(0.0, linAccel.X());
      EXPECT_DOUBLE_EQ(0.0, linAccel.Y());

      iterations++;
    });

  int expectedIterations{10};
  fixture.Server()->Run(true, expectedIterations, false);
  EXPECT_EQ(expectedIterations, iterations);
  EXPECT_DOUBLE_EQ(0.0, linVel.X());
  EXPECT_GT(0.0, linVel.Z());
  EXPECT_GT(0.0, linAccel.Z());

  // Send velocity command
  ignition::transport::Node node;

  ignition::msgs::Twist msg;
  auto linVelMsg = msg.mutable_linear();
  linVelMsg->set_x(10);

  auto pub = node.Advertise<ignition::msgs::Twist>("/model/commanded/cmd_vel");
  pub.Publish(msg);

  // Commands sent through transport are processed asynchronously and may
  // take a while to execute, so we run a few iterations until it takes
  // effect.
  int sleep{0};
  int maxSleep{30};
  for (; sleep < maxSleep && linVel.X() < 0.1; ++sleep)
  {
    fixture.Server()->Run(true, 10, false);
    expectedIterations+= 10;
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_LT(sleep, maxSleep);
  EXPECT_EQ(expectedIterations, iterations);

  EXPECT_DOUBLE_EQ(10.0, linVel.X());
  EXPECT_GT(0.0, linVel.Z());
  EXPECT_GT(0.0, linAccel.Z());
}
