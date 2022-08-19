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

#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/transport/Node.hh>

using namespace std::chrono_literals;

//////////////////////////////////////////////////
// Test sending a command and verify that model reacts accordingly
TEST(ExampleTests, Command)
{
  // Maximum verbosity helps with debugging
  gz::common::Console::SetVerbosity(4);

  // Instantiate test fixture
  gz::sim::TestFixture fixture("../command.sdf");

  // Get the link that we'll be inspecting
  bool configured{false};
  gz::sim::Link link;
  fixture.OnConfigure(
    [&link, &configured](const gz::sim::Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/)
    {
      gz::sim::World world(_worldEntity);

      auto modelEntity = world.ModelByName(_ecm, "commanded");
      EXPECT_NE(gz::sim::kNullEntity, modelEntity);

      auto model = gz::sim::Model(modelEntity);

      auto linkEntity = model.LinkByName(_ecm, "link");
      EXPECT_NE(gz::sim::kNullEntity, linkEntity);

      link = gz::sim::Link(linkEntity);
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
  gz::math::Vector3d linVel;
  gz::math::Vector3d linAccel;
  fixture.OnPostUpdate(
    [&](
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
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
  gz::transport::Node node;

  gz::msgs::Twist msg;
  auto linVelMsg = msg.mutable_linear();
  linVelMsg->set_x(10);

  auto pub = node.Advertise<gz::msgs::Twist>("/model/commanded/cmd_vel");
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
