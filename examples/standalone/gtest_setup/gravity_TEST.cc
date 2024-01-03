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

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>

//////////////////////////////////////////////////
// Test that an object falls due to gravity
TEST(ExampleTests, Gravity)
{
  // Maximum verbosity helps with debugging
  gz::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  gz::sim::TestFixture fixture("../gravity.sdf");

  int iterations{0};
  gz::sim::Entity modelEntity;
  gz::math::Vector3d gravity;

  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&modelEntity, &gravity](const gz::sim::Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/)
    {
      // Get gravity
      gz::sim::World world(_worldEntity);
      gravity = world.Gravity(_ecm).value();

      // Get falling entity
      modelEntity = world.ModelByName(_ecm, "falling");
      EXPECT_NE(gz::sim::kNullEntity, modelEntity);
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &modelEntity, &gravity](
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
    {
      // Inspect all model poses
      auto pose = gz::sim::worldPose(modelEntity, _ecm);

      EXPECT_DOUBLE_EQ(0.0, pose.Pos().X());
      EXPECT_DOUBLE_EQ(0.0, pose.Pos().Y());

      // Check that model is falling due to gravity
      // -g * t^2 / 2
      auto time = std::chrono::duration<double>(_info.simTime).count();
      EXPECT_NEAR(gravity.Z() * time * time * 0.5, pose.Pos().Z(), 1e-2);

      iterations++;
    }).
  // The moment we finalize, the configure callback is called
  Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 1000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(1000, iterations);
}
