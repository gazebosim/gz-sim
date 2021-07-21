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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/TestFixture.hh>

//////////////////////////////////////////////////
// Test that an object falls due to gravity
TEST(ExampleTests, Gravity)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  ignition::gazebo::TestFixture fixture("../gravity.sdf");

  bool checked{false};

  // This callback is called every simulation iteration
  fixture.OnPostUpdate(
    [&checked](const ignition::gazebo::UpdateInfo &,
    const ignition::gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<ignition::gazebo::components::Model,
                ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity &,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Pose *_pose)->bool
        {
          if (nullptr == _pose)
            return testing::AssertionFailure();

          // TODO: check pose

          return true;
        });
        checked = true;
    });

  // Setup simulation server
  fixture.Server()->Run(true, 1000, false);

  EXPECT_TRUE(checked);
}
