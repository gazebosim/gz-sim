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
#include <algorithm>
#include <string>
#include <vector>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Pose.hh>

using namespace ignition;
using namespace gazebo;
using namespace components;

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, Type)
{
  std::vector<std::string> comps = Factory::Components();
  EXPECT_FALSE(comps.empty());
  EXPECT_TRUE(std::find(comps.begin(), comps.end(),
      std::string("ign_gazebo_components.Altimeter")) != comps.end());
}

/////////////////////////////////////////////////
TEST(ComponentFactoryTest, New)
{
  auto comp = Factory::New<components::Pose>("__unknown_component__");
  ASSERT_TRUE(comp.get() == nullptr);

  comp = Factory::New<components::Pose>("ign_gazebo_components.Pose");
  ASSERT_TRUE(comp.get() != nullptr);
}
