/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/math/Pose3.hh>
#include "ComponentFactory.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(ComponentFactory, AdjacentMemorySingleComponentType)
{
  gazebo::ComponentFactory factory;

  std::vector<ignition::math::Pose3d> poses;
  std::vector<gazebo::ComponentKey> keys;

  int count = 100000;

  // Create the components.
  for(int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    keys.push_back(factory.CreateComponent(poses.back()));
  }
  ASSERT_EQ(poses.size(), keys.size());

  int poseSize = sizeof(ignition::math::Pose3d);
  const ignition::math::Pose3d *pose = nullptr, *prevPose = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = factory.Component<ignition::math::Pose3d>(keys[i]);
    if (prevPose != nullptr)
    {
      EXPECT_EQ(poseSize, pose - prevPose);
    }
  }
}

/////////////////////////////////////////////////
TEST(ComponentFactory, AdjacentMemoryTwoComponentTypes)
{
  gazebo::ComponentFactory factory;

  std::vector<ignition::math::Pose3d> poses;
  std::vector<int> ints;
  std::vector<gazebo::ComponentKey> poseKeys;
  std::vector<gazebo::ComponentKey> intKeys;

  int count = 100000;

  // Create the components.
  for(int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    ints.push_back(i);

    poseKeys.push_back(factory.CreateComponent(poses.back()));
    intKeys.push_back(factory.CreateComponent(ints.back()));
  }
  ASSERT_EQ(static_cast<size_t>(count), poses.size());
  ASSERT_EQ(static_cast<size_t>(count), ints.size());
  ASSERT_EQ(static_cast<size_t>(count), poseKeys.size());
  ASSERT_EQ(static_cast<size_t>(count), intKeys.size());

  int poseSize = sizeof(ignition::math::Pose3d);
  int intSize = sizeof(int);
  const ignition::math::Pose3d *pose = nullptr, *prevPose = nullptr;
  const int *it = nullptr, *prevIt = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = factory.Component<ignition::math::Pose3d>(poseKeys[i]);
    it = factory.Component<int>(intKeys[i]);
    if (prevPose != nullptr)
    {
      EXPECT_EQ(poseSize, pose - prevPose);
    }

    if (prevIt != nullptr)
    {
      EXPECT_EQ(intSize, it - prevIt);
    }
  }
}
