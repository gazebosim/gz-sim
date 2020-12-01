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

#include "ignition/gazebo/World.hh"

/////////////////////////////////////////////////
TEST(WorldTest, Constructor)
{
  ignition::gazebo::World worldNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, worldNull.Entity());

  ignition::gazebo::Entity id(3);
  ignition::gazebo::World world(id);

  EXPECT_EQ(id, world.Entity());
}

/////////////////////////////////////////////////
TEST(WorldTest, CopyConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::World world(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  ignition::gazebo::World worldCopy(world); // NOLINT
  EXPECT_EQ(world.Entity(), worldCopy.Entity());
}

/////////////////////////////////////////////////
TEST(WorldTest, CopyAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::World world(id);

  ignition::gazebo::World worldCopy;
  worldCopy = world;
  EXPECT_EQ(world.Entity(), worldCopy.Entity());
}

/////////////////////////////////////////////////
TEST(WorldTest, MoveConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::World world(id);

  ignition::gazebo::World worldMoved(std::move(world));
  EXPECT_EQ(id, worldMoved.Entity());
}

/////////////////////////////////////////////////
TEST(WorldTest, MoveAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::World world(id);

  ignition::gazebo::World worldMoved;
  worldMoved = std::move(world);
  EXPECT_EQ(id, worldMoved.Entity());
}
