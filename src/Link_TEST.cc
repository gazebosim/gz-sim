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

#include "ignition/gazebo/Link.hh"

/////////////////////////////////////////////////
TEST(LinkTest, Constructor)
{
  ignition::gazebo::Link linkNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, linkNull.Entity());

  ignition::gazebo::Entity id(3);
  ignition::gazebo::Link link(id);

  EXPECT_EQ(id, link.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Link link(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  ignition::gazebo::Link linkCopy(link); // NOLINT
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Link link(id);

  ignition::gazebo::Link linkCopy;
  linkCopy = link;
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveConstructor)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Link link(id);

  ignition::gazebo::Link linkMoved(std::move(link));
  EXPECT_EQ(id, linkMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveAssignmentOperator)
{
  ignition::gazebo::Entity id(3);
  ignition::gazebo::Link link(id);

  ignition::gazebo::Link linkMoved;
  linkMoved = std::move(link);
  EXPECT_EQ(id, linkMoved.Entity());
}
