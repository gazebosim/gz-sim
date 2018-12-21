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

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;
using namespace gazebo;

class ComponentsTest : public ::testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(ComponentsTest, CanonicalLink)
{
  // Create components
  auto comp1 = components::CanonicalLink();
  auto comp2 = components::CanonicalLink();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Collision)
{
  // Create components
  auto comp1 = components::Collision();
  auto comp2 = components::Collision();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Joint)
{
  // Create components
  auto comp1 = components::Joint();
  auto comp2 = components::Joint();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Link)
{
  // Create components
  auto comp1 = components::Link();
  auto comp2 = components::Link();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Model)
{
  // Create components
  auto comp1 = components::Model();
  auto comp2 = components::Model();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Visual)
{
  // Create components
  auto comp1 = components::Visual();
  auto comp2 = components::Visual();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, World)
{
  // Create components
  auto comp1 = components::World();
  auto comp2 = components::World();

  // Equality operators
  EXPECT_EQ(comp1, comp2);
  EXPECT_TRUE(comp1 == comp2);
  EXPECT_FALSE(comp1 != comp2);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ChildLinkName)
{
  // Create components
  auto comp11 = components::ChildLinkName("comp1");
  auto comp12 = components::ChildLinkName("comp1");
  auto comp2 = components::ChildLinkName("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Name)
{
  // Create components
  auto comp11 = components::Name("comp1");
  auto comp12 = components::Name("comp1");
  auto comp2 = components::Name("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ParentEntity)
{
  // Create components
  auto comp11 = components::ParentEntity(1);
  auto comp12 = components::ParentEntity(1);
  auto comp2 = components::ParentEntity(2);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, ParentLinkName)
{
  // Create components
  auto comp11 = components::ParentLinkName("comp1");
  auto comp12 = components::ParentLinkName("comp1");
  auto comp2 = components::ParentLinkName("comp2");

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Pose)
{
  // Create components
  auto comp11 = components::Pose(math::Pose3d(1, 0, 0, 0, 0, 0));
  auto comp12 = components::Pose(math::Pose3d(1, 0, 0, 0, 0, 0));
  auto comp2 = components::Pose(math::Pose3d(2, 0, 0, 0, 0, 0));

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}

/////////////////////////////////////////////////
TEST_F(ComponentsTest, Static)
{
  // Create components
  auto comp11 = components::Static(true);
  auto comp12 = components::Static(true);
  auto comp2 = components::Static(false);

  // Equality operators
  EXPECT_EQ(comp11, comp12);
  EXPECT_NE(comp11, comp2);
  EXPECT_TRUE(comp11 == comp12);
  EXPECT_TRUE(comp11 != comp2);
  EXPECT_FALSE(comp11 == comp2);
  EXPECT_FALSE(comp11 != comp12);
}
