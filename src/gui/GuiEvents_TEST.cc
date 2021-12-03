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

#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

using namespace ignition;
using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TEST(GuiEventsTest, GuiNewRemovedEntities)
{
  events::GuiNewRemovedEntities event({1, 2, 3}, {4, 5});

  EXPECT_LT(QEvent::User, event.type());

  auto addedEntities = event.NewEntities();
  EXPECT_EQ(3u, addedEntities.size());
  EXPECT_NE(addedEntities.find(1), addedEntities.end());
  EXPECT_NE(addedEntities.find(2), addedEntities.end());
  EXPECT_NE(addedEntities.find(3), addedEntities.end());
  EXPECT_EQ(addedEntities.find(100), addedEntities.end());

  auto removedEntities = event.RemovedEntities();
  EXPECT_EQ(2u, removedEntities.size());
  EXPECT_NE(removedEntities.find(4), removedEntities.end());
  EXPECT_NE(removedEntities.find(5), removedEntities.end());
  EXPECT_EQ(removedEntities.find(6), removedEntities.end());
}

/////////////////////////////////////////////////
TEST(GuiEventsTest, NewRemovedEntities)
{
  events::NewRemovedEntities event({1, 2, 3}, {4, 5});

  EXPECT_LT(QEvent::User, event.type());

  auto addedEntities = event.NewEntities();
  EXPECT_EQ(3u, addedEntities.size());
  EXPECT_NE(addedEntities.find(1), addedEntities.end());
  EXPECT_NE(addedEntities.find(2), addedEntities.end());
  EXPECT_NE(addedEntities.find(3), addedEntities.end());
  EXPECT_EQ(addedEntities.find(100), addedEntities.end());

  auto removedEntities = event.RemovedEntities();
  EXPECT_EQ(2u, removedEntities.size());
  EXPECT_NE(removedEntities.find(4), removedEntities.end());
  EXPECT_NE(removedEntities.find(5), removedEntities.end());
  EXPECT_EQ(removedEntities.find(6), removedEntities.end());
}

/////////////////////////////////////////////////
TEST(GuiEventsTest, ModelEditorAddEntity)
{
  events::ModelEditorAddEntity event("joint_1", "fixed", 1u);

  event.SetData("parent", "link_1");
  event.SetData("child", "link_2");

  EXPECT_LT(QEvent::User, event.type());


  EXPECT_EQ("joint_1", event.Entity());
  EXPECT_EQ("fixed", event.EntityType());
  EXPECT_EQ(1u, event.ParentEntity());

  EXPECT_TRUE(event.HasData("parent"));
  EXPECT_TRUE(event.HasData("child"));
  EXPECT_FALSE(event.HasData("foobar"));

  EXPECT_EQ("link_1", event.Data("parent"));
  EXPECT_EQ("link_2", event.Data("child"));

  auto data = event.Data();
  EXPECT_EQ(2u, data.size());
}

