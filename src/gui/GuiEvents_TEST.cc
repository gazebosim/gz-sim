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

#include "test_config.hh"
#include "gz/sim/gui/GuiEvents.hh"

using namespace gz;
using namespace sim;
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
