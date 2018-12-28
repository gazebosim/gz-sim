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

#include "ignition/gazebo/Model.hh"

/////////////////////////////////////////////////
TEST(ModelTest, Constructor)
{
  ignition::gazebo::Model modelNull;
  EXPECT_EQ(ignition::gazebo::kNullEntity, modelNull.Id());

  ignition::gazebo::EntityId id(3);
  ignition::gazebo::Model model(id);

  EXPECT_EQ(id, model.Id());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyConstructor)
{
  ignition::gazebo::EntityId id(3);
  ignition::gazebo::Model model(id);

  ignition::gazebo::Model modelCopy(model);
  EXPECT_EQ(model.Id(), modelCopy.Id());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyAssignmentOperator)
{
  ignition::gazebo::EntityId id(3);
  ignition::gazebo::Model model(id);

  ignition::gazebo::Model modelCopy;
  modelCopy = model;
  EXPECT_EQ(model.Id(), modelCopy.Id());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveConstructor)
{
  ignition::gazebo::EntityId id(3);
  ignition::gazebo::Model model(id);

  ignition::gazebo::Model modelMoved(std::move(model));
  EXPECT_EQ(id, modelMoved.Id());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveAssignmentOperator)
{
  ignition::gazebo::EntityId id(3);
  ignition::gazebo::Model model(id);

  ignition::gazebo::Model modelMoved;
  modelMoved = std::move(model);
  EXPECT_EQ(id, modelMoved.Id());
}
