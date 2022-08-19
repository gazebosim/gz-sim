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
  gz::sim::Model modelNull;
  EXPECT_EQ(gz::sim::kNullEntity, modelNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  EXPECT_EQ(id, model.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Model modelCopy(model); // NOLINT
  EXPECT_EQ(model.Entity(), modelCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelCopy;
  modelCopy = model;
  EXPECT_EQ(model.Entity(), modelCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelMoved(std::move(model));
  EXPECT_EQ(id, modelMoved.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelMoved;
  modelMoved = std::move(model);
  EXPECT_EQ(id, modelMoved.Entity());
}
