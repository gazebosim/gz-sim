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

#include "ignition/gazebo/Entity.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Entity, Constructor)
{
  gazebo::Entity entity, entity2;
  EXPECT_EQ(gazebo::kNullEntity, entity.Id());
  EXPECT_TRUE(entity == entity2);

  gazebo::Entity *entity3 = new gazebo::Entity(std::move(entity));
  EXPECT_TRUE(entity2 == *entity3);
  delete entity3;
  entity3 = nullptr;

  gazebo::Entity entity5;
  entity5 = std::move(entity2);
  EXPECT_TRUE(entity5 == entity);
}
