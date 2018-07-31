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

#include "ignition/gazebo/EntityQuery.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(EntityQuery, Constructor)
{
  gazebo::EntityQuery query;
  EXPECT_TRUE(query.Empty());
  EXPECT_TRUE(query.Entities().empty());

  query.AddEntity(2);
  const std::set<gazebo::EntityId> &ids = query.Entities();
  EXPECT_EQ(1u, ids.size());
  EXPECT_EQ(2, *ids.find(2));

  query.RemoveEntity(2);
  EXPECT_TRUE(query.Entities().empty());

  query.AddEntity(3);
  query.AddEntity(4);
  EXPECT_EQ(2u, query.Entities().size());
  query.Clear();
  EXPECT_TRUE(query.Empty());
}

/////////////////////////////////////////////////
TEST(EntityQuery, Components)
{
  gazebo::EntityQuery query;
  gazebo::EntityQuery query2;
  EXPECT_EQ(query, query2);

  EXPECT_FALSE(query.AddComponentType(gazebo::kComponentTypeIdInvalid));
  EXPECT_TRUE(query.AddComponentType(1));
  EXPECT_NE(query, query2);

  EXPECT_TRUE(query2.AddComponentType(1));
  EXPECT_EQ(query, query2);

  gazebo::EntityQuery query3;
  query3 = query;
  EXPECT_EQ(query, query3);

  gazebo::EntityQuery query4;
  query4 = std::move(query3);
  EXPECT_EQ(query, query4);
}
