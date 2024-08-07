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

#include "gz/sim/Link.hh"

/////////////////////////////////////////////////
TEST(LinkTest, Constructor)
{
  gz::sim::Link linkNull;
  EXPECT_EQ(gz::sim::kNullEntity, linkNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  EXPECT_EQ(id, link.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Link linkCopy(link); // NOLINT
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkCopy;
  linkCopy = link;
  EXPECT_EQ(link.Entity(), linkCopy.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkMoved(std::move(link));
  EXPECT_EQ(id, linkMoved.Entity());
}

/////////////////////////////////////////////////
TEST(LinkTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Link link(id);

  gz::sim::Link linkMoved;
  linkMoved = std::move(link);
  EXPECT_EQ(id, linkMoved.Entity());
}
