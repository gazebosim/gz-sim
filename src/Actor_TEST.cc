/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <cstddef>

#include "gz/sim/Actor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

/////////////////////////////////////////////////
TEST(ActorTest, Constructor)
{
  gz::sim::Actor actorNull;
  EXPECT_EQ(gz::sim::kNullEntity, actorNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Actor actor(id);

  EXPECT_EQ(id, actor.Entity());
}

/////////////////////////////////////////////////
TEST(ActorTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Actor actor(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Actor actorCopy(actor); // NOLINT
  EXPECT_EQ(actor.Entity(), actorCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ActorTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Actor actor(id);

  gz::sim::Actor actorCopy;
  actorCopy = actor;
  EXPECT_EQ(actor.Entity(), actorCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ActorTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Actor actor(id);

  gz::sim::Actor actorMoved(std::move(actor));
  EXPECT_EQ(id, actorMoved.Entity());
}

/////////////////////////////////////////////////
TEST(ActorTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Actor actor(id);

  gz::sim::Actor actorMoved;
  actorMoved = std::move(actor);
  EXPECT_EQ(id, actorMoved.Entity());
}
