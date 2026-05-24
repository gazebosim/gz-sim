/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>

#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "../../../../test/helpers/Util.hh"

#include "VisualizeContacts.hh"

using namespace gz;

/// \brief Tests for the visualize contacts GUI plugin.
class VisualizeContactsTest : public ::testing::Test
{
};

/////////////////////////////////////////////////
TEST_F(VisualizeContactsTest, RequestsContactDataForNewCollisions)
{
  sim::EntityComponentManager ecm;

  auto worldEntity = ecm.CreateEntity();
  ecm.CreateComponent(worldEntity, sim::components::World());
  ecm.CreateComponent(worldEntity, sim::components::Name("test"));

  auto firstCollision = ecm.CreateEntity();
  ecm.CreateComponent(firstCollision, sim::components::Collision());

  std::mutex mutex;
  std::vector<sim::Entity> requestedCollisions;
  std::function<bool(const msgs::Entity &, msgs::Boolean &)> enableCollisionCb =
    [&mutex, &requestedCollisions](const msgs::Entity &_req,
                                   msgs::Boolean &_res) -> bool
    {
      std::lock_guard<std::mutex> lock(mutex);
      requestedCollisions.push_back(_req.id());
      _res.set_data(true);
      return true;
    };

  transport::Node node;
  const std::string service = "/world/test/enable_collision";
  ASSERT_TRUE(node.Advertise(service, enableCollisionCb));
  ASSERT_TRUE(sim::test::waitForService(node, service, 1));

  sim::VisualizeContacts plugin;
  sim::UpdateInfo info;
  plugin.Update(info, ecm);

  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(1u, requestedCollisions.size());
    EXPECT_EQ(firstCollision, requestedCollisions[0]);
  }

  auto secondCollision = ecm.CreateEntity();
  ecm.CreateComponent(secondCollision, sim::components::Collision());

  plugin.Update(info, ecm);

  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(2u, requestedCollisions.size());
    EXPECT_EQ(firstCollision, requestedCollisions[0]);
    EXPECT_EQ(secondCollision, requestedCollisions[1]);
  }
}
