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

#include <ignition/msgs/contacts.pb.h>

#include <thread>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class ContactSystemTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// The test checks that contacts are published by the contact system
TEST_F(ContactSystemTest, MultipleCollisionsAsContactSensors)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::mutex contactMutex;
  std::vector<msgs::Contacts> contactMsgs;

  auto contactCb = [&](const msgs::Contacts &_msg) -> void
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    contactMsgs.push_back(_msg);
  };

  // subscribe to contacts topic
  transport::Node node;
  // Have to create an lvalue here for Node::Subscribe to work.
  auto callbackFunc = std::function<void(const msgs::Contacts &)>(contactCb);
  node.Subscribe("/test_multiple_collisions", callbackFunc);

  // Run server
  size_t iters = 1000;
  server.Run(true, iters, false);
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    ASSERT_GE(contactMsgs.size(), 1u);
  }

  // "contact_model" has one contact sensor, but the sensor uses two collisions
  // as sensors. Therefore, once the "contact_model" model has fallen and has
  // stabilized, we expect that each collision will have 1 contact point.
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    const auto &lastContacts = contactMsgs.back();
    EXPECT_EQ(4, lastContacts.contact_size());

    // It is easy to determine the contact points because the sphere falls and
    // rests on the edges of the boxes. The position of the boxes is symmetric
    // about the centers of the spheres, thus, the points of contact in each
    // axis only differ by their signs
    for (const auto &contact : lastContacts.contact())
    {
      ASSERT_EQ(1, contact.position_size());
      EXPECT_NEAR(0.25, std::abs(contact.position(0).x()), 5e-2);
      EXPECT_NEAR(1, std::abs(contact.position(0).y()), 5e-2);
      EXPECT_NEAR(1, contact.position(0).z(), 5e-2);
    }
  }

  // Remove the colliding boxes and check that contacts are no longer generated.
  server.RequestRemoveEntity("box1");
  server.RequestRemoveEntity("box2");
  // Run once to remove entities
  server.Run(true, 1, false);

  contactMsgs.clear();
  server.Run(true, 10, false);
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    EXPECT_EQ(0u, contactMsgs.size());
  }
}

TEST_F(ContactSystemTest, RemoveContactSensor)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  std::mutex contactMutex;
  std::vector<msgs::Contacts> contactMsgs;

  auto contactCb = [&](const msgs::Contacts &_msg) -> void
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    contactMsgs.push_back(_msg);
  };

  // subscribe to contacts topic
  transport::Node node;
  // Have to create an lvalue here for Node::Subscribe to work.
  auto callbackFunc = std::function<void(const msgs::Contacts &)>(contactCb);
  node.Subscribe("/test_multiple_collisions", callbackFunc);

  // Run server for a few iterations before removing sensor
  server.Run(true, 1000, false);
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    EXPECT_GE(contactMsgs.size(), 1u);
  }

  // Remove the contact sensor and check that contacts are no longer generated.
  server.RequestRemoveEntity("sensor_contact");
  // Run once to remove entities
  server.Run(true, 1, false);

  auto sensorEntity = server.EntityByName("sensor_contact");
  EXPECT_EQ(std::nullopt, sensorEntity);

  contactMsgs.clear();
  server.Run(true, 10, false);
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    EXPECT_EQ(0u, contactMsgs.size());
  }
}
