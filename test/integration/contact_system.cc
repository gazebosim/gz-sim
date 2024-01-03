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

#include <gz/msgs/contacts.pb.h>

#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class ContactSystemTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// This test verifies that colliding entity names are populated in
// the contact points message.
TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(EnableCollidingEntityNames))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact_with_entity_names.sdf";
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

    for (const auto &contact : lastContacts.contact())
    {
      ASSERT_EQ(1, contact.position_size());
      bool entityNameFound =
        contact.collision1().name() ==
        "contact_model::link::collision_sphere1" ||
        contact.collision1().name() ==
        "contact_model::link::collision_sphere2";
      EXPECT_TRUE(entityNameFound);
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

/////////////////////////////////////////////////
// This test verifies that colliding entity names are not populated in
// the contact points message.
TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(DisableCollidingEntityNames))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact_without_entity_names.sdf";
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

    for (const auto &contact : lastContacts.contact())
    {
      ASSERT_EQ(1, contact.position_size());
      bool entityNameEmpty =
        contact.collision1().name() == "";
      EXPECT_TRUE(entityNameEmpty);
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

/////////////////////////////////////////////////
// The test checks that contacts are published by the contact system
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(MultipleCollisionsAsContactSensors))
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
      bool entityNameFound =
        contact.collision1().name() ==
        "contact_model::link::collision_sphere1" ||
        contact.collision1().name() ==
        "contact_model::link::collision_sphere2";
      EXPECT_TRUE(entityNameFound);
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

/////////////////////////////////////////////////
// The test checks that contacts are published with
// the correct extraContactData
TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ExtraContactData))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact_extra_data.sdf";
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
  node.Subscribe("/test_extra_collision_data", callbackFunc);

  // Run server for 10 iters to ensure
  // contact solver has converged
  size_t iters = 10;
  server.Run(true, iters, false);
  {
    std::lock_guard<std::mutex> lock(contactMutex);
    ASSERT_GE(contactMsgs.size(), 1u);
  }

  {
    std::lock_guard<std::mutex> lock(contactMutex);
    const auto &lastContacts = contactMsgs.back();
    EXPECT_EQ(1, lastContacts.contact_size());

    // The sphere weighs 1kg and gravity is set to 9.81 m/s^2
    // Hence the contact force should be m*g = 9.81 N
    // along the z-axis. The force on body 2 should be equal
    // and opposite.
    // All torques should be zero.
    // The normal should align with the world z-axis
    for (const auto &contact : lastContacts.contact())
    {
      ASSERT_EQ(1, contact.wrench_size());
      ASSERT_EQ(1, contact.normal_size());

      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_1_wrench().force().x(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_1_wrench().force().y(),
                  5e-2);
      EXPECT_NEAR(9.81, contact.wrench().Get(0).body_1_wrench().force().z(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_1_wrench().torque().x(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_1_wrench().torque().y(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_1_wrench().torque().z(),
                  5e-2);

      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_2_wrench().force().x(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_2_wrench().force().y(),
                  5e-2);
      EXPECT_NEAR(-9.81, contact.wrench().Get(0).body_2_wrench().force().z(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_2_wrench().torque().x(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_2_wrench().torque().y(),
                  5e-2);
      EXPECT_NEAR(0.0, contact.wrench().Get(0).body_2_wrench().torque().z(),
                  5e-2);

      EXPECT_NEAR(0.0, contact.normal().Get(0).x(), 5e-2);
      EXPECT_NEAR(0.0, contact.normal().Get(0).y(), 5e-2);
      EXPECT_NEAR(1.0, contact.normal().Get(0).z(), 5e-2);

    }
  }
}

TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(RemoveContactSensor))
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
