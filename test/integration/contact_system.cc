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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Name.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"
#include "../helpers/Util.hh"

using namespace gz;
using namespace sim;

class ContactSystemTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
bool validMultipleCollisionContacts(const msgs::Contacts &_contacts)
{
  if (_contacts.contact_size() != 4)
    return false;

  for (const auto &contact : _contacts.contact())
  {
    if (contact.position_size() != 1)
      return false;

    if (std::abs(std::abs(contact.position(0).x()) - 0.25) > 5e-2 ||
        std::abs(std::abs(contact.position(0).y()) - 1.0) > 5e-2 ||
        std::abs(contact.position(0).z() - 1.0) > 5e-2)
    {
      return false;
    }

    const bool entityNameFound =
      contact.collision1().name() ==
      "contact_model::link::collision_sphere1" ||
      contact.collision1().name() ==
      "contact_model::link::collision_sphere2";
    if (!entityNameFound)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
bool contactsFromEcm(const EntityComponentManager &_ecm,
    msgs::Contacts &_contacts)
{
  _contacts.Clear();

  for (const auto &collisionName :
       {"collision_sphere1", "collision_sphere2"})
  {
    const auto collision = _ecm.EntityByComponents(
        components::Collision(), components::Name(collisionName));
    if (kNullEntity == collision)
      return false;

    const auto contactData =
        _ecm.Component<components::ContactSensorData>(collision);
    if (nullptr == contactData)
      return false;

    for (const auto &contact : contactData->Data().contact())
      *_contacts.add_contact() = contact;
  }

  return true;
}

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
        contact.collision1().name().empty();
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
TEST_F(ContactSystemTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetRestoresEarlyContacts))
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

  msgs::Contacts contacts;
  test::Relay contactDataRecorder;
  contactDataRecorder.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        contactsFromEcm(_ecm, contacts);
      });
  server.AddSystem(contactDataRecorder.systemPtr);

  auto waitForContacts =
      [&server, &contacts]()
      {
        return test::StepUntil(server, 2000, [&]
        {
          return validMultipleCollisionContacts(contacts);
        });
      };

  ASSERT_TRUE(waitForContacts());

  server.ResetAll();

  contacts.Clear();
  ASSERT_TRUE(waitForContacts());
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

/////////////////////////////////////////////////
/// \brief Contact messages of the sensors of contact_update_rate.sdf, by topic
class UpdateRateMessages
{
  /// \brief Subscribe to the topics of the three sensors
  public: UpdateRateMessages()
  {
    for (const auto &it : this->periodsMs)
    {
      const std::string topic = it.first;
      this->msgs[topic];
      auto cb = std::function<void(const msgs::Contacts &)>(
          [this, topic](const msgs::Contacts &_msg)
          {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->msgs[topic].push_back(_msg);
          });
      this->node.Subscribe(topic, cb);
    }
  }

  /// \brief Simulation time of a message, in ms
  public: static int64_t StampMs(const msgs::Contacts &_msg)
  {
    return _msg.header().stamp().sec() * 1000 +
        _msg.header().stamp().nsec() / 1000000;
  }

  /// \brief Wait until every topic has its last message up to _ms, the one
  /// stamped within a period of _ms. Messages arrive asynchronously, in
  /// order for each topic.
  public: void WaitFor(int64_t _ms)
  {
    for (int i = 0; i < 100; ++i)
    {
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (std::all_of(this->msgs.begin(), this->msgs.end(),
            [&](const auto &_it)
            {
              return !_it.second.empty() &&
                  StampMs(_it.second.back()) >
                      _ms - this->periodsMs.at(_it.first);
            }))
        {
          return;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  /// \brief Number of messages of a topic stamped in (_fromMs, _toMs],
  /// counting only those after the last jump back in time (a reset)
  public: int64_t CountBetween(const std::string &_topic, int64_t _fromMs,
                               int64_t _toMs)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    const auto &topicMsgs = this->msgs[_topic];
    auto start = topicMsgs.begin();
    for (auto it = topicMsgs.begin(); it != topicMsgs.end(); ++it)
    {
      if (it != topicMsgs.begin() && StampMs(*it) < StampMs(*(it - 1)))
        start = it;
    }
    return std::count_if(start, topicMsgs.end(),
        [&](const msgs::Contacts &_msg)
        {
          return StampMs(_msg) > _fromMs && StampMs(_msg) <= _toMs;
        });
  }

  /// \brief Number of contacts in the last message of a topic
  public: int LastContactSize(const std::string &_topic)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    const auto &topicMsgs = this->msgs[_topic];
    return topicMsgs.empty() ? 0 : topicMsgs.back().contact_size();
  }

  /// \brief Topics, with their publication period in ms rounded up to a
  /// whole physics step (1 ms)
  private: const std::map<std::string, int64_t> periodsMs{
      {"/contacts/every_iteration", 1}, {"/contacts/hundred_hz", 10},
      {"/contacts/thirty_hz", 34}};

  /// \brief Protects msgs
  private: std::mutex mutex;

  /// \brief Messages received, by topic
  private: std::map<std::string, std::vector<msgs::Contacts>> msgs;

  /// \brief Transport node of the subscriptions
  private: transport::Node node;
};

/////////////////////////////////////////////////
// This test verifies that a contact sensor with an update rate publishes at
// that rate on average, with the contacts gathered since its last
// publication, also when the period is not a multiple of the physics step.
TEST_F(ContactSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(UpdateRate))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact_update_rate.sdf");
  Server server(serverConfig);
  UpdateRateMessages messages;

  // Let the sphere settle on the ground (0.1 s), then count over 1 s
  server.Run(true, 1100, false);
  messages.WaitFor(1100);

  EXPECT_EQ(1000,
      messages.CountBetween("/contacts/every_iteration", 100, 1100));
  EXPECT_EQ(100, messages.CountBetween("/contacts/hundred_hz", 100, 1100));
  EXPECT_EQ(30, messages.CountBetween("/contacts/thirty_hz", 100, 1100));

  // All sensors watch the same collision: a 100 Hz message carries the
  // contacts of the 10 iterations since the last one
  const int perIteration =
      messages.LastContactSize("/contacts/every_iteration");
  EXPECT_GT(perIteration, 0);
  EXPECT_EQ(10 * perIteration,
      messages.LastContactSize("/contacts/hundred_hz"));
}

/////////////////////////////////////////////////
// This test verifies that the update rates still hold after a world reset,
// which takes the simulation time back to zero and restarts the schedules.
TEST_F(ContactSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(UpdateRateReset))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/contact_update_rate.sdf");
  Server server(serverConfig);
  UpdateRateMessages messages;

  server.Run(true, 600, false);
  messages.WaitFor(600);

  // Count over 0.9 s after the reset: a few iterations go to the reset itself
  server.ResetAll();
  server.Run(true, 1100, false);
  messages.WaitFor(1000);

  EXPECT_EQ(900,
      messages.CountBetween("/contacts/every_iteration", 100, 1000));
  EXPECT_EQ(90, messages.CountBetween("/contacts/hundred_hz", 100, 1000));
  EXPECT_EQ(27, messages.CountBetween("/contacts/thirty_hz", 100, 1000));
}
