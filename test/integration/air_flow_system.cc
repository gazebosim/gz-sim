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

#include <gz/msgs/air_flow_sensor.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/AirFlowSensor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Server.hh"
#include "test_config.hh"
#include "gz/sim/TestFixture.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test AirFlowTest system
class AirFlowTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(AirFlowTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AirFlow))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "air_flow.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "air_flow_sensor";

  auto topic = "world/air_flow_sensor/model/air_flow_model/link/link/"
      "sensor/air_flow_sensor/air_flow";

  bool updateChecked{false};

  // Create a system that checks sensor topic
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::AirFlowSensor, components::Name>(
            [&](const Entity &_entity,
                const components::AirFlowSensor *,
                const components::Name *_name) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              updateChecked = true;

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Subscribe to air_flow topic
  bool received{false};
  msgs::AirFlowSensor msg;
  msg.Clear();
  std::function<void(const msgs::AirFlowSensor &)>  cb =
      [&received, &msg](const msgs::AirFlowSensor &_msg)
  {
    // Only need one message
    if (received)
      return;

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(topic, cb);

  // Run server
  server.Run(true, 100, false);
  EXPECT_TRUE(updateChecked);

  // Wait for message to be received
  for (int sleep = 0; !received && sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(received);

  // check air flow
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_EQ(0, msg.header().stamp().sec());
  EXPECT_LT(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(-M_PI_2, msg.xy_direction());
  EXPECT_DOUBLE_EQ(0, msg.xy_speed());
}

/////////////////////////////////////////////////
// The test checks if the sensor is capable of measuring
// the wind conditions when it is stationary
TEST_F(AirFlowTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AirFlowWindy))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "air_flow_windy.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "air_flow_sensor";

  auto topic = "world/air_flow_sensor/model/air_flow_model/link/link/"
      "sensor/air_flow_sensor/air_flow";

  bool updateChecked{false};

  // Create a system that checks sensor topic
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::AirFlowSensor, components::Name>(
            [&](const Entity &_entity,
                const components::AirFlowSensor *,
                const components::Name *_name) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              updateChecked = true;

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Subscribe to air_flow topic
  bool received{false};
  msgs::AirFlowSensor msg;
  msg.Clear();
  std::function<void(const msgs::AirFlowSensor &)>  cb =
      [&received, &msg](const msgs::AirFlowSensor &_msg)
  {

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(topic, cb);

  // Run server
  server.Run(true, 100, false);
  EXPECT_TRUE(updateChecked);


  // Wait for message to be received
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(received);

  // check air flow
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_EQ(5.0, msg.xy_speed());
  EXPECT_NEAR(0.0, msg.xy_direction(), 1e-3);
}

/////////////////////////////////////////////////
// The test checks that the sensor measures the correct relative airflow
// when it is moving within windy condition.
TEST_F(AirFlowTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(AirFlowMoveWindy))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "air_flow_windy.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "air_flow_sensor";

  auto topic = "world/air_flow_sensor/model/air_flow_model/link/link/"
      "sensor/air_flow_sensor/air_flow";

  bool updateChecked{false};

  // Create a system that checks sensor topic
  TestFixture testSystem(serverConfig);

  Link body;
  Model model;


  testSystem.OnConfigure(
    [&](const Entity &_worldEntity,
      const std::shared_ptr<const sdf::Element> &/*_sdf*/,
      EntityComponentManager &_ecm,
      EventManager &/*eventMgr*/)
    {
        World world(_worldEntity);
        auto modelEntity =  world.ModelByName(_ecm, "air_flow_model");
        EXPECT_NE(modelEntity, kNullEntity);
        model = Model(modelEntity);

        auto bodyEntity = model.LinkByName(_ecm, "link");
        EXPECT_NE(bodyEntity, kNullEntity);

        body = Link(bodyEntity);
        body.EnableVelocityChecks(_ecm);

        body.SetLinearVelocity(_ecm, gz::math::Vector3d(1, 5, 0));

    }).OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {

        _ecm.Each<components::AirFlowSensor, components::Name>(
            [&](const Entity &_entity,
                const components::AirFlowSensor *,
                const components::Name *_name) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              if (_info.iterations == 1)
                return true;

              // This component is created on the 2nd PreUpdate
              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              updateChecked = true;

              return true;
            });
      }).Finalize();

  // Subscribe to air_flow topic
  bool received{false};
  msgs::AirFlowSensor msg;
  msg.Clear();
  std::function<void(const msgs::AirFlowSensor &)>  cb =
      [&received, &msg](const msgs::AirFlowSensor &_msg)
  {

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(topic, cb);


  testSystem.Server()->Run(true, 10, false);
  // Run server
  // server.Run(true, 10, false);
  EXPECT_TRUE(updateChecked);


  // Wait for message to be received
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(received);

  // check air flow
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_NEAR(10.04987562, msg.xy_speed(), 1e-3);
  EXPECT_NEAR(-0.0996686524911, msg.xy_direction(), 1e-2);
}
