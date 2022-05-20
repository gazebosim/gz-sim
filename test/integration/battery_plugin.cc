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

#include <string>

#include <gz/common/Battery.hh>
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/Filesystem.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/test_config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class BatteryPluginTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    auto plugin = sm.LoadPlugin("libMockSystem.so",
                                "gz::sim::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<gazebo::MockSystem *>(
        systemPtr->QueryInterface<gazebo::System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: gazebo::MockSystem *mockSystem;

  private: gazebo::SystemLoader sm;
};


/////////////////////////////////////////////////
// Single model consuming single batter
// See https://github.com/ignitionrobotics/ign-gazebo/issues/1175
TEST_F(BatteryPluginTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(SingleBattery))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  gazebo::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;

      // Check a battery exists
      EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

      // Find the battery entity
      Entity batEntity = ecm->EntityByComponents(components::Name(
        "linear_battery"));
      EXPECT_NE(kNullEntity, batEntity);

      // Find the battery component
      EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
        components::BatterySoC::typeId));
      auto batComp = ecm->Component<components::BatterySoC>(batEntity);

      // Check voltage is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/ignitionrobotics/ign-gazebo/issues/55
      EXPECT_GT(batComp->Data(), 0);
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

  // Find the battery entity
  Entity batEntity = ecm->EntityByComponents(components::Name(
    "linear_battery"));
  EXPECT_NE(kNullEntity, batEntity);

  // Find the battery component
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::BatterySoC::typeId));
  auto batComp = ecm->Component<components::BatterySoC>(batEntity);

  // Check voltage after consumption is lower than initial voltage
  EXPECT_LT(batComp->Data(), 12.592);

  // Check there is a single battery matching exactly the one specified
  int linearBatCount = 0;
  int totalBatCount = 0;
  ecm->Each<components::BatterySoC, components::Name>(
      [&](const Entity &_batEntity, components::BatterySoC *_batComp,
          components::Name *_nameComp) -> bool
      {
        totalBatCount++;
        if (_nameComp->Data() == "linear_battery")
        {
          linearBatCount++;

          EXPECT_NE(kNullEntity, _batEntity);
          EXPECT_EQ(_nameComp->Data(), "linear_battery");

          // Check battery component voltage data is lower than initial voltage
          EXPECT_LT(_batComp->Data(), 12.592);
        }

        return true;
      });
  EXPECT_EQ(linearBatCount, 1);
  EXPECT_EQ(totalBatCount, 2);
}

/////////////////////////////////////////////////
// Battery with  power draining topics
// See https://github.com/ignitionrobotics/ign-gazebo/issues/1175
TEST_F(BatteryPluginTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(PowerDrainTopic))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  gazebo::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;

      // Check a battery exists
      EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

      // Find the battery entity
      Entity batEntity = ecm->EntityByComponents(components::Name(
        "linear_battery_topics"));
      EXPECT_NE(kNullEntity, batEntity);

      // Find the battery component
      EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
        components::BatterySoC::typeId));
      auto batComp = ecm->Component<components::BatterySoC>(batEntity);

      // Check voltage is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/ignitionrobotics/ign-gazebo/issues/55
      EXPECT_GT(batComp->Data(), 0);
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

  // Find the battery entity
  Entity batEntity = ecm->EntityByComponents(components::Name(
    "linear_battery_topics"));
  EXPECT_NE(kNullEntity, batEntity);

  // Find the battery component
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::BatterySoC::typeId));
  auto batComp = ecm->Component<components::BatterySoC>(batEntity);

  // Check state of charge should be 1, since the batery has not drained
  // and the <initial_charge> is equivalent ot the <capacity>.
  EXPECT_DOUBLE_EQ(batComp->Data(), 1.0);

  // Send a message on one of the <power_draining_topic> topics, which will
  // start the battery draining when the server starts again.
  gz::transport::Node node;
  auto pub = node.Advertise<msgs::StringMsg>("/battery/discharge2");
  msgs::StringMsg msg;
  pub.Publish(msg);

  // Run the server again.
  server.Run(true, 100, false);

  // The state of charge should be <1, since the batery has started
  // draining.
  EXPECT_LT(batComp->Data(), 1.0);
}
