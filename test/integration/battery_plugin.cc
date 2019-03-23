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

#include <ignition/common/Battery.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/components/Battery.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

class BatteryPluginTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);

    auto plugin = sm.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<gazebo::MockSystem *>(
        systemPtr->QueryInterface<gazebo::System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: ignition::gazebo::SystemPluginPtr systemPtr;
  public: gazebo::MockSystem *mockSystem;

  private: gazebo::SystemLoader sm;
};

/////////////////////////////////////////////////
// Single link consuming single battery
TEST_F(BatteryPluginTest, SingleLinkSingleBattery)
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_slsb.sdf");
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
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check the link exists
  Entity linkEntity = ecm->EntityByComponents(components::Link(),
    components::Name("body"));
  EXPECT_NE(kNullEntity, linkEntity);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::Battery::typeId));

  // Find the battery entity
  Entity batEntity = ecm->EntityByComponents(components::Name(
    "linear_battery"));
  EXPECT_NE(kNullEntity, batEntity);

  // Find the battery component
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::Battery::typeId));
  auto batComp = ecm->Component<components::Battery>(batEntity);
  EXPECT_NE(nullptr, batComp->Data());

  // Check battery initial parameters
  EXPECT_EQ(batComp->Data()->Name(), "linear_battery");
  EXPECT_NEAR(batComp->Data()->InitVoltage(), 12.592, 1e-6);

  // Check there is a single consumer
  EXPECT_EQ(batComp->Data()->PowerLoads().size(), 1lu);

  // Check voltage after consumption is lower than initial voltage
  EXPECT_LT(batComp->Data()->Voltage(), 12.592);

  // Check there is a single battery matching exactly the one specified
  int batCount = 0;
  ecm->Each<components::Battery, components::Name>(
      [&](const Entity &_batEntity, components::Battery *_batComp,
          components::Name *_nameComp) -> bool
      {
        batCount++;

        EXPECT_NE(kNullEntity, _batEntity);
        EXPECT_EQ(_nameComp->Data(), "linear_battery");

        // Check battery initial parameters
        EXPECT_NE(nullptr, _batComp->Data());
        EXPECT_EQ(_batComp->Data()->Name(), "linear_battery");
        EXPECT_NEAR(_batComp->Data()->InitVoltage(), 12.592, 1e-6);

        return true;
      });
  EXPECT_EQ(batCount, 1);
}

/////////////////////////////////////////////////
// Multiple links consuming the same battery
TEST_F(BatteryPluginTest, MultipleLinksSingleBattery)
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_mlsb.sdf");
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
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check the links exist
  Entity linkEntity1 = ecm->EntityByComponents(components::Link(),
    components::Name("lower_link"));
  EXPECT_NE(kNullEntity, linkEntity1);

  Entity linkEntity2 = ecm->EntityByComponents(components::Link(),
    components::Name("base"));
  EXPECT_NE(kNullEntity, linkEntity2);

  // Find the battery entity
  Entity batEntity = ecm->EntityByComponents(components::Name(
    "linear_battery"));
  EXPECT_NE(kNullEntity, batEntity);

  // Find battery component
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::Battery::typeId));
  auto batComp = ecm->Component<components::Battery>(batEntity);
  EXPECT_NE(nullptr, batComp->Data());

  // Check battery initial parameters
  EXPECT_EQ(batComp->Data()->Name(), "linear_battery");
  EXPECT_NEAR(batComp->Data()->InitVoltage(), 12.592, 1e-6);

  // Check there are two consumers
  EXPECT_EQ(batComp->Data()->PowerLoads().size(), 2lu);

  // Check voltage after consumption is lower than initial voltage
  EXPECT_LT(batComp->Data()->Voltage(), 12.592);

  // Check there are two battery entities, which share the same single battery
  int batCount = 0;
  ecm->Each<components::Battery, components::Name>(
      [&](const Entity &_batEntity, components::Battery *_batComp,
          components::Name *_nameComp) -> bool
      {
        batCount++;

        EXPECT_NE(kNullEntity, _batEntity);
        EXPECT_EQ(_nameComp->Data(), "linear_battery");

        // Check battery initial parameters
        EXPECT_NE(nullptr, _batComp->Data());
        EXPECT_EQ(_batComp->Data()->Name(), "linear_battery");
        EXPECT_NEAR(_batComp->Data()->InitVoltage(), 12.592, 1e-6);

        return true;
      });
  EXPECT_EQ(batCount, 2);
}

/////////////////////////////////////////////////
// Single link consuming multiple batteries
TEST_F(BatteryPluginTest, SingleLinkMultipleBatteries)
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_slmb.sdf");
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
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check the link exists
  Entity linkEntity = ecm->EntityByComponents(components::Link(),
    components::Name("body"));
  EXPECT_NE(kNullEntity, linkEntity);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::Battery::typeId));

  std::string batNames[2] = {"linear_battery1", "linear_battery2"};
  double batVolts[2] = {12.592, 6.0};
  for (int i = 0; i < 2; ++i)
  {
    // Find battery entities
    Entity batEntity = ecm->EntityByComponents(components::Name(
      batNames[i]));
    EXPECT_NE(kNullEntity, batEntity);
 
    // Find battery components
    EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
      components::Battery::typeId));
    auto batComp = ecm->Component<components::Battery>(batEntity);
    EXPECT_NE(nullptr, batComp->Data());
 
    // Check battery initial parameters
    EXPECT_EQ(batComp->Data()->Name(), batNames[i]);
    EXPECT_NEAR(batComp->Data()->InitVoltage(), batVolts[i], 1e-6);
 
    // Check there is a single consumer
    EXPECT_EQ(batComp->Data()->PowerLoads().size(), 1lu);
  }

  // Check there are two battery entities
  int batCount = 0;
  ecm->Each<components::Battery>(
      [&](const Entity & /*_batEntity*/, components::Battery * /*_batComp*/)
        -> bool
      {
        batCount++;
        return true;
      });
  EXPECT_EQ(batCount, 2);
}

/////////////////////////////////////////////////
// Multiple links, each consuming a separate battery
TEST_F(BatteryPluginTest, MultipleLinksMultipleBatteries)
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_mlmb.sdf");
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
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check the links exist
  Entity linkEntity1 = ecm->EntityByComponents(components::Link(),
    components::Name("lower_link"));
  EXPECT_NE(kNullEntity, linkEntity1);

  Entity linkEntity2 = ecm->EntityByComponents(components::Link(),
    components::Name("base"));
  EXPECT_NE(kNullEntity, linkEntity2);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::Battery::typeId));

  std::string batNames[2] = {"linear_battery1", "linear_battery2"};
  double batVolts[2] = {12.592, 6.0};
  long unsigned int nConsumers[2] = {1, 2};
  for (int i = 0; i < 2; ++i)
  {
    // Find battery entities
    Entity batEntity = ecm->EntityByComponents(components::Name(
      batNames[i]));
    EXPECT_NE(kNullEntity, batEntity);
 
    // Find battery components
    EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
      components::Battery::typeId));
    auto batComp = ecm->Component<components::Battery>(batEntity);
    EXPECT_NE(nullptr, batComp->Data());
 
    // Check battery initial parameters
    EXPECT_EQ(batComp->Data()->Name(), batNames[i]);
    EXPECT_NEAR(batComp->Data()->InitVoltage(), batVolts[i], 1e-6);
 
    // Check there is a single consumer
    EXPECT_EQ(batComp->Data()->PowerLoads().size(), nConsumers[i]);
  }

  // Check there are three battery entities, two of which share the same
  //   battery (two links consuming the same battery)
  size_t batCount = 0;
  std::vector<common::BatteryPtr> batteries;
  ecm->Each<components::Battery>(
      [&](const Entity & /*_batEntity*/, components::Battery *_batComp)
        -> bool
      {
        // Increment on the entity
        batCount++;
 
        // Look for a battery the same as this one
        bool newBat = true;
        for (auto it = batteries.begin(); it != batteries.end(); ++it)
        {
          if (*it == _batComp->Data())
          {
            newBat = false;
            break;
          }
        }
        if (newBat)
          batteries.push_back (_batComp->Data());
        return true;
      });
  // Three battery entities
  EXPECT_EQ(batCount, 3lu);
  // Two actual common::Battery instances
  EXPECT_EQ(batteries.size(), 2lu);
}
