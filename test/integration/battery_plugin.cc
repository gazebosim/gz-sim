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
#include "ignition/gazebo/components/BatterySoC.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
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
// Single model consuming single battery
TEST_F(BatteryPluginTest, SingleBattery)
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
  int batCount = 0;
  ecm->Each<components::BatterySoC, components::Name>(
      [&](const Entity &_batEntity, components::BatterySoC *_batComp,
          components::Name *_nameComp) -> bool
      {
        batCount++;

        EXPECT_NE(kNullEntity, _batEntity);
        EXPECT_EQ(_nameComp->Data(), "linear_battery");

        // Check battery component voltage data is lower than initial voltage
        EXPECT_LT(_batComp->Data(), 12.592);

        return true;
      });
  EXPECT_EQ(batCount, 1);
}
