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
  ServerConfig serverConfig;

  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_slsb.sdf");

  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  // Pass changed SDF to server
  serverConfig.SetSdfFile(sdfPath);

  // Start server
  Server server(serverConfig);
  // TODO(mabelzhang) This seg faults, unless comment out battery plugin in SDF
  server.Run(true, 100, false);

  // TODO(mabelzhang) Check battery voltage < sdf::Battery::Voltage() init voltage,
  //   and components::Battery::Data()->PowerLoads.size() == 1
}

/////////////////////////////////////////////////
// Multiple links consuming the same battery
TEST_F(BatteryPluginTest, MultipleLinksSingleBattery)
{
  ServerConfig serverConfig;

  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_mlsb.sdf");

  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  // Pass changed SDF to server
  serverConfig.SetSdfFile(sdfPath);

  Server server(serverConfig);

  // A pointer to the ecm. This will be valid once we run the mock system
  gazebo::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const gazebo::UpdateInfo &, gazebo::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    };

  server.AddSystem(this->systemPtr);

  server.Run(true, 10, false);
  EXPECT_NE(nullptr, ecm);


  Entity linkEntity = ecm->EntityByComponents(components::Link(),
    components::Name("lower_link"));
  EXPECT_NE(kNullEntity, linkEntity);
  std::cerr << "PASSED: linkEntity != kNullEntity\n";

  EXPECT_TRUE(ecm->HasComponentType(components::Battery::typeId));
  std::cerr << "PASSED: HasComponentType(components::Battery::typeId)\n";


  // Verify voltages
  Entity batEntity = ecm->EntityByComponents(components::Battery(),
    components::Name("linear_battery"));
  // TODO(mabelzhang) this test fails
  EXPECT_NE(kNullEntity, batEntity);
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity, components::Battery::typeId));

  //auto batComp = ecm->Component<components::Battery>(batEntity);
  //EXPECT_NE(nullptr, batComp->Data());
  //EXPECT_GT(batComp->Data()->PowerLoads().size(), 1lu);

  /*
  ecm->Each<components::Battery, components::Name>(
      [&](const Entity &_batEntity, components::Battery *_batComp,
          components::Name *_nameComp) -> bool
      {
        //EXPECT_NE(kNullEntity, _batEntity);
        //EXPECT_NE(nullptr, _batComp->Data());
        //EXPECT_EQ(_nameComp->Data(), "linear_battery");

        //EXPECT_GT(_batComp->Data()->PowerLoads().size(), 1lu);

        return true;
      });
  */

}

/*
/////////////////////////////////////////////////
// Single link consuming multiple batteries
TEST_F(BatteryPluginTest, SingleLinkMultipleBatteries)
{
  //const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
  //  "test", "worlds", "battery_slmb.sdf");
}

/////////////////////////////////////////////////
// Multiple links, each consuming a separate battery
TEST_F(BatteryPluginTest, MultipleLinksMultipleBatteries)
{
  //const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
  //  "test", "worlds", "battery_mlmb.sdf");
}
*/


