/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define tol 10e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test Thermal system
class ThermalTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                 nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};


/////////////////////////////////////////////////
TEST_F(ThermalTest, TemperatureComponent)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/thermal.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that checks for thermal component
  Relay testSystem;

  std::map<std::string, math::Temperature> entityTemp;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Temperature, components::Name>(
          [&](const ignition::gazebo::Entity &_id,
              const components::Temperature *_temp,
              const components::Name *_name) -> bool
          {
            // store temperature data
            entityTemp[_name->Data()] = _temp->Data();

            // verify temperature data belongs to a visual
            EXPECT_NE(nullptr, _ecm.Component<components::Visual>(_id));

            return true;
          });
    });
  server.AddSystem(testSystem.systemPtr);

  // verify nothing in map at beginning
  EXPECT_TRUE(entityTemp.empty());

  // Run server
  server.Run(true, 1, false);

  // verify temperature components are created and the values are correct
  EXPECT_EQ(3u, entityTemp.size());
  EXPECT_DOUBLE_EQ(200.0, entityTemp["box_visual"].Kelvin());
  EXPECT_DOUBLE_EQ(600.0, entityTemp["sphere_visual"].Kelvin());
  EXPECT_DOUBLE_EQ(400.0, entityTemp["cylinder_visual"].Kelvin());
}
