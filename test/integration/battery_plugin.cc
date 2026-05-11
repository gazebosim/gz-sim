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

#include <chrono>
#include <string>
#include <utility>

#include <gz/common/Battery.hh>
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/Filesystem.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/BatteryPowerLoad.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Util.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class BatteryPluginTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetName("gz::sim::MockSystem");
    sdfPlugin.SetFilename("MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: SystemPluginPtr systemPtr;
  public: MockSystem *mockSystem;

  private: SystemLoader sm;
};


/////////////////////////////////////////////////
// Single model consuming single batter
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(BatteryPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SingleBattery))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const UpdateInfo &, EntityComponentManager &_ecm)
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

      // Check state of charge is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/gazebosim/gz-sim/issues/55
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

  // Check state of charge after consumption is lower than 1 (full charge).
  EXPECT_LT(batComp->Data(), 1);

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

          // Check state of charge is lower than initial charge.
          EXPECT_LT(_batComp->Data(), 1);
        }

        return true;
      });
  EXPECT_EQ(linearBatCount, 1);
  EXPECT_EQ(totalBatCount, 2);
}

/////////////////////////////////////////////////
// Single battery with 1 extra consumer
TEST_F(BatteryPluginTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(SingleBatteryMultipleConsumers))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const UpdateInfo &, EntityComponentManager &_ecm)
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

      // Check state of charge is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/gazebosim/gz-sim/issues/55
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

  // Find the battery component.
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::BatterySoC::typeId));
  auto batComp = ecm->Component<components::BatterySoC>(batEntity);

  // Check state of charge after consumption is lower than initial one
  EXPECT_LT(batComp->Data(), 1);

  auto batLoad = batComp->Data();

  // Add Entity with a battery power load component
  Entity consumerEntity =  ecm->CreateEntity();
  components::BatteryPowerLoadInfo batteryPowerLoadInfo{batEntity, 500};
  ecm->CreateComponent(consumerEntity,
      components::BatteryPowerLoad(batteryPowerLoadInfo));

  // Reset battery state of charge and run the server
  batComp->Data() = 1;
  EXPECT_DOUBLE_EQ(batComp->Data(), 1.0);
  server.Run(true, 100, false);

  // Battery consumed this time should be lower than before due to
  // the extra consumer
  EXPECT_LT(batComp->Data(), batLoad);
}

/////////////////////////////////////////////////
// Two models with its own battery, one with one extra consumer.
TEST_F(BatteryPluginTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(BatteriesDifferentConsumers))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery_thruster_consumer.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;

      // Check a battery exists
      EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

      // Find the battery entities
      Entity batEntity = ecm->EntityByComponents(components::Name(
        "linear_battery"));
      EXPECT_NE(kNullEntity, batEntity);
      Entity batEntity2 = ecm->EntityByComponents(components::Name(
        "linear_battery2"));
      EXPECT_NE(kNullEntity, batEntity2);

      // Find the battery components
      EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
        components::BatterySoC::typeId));
      auto batComp = ecm->Component<components::BatterySoC>(batEntity);
      EXPECT_TRUE(ecm->EntityHasComponentType(batEntity2,
        components::BatterySoC::typeId));
      auto batComp2 = ecm->Component<components::BatterySoC>(batEntity2);

      // Check state of charge is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/gazebosim/gz-sim/issues/55
      EXPECT_GT(batComp->Data(), 0);
      EXPECT_GT(batComp2->Data(), 0);
    };

  // Start server
  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  EXPECT_NE(nullptr, ecm);

  // Check a battery exists
  EXPECT_TRUE(ecm->HasComponentType(components::BatterySoC::typeId));

  // Find the battery entities
  Entity batEntity = ecm->EntityByComponents(components::Name(
    "linear_battery"));
  EXPECT_NE(kNullEntity, batEntity);
  Entity batEntity2 = ecm->EntityByComponents(components::Name(
    "linear_battery2"));
  EXPECT_NE(kNullEntity, batEntity2);

  // Find the batteries components.
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity,
    components::BatterySoC::typeId));
  auto batComp = ecm->Component<components::BatterySoC>(batEntity);
  EXPECT_TRUE(ecm->EntityHasComponentType(batEntity2,
    components::BatterySoC::typeId));
  auto batComp2 = ecm->Component<components::BatterySoC>(batEntity2);

  // Check state of charge after consumption is lower than initial one
  EXPECT_LT(batComp->Data(), 1);
  EXPECT_LT(batComp2->Data(), 1);

  // Check state of charge of the battery with an extra consumer is lower
  // than the one without it.
  EXPECT_LT(batComp2->Data(), batComp->Data());
}

/////////////////////////////////////////////////
// Battery with power draining topics
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(BatteryPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PowerDrainTopic))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");
  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
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

      // Check state of charge is never zero.
      // This check is here to guarantee that components::BatterySoC in
      // the LinearBatteryPlugin is not zero when created. If
      // components::BatterySoC is zero on start, then the Physics plugin
      // can disable a joint. This in turn can prevent the joint from
      // rotating. See https://github.com/gazebosim/gz-sim/issues/55
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

  // Check state of charge should be 1, since the battery has not drained
  // and the <initial_charge> is equivalent to the <capacity>.
  EXPECT_DOUBLE_EQ(batComp->Data(), 1.0);

  // Send a message on one of the <power_draining_topic> topics, which will
  // start the battery draining when the server starts again.
  gz::transport::Node node;
  auto dischargePub = node.Advertise<msgs::StringMsg>("/battery/discharge");
  msgs::StringMsg msg;
  dischargePub.Publish(msg);

  // Run the server again.
  server.Run(true, 100, false);

  // Send a message on one of the <stop_power_draining_topic> topics, which
  // will stop the battery draining when the server starts again.
  auto stopPub = node.Advertise<msgs::StringMsg>("/battery/stop_discharge");
  stopPub.Publish(msg);

  // Run the server a little bit to allow msg be propagated inside plugin
  server.Run(true, 50, false);

  // The state of charge should be <1, since the battery has started
  // draining.
  const double stateOfCharge = batComp->Data();
  EXPECT_LT(batComp->Data(), 1.0);

  // Run the server again.
  server.Run(true, 100, false);

  // The state of charge should be the same since the discharge was stopped
  EXPECT_DOUBLE_EQ(batComp->Data(), stateOfCharge);
}

/////////////////////////////////////////////////
// Battery reset should restore SoC and clear topic-triggered drain state.
TEST_F(BatteryPluginTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(ResetClearsDrainState))
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "battery.sdf");

  ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfPath);

  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    };

  Server server(serverConfig);
  server.AddSystem(this->systemPtr);
  server.Run(true, 100, false);
  ASSERT_NE(nullptr, ecm);

  const auto findBatterySoC = [&]() -> std::pair<Entity, double>
  {
    const Entity batEntity = ecm->EntityByComponents(
      components::Name("linear_battery_topics"));
    EXPECT_NE(kNullEntity, batEntity);
    EXPECT_TRUE(ecm->EntityHasComponentType(
      batEntity, components::BatterySoC::typeId));
    const auto *batComp = ecm->Component<components::BatterySoC>(batEntity);
    EXPECT_NE(nullptr, batComp);
    return {batEntity, batComp->Data()};
  };

  const auto batteryCount = [&]() -> int
  {
    int count = 0;
    ecm->Each<components::BatterySoC, components::Name>(
      [&](const Entity &,
          const components::BatterySoC *,
          const components::Name *_nameComp) -> bool
      {
        if (_nameComp->Data() == "linear_battery_topics")
          ++count;
        return true;
      });
    return count;
  };

  const auto [initialBatteryEntity, initialSoC] = findBatterySoC();
  EXPECT_NE(kNullEntity, initialBatteryEntity);
  EXPECT_DOUBLE_EQ(initialSoC, 1.0);
  EXPECT_EQ(1, batteryCount());

  gz::transport::Node node;
  auto dischargePub = node.Advertise<msgs::StringMsg>("/battery/discharge");
  ASSERT_TRUE(gz::sim::test::WaitUntil(5s, [&dischargePub]
      {
        return dischargePub.HasConnections();
      }));

  msgs::StringMsg msg;
  const auto publishAndWaitForDrain =
    [&](auto &_pub, double _threshold) -> bool
    {
      return gz::sim::test::StepUntil(server, 200u, [&]()
      {
        _pub.Publish(msg);
        return findBatterySoC().second < _threshold;
      });
    };

  ASSERT_TRUE(publishAndWaitForDrain(dischargePub, 1.0));

  const auto [drainedBatteryEntity, drainedSoC] = findBatterySoC();
  EXPECT_NE(kNullEntity, drainedBatteryEntity);
  EXPECT_LT(drainedSoC, 1.0);

  server.ResetAll();
  server.Run(true, 2, false);
  server.Run(true, 100, false);

  const auto [resetBatteryEntity, resetSoC] = findBatterySoC();
  EXPECT_NE(kNullEntity, resetBatteryEntity);
  EXPECT_DOUBLE_EQ(resetSoC, 1.0);
  EXPECT_EQ(1, batteryCount());

  server.Run(true, 100, false);
  const auto [stableBatteryEntity, stableSoC] = findBatterySoC();
  EXPECT_NE(kNullEntity, stableBatteryEntity);
  EXPECT_DOUBLE_EQ(stableSoC, resetSoC);

  gz::transport::Node postResetNode;
  auto postResetDischargePub =
    postResetNode.Advertise<msgs::StringMsg>("/battery/discharge");
  ASSERT_TRUE(gz::sim::test::WaitUntil(5s, [&postResetDischargePub]
      {
        return postResetDischargePub.HasConnections();
      }));
  ASSERT_TRUE(publishAndWaitForDrain(postResetDischargePub, resetSoC));

  const auto [freshBatteryEntity, freshDrainSoC] = findBatterySoC();
  EXPECT_NE(kNullEntity, freshBatteryEntity);
  EXPECT_LT(freshDrainSoC, resetSoC);
}
