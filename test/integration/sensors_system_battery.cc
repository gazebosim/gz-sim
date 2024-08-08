/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <sdf/Root.hh>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/Name.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
namespace components = gz::sim::components;

unsigned int imgCount = 0u;
unsigned int depthImgCount = 0u;
std::mutex mutex;

/////////////////////////////////////////////////
void imageCb(const msgs::Image &)
{
  std::lock_guard<std::mutex> lock(mutex);
  imgCount++;
}

/////////////////////////////////////////////////
void depthCb(const msgs::Image &)
{
  std::lock_guard<std::mutex> lock(mutex);
  depthImgCount++;
}

//////////////////////////////////////////////////
class SensorsFixture : public InternalFixture<InternalFixture<::testing::Test>>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetName("gz::sim::MockSystem");
    sdfPlugin.SetFilename("MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
};

/////////////////////////////////////////////////
// Battery
TEST_F(SensorsFixture, SensorsBatteryState)
{
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "sensors_system_battery.sdf");
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

  // Check state of charge should be 1, since the batery has not drained
  // and the <initial_charge> is equivalent ot the <capacity>.
  EXPECT_DOUBLE_EQ(batComp->Data(), 1.0);

  gz::transport::Node node;

  // subscribe to img topics to make sure we are receiving images.
  node.Subscribe("/camera", &imageCb);
  node.Subscribe("/depth_camera", &depthCb);

  // Send a message on one of the <power_draining_topic> topics, which will
  // start the battery draining when the server starts again.
  auto pub = node.Advertise<msgs::StringMsg>("/battery/discharge");
  msgs::StringMsg msg;
  pub.Publish(msg);

  // Run the server again.
  server.Run(true, 50, false);

  // The state of charge should be <1, since the batery has started
  // draining.
  EXPECT_LT(batComp->Data(), 1.0);

  // wait and make sure camera is publishing images
  int sleep = 0;
  int maxSleep = 50;
  unsigned int icount = 0u;
  unsigned int dcount = 0u;
  while ((icount <= 0u || dcount <= 0u) && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> lock(mutex);
    icount = imgCount;
    dcount = depthImgCount;
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_LT(0u, imgCount);
    EXPECT_LT(0u, depthImgCount);
    imgCount = 0u;
    depthImgCount = 0u;
  }

  unsigned int totalIter = 0u;
  unsigned int iterToRun = 50u;

  // Run until battery is completely drained
  while (batComp->Data() > 0.0 && totalIter < 1000)
  {
    server.Run(true, iterToRun, false);
    totalIter += iterToRun;
  }
  EXPECT_LE(batComp->Data(), 0.0);

  // wait and make sure we receive expected no. of images
  int rate = 30;
  unsigned int expectedImgCount =  totalIter / rate;
  sleep = 0;
  icount = 0u;
  dcount = 0u;
  while ((icount < expectedImgCount || dcount << expectedImgCount)
      && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> lock(mutex);
    icount = imgCount;
    dcount = depthImgCount;
  }

  // verify image count
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_NEAR(expectedImgCount, imgCount, 1);
    EXPECT_NEAR(expectedImgCount, depthImgCount, 1);
    imgCount = 0u;
    depthImgCount = 0u;
  }

  // sensors should be disabled.
  // run for more iterations and verify we do not receive more images
  for (int i = 0; i < 5; ++i)
  {
    server.Run(true, iterToRun, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(0u, imgCount);
    EXPECT_EQ(0u, depthImgCount);
  }
}
