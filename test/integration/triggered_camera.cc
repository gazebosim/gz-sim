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
#include <gz/msgs/boolean.pb.h>

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <gz/msgs/image.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "gz/sim/rendering/Events.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test TriggeredCameraTest system
class TriggeredCameraTest : public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetFilename("libMockSystem.so");
    sdfPlugin.SetName("gz::sim::MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
  }

  public: SystemPluginPtr systemPtr;
  public: MockSystem *mockSystem;

  private: SystemLoader sm;
};

std::mutex mutex;
msgs::Image imageMsg;
unsigned char *imageBuffer = nullptr;
bool renderingStarted = false;

/////////////////////////////////////////////////
void imageCb(const msgs::Image &_msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  mutex.lock();
  unsigned int imageSamples = _msg.width() * _msg.height() * 3;

  if (!imageBuffer)
    imageBuffer = new unsigned char[imageSamples];
  memcpy(imageBuffer, _msg.data().c_str(), imageSamples);
  mutex.unlock();
}

/////////////////////////////////////////////////
void OnPostRender()
{
  std::lock_guard<std::mutex> lock(mutex);
  renderingStarted = true;
}

/////////////////////////////////////////////////
// The test checks the Triggered Camera readings
TEST_F(TriggeredCameraTest,
    GZ_UTILS_TEST_DISABLED_ON_MAC(TriggeredCameraBox))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "triggered_camera_sensor.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(100us);

  common::ConnectionPtr postRenderConn;
  this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
  this->mockSystem->configureCallback =
    [&](const Entity &,
           const std::shared_ptr<const sdf::Element> &,
           EntityComponentManager &,
           EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<events::PostRender>(
          std::bind(&::OnPostRender));
    };

  server.AddSystem(this->systemPtr);
  server.Run(false, 0, false);

  // wait for rendering to be initialized
  int sleep{0};
  int maxSleep{30};
  bool ready = false;
  while (!ready && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(100ms);
    std::lock_guard<std::mutex> lock(mutex);
    ready = renderingStarted;
  }

  // Subscribe to the image topic
  transport::Node node;
  node.Subscribe("/camera", &imageCb);

  transport::Node triggerNode;
  std::string triggerTopic =
      "/camera/trigger";

  auto pub = triggerNode.Advertise<msgs::Boolean>(triggerTopic);
  msgs::Boolean msg;
  msg.set_data(true);

  sleep = 0;
  while (imageBuffer == nullptr && sleep < maxSleep)
  {
    pub.Publish(msg);
    std::this_thread::sleep_for(100ms);
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
  ASSERT_NE(imageBuffer, nullptr);

  delete[] imageBuffer;
}
