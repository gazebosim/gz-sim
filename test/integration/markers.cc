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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utilities/ExtraTestMacros.hh>

#include "gz/sim/rendering/MarkerManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace gz::sim;

/// \brief Test MarkersTest system
class MarkersTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::Marker> markerMsgs;

/////////////////////////////////////////////////
void markerCb(const msgs::Marker &_msg)
{
  std::cout << "OnMarkerCb\n";
  mutex.lock();
  markerMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
TEST_F(MarkersTest, MarkerPublisher)
{
  MarkerManager markerManager;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/empty.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);

  // subscribe to marker topic
  transport::Node node;
  node.Subscribe("/marker", &markerCb);

  gz::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::SPHERE);
  markerMsg.set_visibility(gz::msgs::Marker::GUI);
  node.Request("/marker", markerMsg);

//  // Run server and verify that we are receiving a message
//  // from the lidar
//  size_t iters100 = 100u;
//  server.Run(true, iters100, false);
//
  // Wait for a message to be received
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    bool received = !markerMsgs.empty();
    mutex.unlock();

    if (received)
      break;
  }

  mutex.lock();
  EXPECT_GT(markerMsgs.size(), 0u);
  auto lastMsg = markerMsgs.back();
  mutex.unlock();
}
