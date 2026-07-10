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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include "gz/rendering/Scene.hh"
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/rendering/MarkerManager.hh"
#include "gz/sim/Server.hh"
#include "test_config.hh"

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
  mutex.lock();
  markerMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
void markerArrayCb(const gz::msgs::Boolean &, const bool)
{
}

/////////////////////////////////////////////////
TEST_F(MarkersTest, MarkerPublisher)
{
  std::map<std::string, std::string> params;
  auto engine = gz::rendering::engine("ogre2", params);
  auto scene = engine->CreateScene("testscene");

  gz::msgs::Marker markerMsg;

  // Function that Waits for a message to be received
  auto wait = [&](std::size_t _size, gz::msgs::Marker &_markerMsg) {
    for (int sleep = 0; sleep < 30; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      mutex.lock();
      bool received = markerMsgs.size() == _size;
      mutex.unlock();

      if (received)
        break;
    }

    mutex.lock();
    EXPECT_EQ(markerMsgs.size(), _size);
    auto lastMsg = markerMsgs.back();
    EXPECT_EQ(_markerMsg.DebugString(), lastMsg.DebugString());
    mutex.unlock();
  };


  MarkerManager markerManager;
  markerManager.Init(scene);

  // subscribe to marker topic
  transport::Node node;
  node.Subscribe("/marker", &markerCb);

  // Send a request, wait for a message
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::SPHERE);
  markerMsg.set_visibility(gz::msgs::Marker::GUI);
  node.Request("/marker", markerMsg);
  markerManager.Update();
  wait(1, markerMsg);

  // Update without a new message
  markerManager.Update();

  // Send another request, and check that there are two messages
  markerMsg.set_ns("default2");
  markerMsg.set_id(1);
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(gz::msgs::Marker::BOX);
  markerMsg.set_visibility(gz::msgs::Marker::GUI);
  node.Request("/marker", markerMsg);
  markerManager.Update();
  wait(2, markerMsg);
}

/////////////////////////////////////////////////
TEST_F(MarkersTest, MarkerArrayPublisher)
{
  std::map<std::string, std::string> params;
  auto engine = gz::rendering::engine("ogre2", params);
  auto scene = engine->CreateScene("testscene_marker_array");

  MarkerManager markerManager;
  markerManager.Init(scene);

  transport::Node node;

  gz::msgs::Marker_V markerArrayMsg;

  auto markerMsg0 = markerArrayMsg.add_marker();
  markerMsg0->set_ns("array");
  markerMsg0->set_id(1);
  markerMsg0->set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg0->set_type(gz::msgs::Marker::SPHERE);
  markerMsg0->set_visibility(gz::msgs::Marker::GUI);

  auto markerMsg1 = markerArrayMsg.add_marker();
  markerMsg1->set_ns("array");
  markerMsg1->set_id(2);
  markerMsg1->set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg1->set_type(gz::msgs::Marker::BOX);
  markerMsg1->set_visibility(gz::msgs::Marker::GUI);

  // Send an asynchronous marker array request.
  EXPECT_TRUE(node.Request(
      "/marker_array", markerArrayMsg, &markerArrayCb));

  rendering::VisualPtr visual0;
  rendering::VisualPtr visual1;

  for (int sleep = 0; sleep < 30; ++sleep)
  {
    // The service request is asynchronous, so keep processing messages.
    markerManager.Update();

    visual0 = scene->VisualByName("__GZ_MARKER_VISUAL_array_1");
    visual1 = scene->VisualByName("__GZ_MARKER_VISUAL_array_2");

    if (visual0 && visual1){
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_NE(nullptr, visual0);
  ASSERT_NE(nullptr, visual1);
}
