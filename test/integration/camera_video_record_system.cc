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
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test CameraVideoRecorder system
class CameraVideoRecorderTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(CameraVideoRecorderTest, IGN_UTILS_TEST_DISABLED_ON_MAC(RecordVideo))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/camera_video_record.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  server.Run(true, 1, false);

  ignition::transport::Node node;
  std::vector<std::string> services;
  bool hasService = false;

  // wait for record video service to appear
  int sleep = 0;
  while (sleep++ < 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    server.Run(true, 1, false);

    node.ServiceList(services);
    for (auto s :  services)
    {
      if (s == "/camera/record_video")
      {
        hasService = true;
        break;
      }
    }
    services.clear();
  }
  EXPECT_TRUE(hasService);

  ignition::msgs::VideoRecord videoRecordMsg;
  ignition::msgs::Boolean res;
  bool result = false;
  unsigned int timeout = 5000;

  videoRecordMsg.set_start(true);
  videoRecordMsg.set_format("mp4");
  videoRecordMsg.set_save_filename("test.mp4");

  // start video recording
  EXPECT_TRUE(node.Request("/camera/record_video",
      videoRecordMsg, timeout, res, result));

  // sleep for a few seconds
  sleep = 0;
  while (sleep++ < 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    server.Run(true, 1, false);
  }

  // stop video recording
  videoRecordMsg.set_stop(true);
  EXPECT_TRUE(node.Request("/camera/record_video",
      videoRecordMsg, timeout, res, result));

  // wait until the file is saved
  sleep = 0;
  while (!common::exists("test.mp4") && sleep++ < 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    server.Run(true, 1, false);
  }
  EXPECT_TRUE(common::exists("test.mp4"));
  EXPECT_TRUE(common::removeFile("test.mp4"));
}
