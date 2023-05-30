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

#include <gz/msgs/logical_camera_image.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/LogicalCamera.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test LogicalCameraTest system
class LogicalCameraTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::LogicalCameraImage> logicalCamera1Msgs;
std::vector<msgs::LogicalCameraImage> logicalCamera2Msgs;

/////////////////////////////////////////////////
void logicalCamera1Cb(const msgs::LogicalCameraImage &_msg)
{
  mutex.lock();
  logicalCamera1Msgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
void logicalCamera2Cb(const msgs::LogicalCameraImage &_msg)
{
  mutex.lock();
  logicalCamera2Msgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// This test checks that both logical cameras in the world can see a box
// at the correct relative pose.
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(LogicalCameraTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogicalCameraBox))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/logical_camera_sensor.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName1 = "logical_camera-1";
  auto topic1 = "world/logical_camera_sensor/model/logical_camera-1/link/"
      "logical_camera_link/sensor/logical_camera-1/logical_camera";

  const std::string sensorName2 = "logical_camera-2";
  auto topic2 = "world/logical_camera_sensor/model/logical_camera-2/link/"
      "logical_camera_link/sensor/logical_camera-2/logical_camera";

  bool update1Checked{false};
  bool update2Checked{false};

  // Create a system that checks sensor topics
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &_info,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::LogicalCamera, components::Name>(
            [&](const Entity &_entity,
                const components::LogicalCamera *,
                const components::Name *_name) -> bool
            {
              // Sensor 1
              if (_name->Data() == sensorName1)
              {
                auto sensorComp = _ecm.Component<components::Sensor>(_entity);
                EXPECT_NE(nullptr, sensorComp);

                if (_info.iterations != 1)
                {
                  auto topicComp =
                      _ecm.Component<components::SensorTopic>(_entity);
                  EXPECT_NE(nullptr, topicComp);
                  if (topicComp)
                  {
                    EXPECT_EQ(topic1, topicComp->Data());
                  }
                }
                update1Checked = true;
              }
              // Sensor 2
              else if (_name->Data() == sensorName2)
              {
                auto sensorComp = _ecm.Component<components::Sensor>(_entity);
                EXPECT_NE(nullptr, sensorComp);

                if (_info.iterations != 1)
                {
                  auto topicComp =
                      _ecm.Component<components::SensorTopic>(_entity);
                  EXPECT_NE(nullptr, topicComp);
                  if (topicComp)
                  {
                    EXPECT_EQ(topic2, topicComp->Data());
                  }
                }
                update2Checked = true;
              }
              else
              {
                // We should not hit this, as it implies an unknown
                // sensor name.
                EXPECT_TRUE(false) << "Unknown sensor name.";
              }
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to logical camera topic
  transport::Node node;
  node.Subscribe(std::string("/world/logical_camera_sensor/") +
      "model/logical_camera-1/link/logical_camera_link" +
      "/sensor/logical_camera-1/logical_camera", &logicalCamera1Cb);

  node.Subscribe(std::string("/world/logical_camera_sensor/") +
  "model/logical_camera-2/link/logical_camera_link" +
  "/sensor/logical_camera-2/logical_camera", &logicalCamera2Cb);

  // Run server and verify that we are receiving messages
  size_t iters100 = 100u;
  server.Run(true, iters100, false);
  EXPECT_TRUE(update1Checked);
  EXPECT_TRUE(update2Checked);
  mutex.lock();
  EXPECT_GT(logicalCamera1Msgs.size(), 0u);
  mutex.unlock();

  mutex.lock();
  EXPECT_GT(logicalCamera2Msgs.size(), 0u);
  mutex.unlock();

  // Sensor 1 should see box
  std::string boxName = "box";
  math::Pose3d boxPose(1, 0, 0.5, 0, 0, 0);
  math::Pose3d sensor1Pose(0.05, 0.05, 0.55, 0, 0, 0);
  mutex.lock();
  msgs::LogicalCameraImage img1 = logicalCamera1Msgs.back();
  EXPECT_EQ(sensor1Pose, msgs::Convert(img1.pose()));
  EXPECT_EQ(1, img1.model().size());
  EXPECT_EQ(boxName, img1.model(0).name());
  math::Pose3d boxPoseCamera1Frame = sensor1Pose.Inverse() * boxPose;
  EXPECT_EQ(boxPoseCamera1Frame, msgs::Convert(img1.model(0).pose()));
  mutex.unlock();

  // Sensor 2 should see box too - note different sensor pose.
  math::Pose3d sensor2Pose(0.05, -0.45, 0.55, 0, 0, 0);
  mutex.lock();
  msgs::LogicalCameraImage img2 = logicalCamera2Msgs.back();
  EXPECT_EQ(sensor2Pose, msgs::Convert(img2.pose()));
  EXPECT_EQ(1, img2.model().size());
  EXPECT_EQ(boxName, img2.model(0).name());
  math::Pose3d boxPoseCamera2Frame = sensor2Pose.Inverse() * boxPose;
  EXPECT_EQ(boxPoseCamera2Frame, msgs::Convert(img2.model(0).pose()));
  mutex.unlock();
}
