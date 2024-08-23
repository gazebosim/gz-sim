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

#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>

#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/RgbdCamera.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/SegmentationCamera.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace std::chrono_literals;
namespace components = gz::sim::components;

//////////////////////////////////////////////////
class SensorsFixture : public InternalFixture<InternalFixture<::testing::Test>>
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
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
};

/////////////////////////////////////////////////
TEST_F(SensorsFixture, UpdateRate)
{
  gz::sim::ServerConfig serverConfig;

  const std::string sdfFile =
    common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "sensor.sdf");

  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->configureCallback =
    [&](const sim::Entity &,
           const std::shared_ptr<const sdf::Element> &,
           sim::EntityComponentManager &_ecm,
           sim::EventManager &)
    {
      ecm = &_ecm;
    };

  server.AddSystem(this->systemPtr);
  transport::Node node;
  std::string prefix{"/world/camera_sensor/model/default_topics/"};

  std::vector<double> imageTimestamps;
  unsigned int imageCount = 0u;
  auto cameraCb = std::function<void(const msgs::Image &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        imageTimestamps.push_back(t);
        ++imageCount;
      });
  node.Subscribe(prefix + "link/camera_link/sensor/camera/image", cameraCb);

  std::vector<double> lidarTimestamps;
  unsigned int lidarCount = 0u;
  auto lidarCb = std::function<void(const msgs::LaserScan &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        lidarTimestamps.push_back(t);
        ++lidarCount;
      });
  node.Subscribe(prefix + "link/gpu_lidar_link/sensor/gpu_lidar/scan", lidarCb);

  std::vector<double> depthTimestamps;
  unsigned int depthCount = 0u;
  auto depthCb = std::function<void(const msgs::Image &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        depthTimestamps.push_back(t);
        ++depthCount;
      });
  node.Subscribe(
      prefix + "link/depth_camera_link/sensor/depth_camera/depth_image",
      depthCb);

  std::vector<double> rgbdTimestamps;
  unsigned int rgbdCount = 0u;
  auto rgbdCb = std::function<void(const msgs::Image &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        rgbdTimestamps.push_back(t);
        ++rgbdCount;
      });
  node.Subscribe(
      prefix + "link/rgbd_camera_link/sensor/rgbd_camera/image",
      rgbdCb);

  std::vector<double> thermalTimestamps;
  unsigned int thermalCount = 0u;
  auto thermalCb = std::function<void(const msgs::Image &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        thermalTimestamps.push_back(t);
        ++thermalCount;
      });
  node.Subscribe(
      prefix + "link/thermal_camera_link/sensor/thermal_camera/image",
      thermalCb);

  std::vector<double> segmentationTimestamps;
  unsigned int segmentationCount = 0u;
  auto segmentationCb = std::function<void(const msgs::Image &)>(
      [&](const auto &_msg)
      {
        double t = _msg.header().stamp().sec() +
            _msg.header().stamp().nsec() * 1e-9;
        segmentationTimestamps.push_back(t);
        ++segmentationCount;
      });
  node.Subscribe(
      prefix + "link/segmentation_camera_link/sensor/segmentation_camera/" +
      "segmentation/colored_map",
      segmentationCb);

  unsigned int iterations = 2000u;
  server.Run(true, iterations, false);

  EXPECT_NE(nullptr, ecm);

  // get the world step size
  auto worldEntity = ecm->EntityByComponents(components::World());
  EXPECT_NE(sim::kNullEntity, worldEntity);
  auto physicsSdf = ecm->Component<components::Physics>(worldEntity)->Data();
  double stepSize = physicsSdf.MaxStepSize();
  EXPECT_LT(0, stepSize);

  // get the sensors update rates
  auto camLinkEntity = ecm->EntityByComponents(components::Name("camera_link"));
  EXPECT_NE(sim::kNullEntity, camLinkEntity);
  auto camEntity = ecm->EntityByComponents(components::Name("camera"),
      components::ParentEntity(camLinkEntity));
  EXPECT_NE(sim::kNullEntity, camEntity);
  auto sensorSdf = ecm->Component<components::Camera>(camEntity)->Data();
  unsigned int camRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, camRate);

  auto lidarEntity = ecm->EntityByComponents(components::Name("gpu_lidar"));
  EXPECT_NE(sim::kNullEntity, lidarEntity);
  sensorSdf = ecm->Component<components::GpuLidar>(lidarEntity)->Data();
  unsigned int lidarRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, lidarRate);

  auto depthEntity = ecm->EntityByComponents(components::Name("depth_camera"));
  EXPECT_NE(sim::kNullEntity, depthEntity);
  sensorSdf = ecm->Component<components::DepthCamera>(depthEntity)->Data();
  unsigned int depthRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, depthRate);

  auto rgbdEntity = ecm->EntityByComponents(components::Name("rgbd_camera"));
  EXPECT_NE(sim::kNullEntity, rgbdEntity);
  sensorSdf = ecm->Component<components::RgbdCamera>(rgbdEntity)->Data();
  unsigned int rgbdRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, rgbdRate);

  auto thermalEntity = ecm->EntityByComponents(
      components::Name("thermal_camera"));
  EXPECT_NE(sim::kNullEntity, thermalEntity);
  sensorSdf = ecm->Component<components::ThermalCamera>(thermalEntity)->Data();
  unsigned int thermalRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, thermalRate);

  auto segmentationEntity = ecm->EntityByComponents(
      components::Name("segmentation_camera"));
  EXPECT_NE(sim::kNullEntity, segmentationEntity);
  sensorSdf = ecm->Component<components::SegmentationCamera>(
      segmentationEntity)->Data();
  unsigned int segmentationRate = sensorSdf.UpdateRate();
  EXPECT_LT(0u, segmentationRate);

  // compute and verify expected msg count based on update rate and sim time
  double timeRan = iterations * stepSize;

  unsigned int expectedCamMsgCount = timeRan / (1.0 / camRate) + 1;
  unsigned int expectedDepthMsgCount = timeRan / (1.0 / depthRate) + 1;
  unsigned int expectedLidarMsgCount = timeRan / (1.0 / lidarRate) + 1;
  unsigned int expectedRgbdMsgCount = timeRan / (1.0 / rgbdRate) + 1;
  unsigned int expectedThermalMsgCount = timeRan / (1.0 / thermalRate) + 1;
  unsigned int expectedSegmentationMsgCount =
      timeRan / (1.0 / segmentationRate) + 1;

  unsigned int sleep = 0;
  unsigned int maxSleep = 100;
  while (sleep++ < maxSleep &&
         (imageCount < expectedCamMsgCount ||
         lidarCount < expectedLidarMsgCount ||
         depthCount < expectedDepthMsgCount ||
         rgbdCount < expectedRgbdMsgCount ||
         thermalCount < expectedThermalMsgCount ||
         segmentationCount < expectedSegmentationMsgCount))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_EQ(expectedCamMsgCount, imageCount);
  EXPECT_EQ(expectedDepthMsgCount, depthCount);
  EXPECT_EQ(expectedLidarMsgCount, lidarCount);
  EXPECT_EQ(expectedRgbdMsgCount, rgbdCount);
  EXPECT_EQ(expectedThermalMsgCount, thermalCount);
  EXPECT_EQ(expectedSegmentationMsgCount, segmentationCount);

  // verify timestamps
  // The first timestamp may not be 0 because the rendering
  // may take some time to start and it does not block the main thread
  // so start with index = 1
  // \todo(anyone) Make the sensors system thread block so we always
  // generate data at t = 0?
  EXPECT_FALSE(imageTimestamps.empty());
  EXPECT_FALSE(lidarTimestamps.empty());
  EXPECT_FALSE(depthTimestamps.empty());
  EXPECT_FALSE(rgbdTimestamps.empty());
  EXPECT_FALSE(thermalTimestamps.empty());
  EXPECT_FALSE(segmentationTimestamps.empty());
  for (unsigned int i = 1; i < imageTimestamps.size()-1; ++i)
  {
    double dt = imageTimestamps[i+1] - imageTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / camRate, dt);
  }
  for (unsigned int i = 1; i < lidarTimestamps.size()-1; ++i)
  {
    double dt = lidarTimestamps[i+1] - lidarTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / lidarRate, dt);
  }
  for (unsigned int i = 1; i < depthTimestamps.size()-1; ++i)
  {
    double dt = depthTimestamps[i+1] - depthTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / depthRate, dt);
  }
  for (unsigned int i = 1; i < rgbdTimestamps.size()-1; ++i)
  {
    double dt = rgbdTimestamps[i+1] - rgbdTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / rgbdRate, dt);
  }
  for (unsigned int i = 1; i < thermalTimestamps.size()-1; ++i)
  {
    double dt = thermalTimestamps[i+1] - thermalTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / thermalRate, dt);
  }
  for (unsigned int i = 1; i < segmentationTimestamps.size()-1; ++i)
  {
    double dt = segmentationTimestamps[i+1] - segmentationTimestamps[i];
    EXPECT_FLOAT_EQ(1.0 / segmentationRate, dt);
  }
}
