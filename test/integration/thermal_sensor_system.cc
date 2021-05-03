/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/TemperatureRange.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test Thermal system
class ThermalSensorTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};


std::mutex g_mutex;
std::vector<msgs::Image> g_imageMsgs;
unsigned char *g_image = nullptr;

/////////////////////////////////////////////////
void thermalCb(const msgs::Image &_msg)
{
  std::lock_guard<std::mutex> g_lock(g_mutex);
  g_imageMsgs.push_back(_msg);

  unsigned int width = _msg.width();
  unsigned int height = _msg.height();
  unsigned int size = width * height * sizeof(unsigned char);
  if (!g_image)
  {
    g_image = new unsigned char[size];
  }
  memcpy(g_image, _msg.data().c_str(), size);
}

/////////////////////////////////////////////////
TEST_F(ThermalSensorTest,
    IGN_UTILS_TEST_DISABLED_ON_MAC(ThermalSensorSystemInvalidConfig))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
        "test/worlds/thermal_invalid.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that checks for thermal component
  test::Relay testSystem;

  double resolution = 0;
  double minTemp = std::numeric_limits<double>::max();
  double maxTemp = 0.0;
  std::map<std::string, math::Temperature> entityTemp;
  std::string name;
  sdf::Sensor sensorSdf;

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
  testSystem.OnUpdate([&](const gazebo::UpdateInfo &,
    gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::ThermalCamera, components::Name>(
          [&](const ignition::gazebo::Entity &_id,
              const components::ThermalCamera *_sensor,
              const components::Name *_name) -> bool
          {
            // store temperature data
            sensorSdf = _sensor->Data();
            name = _name->Data();

            auto resolutionComp =
                _ecm.Component<components::TemperatureLinearResolution>(
                _id);
            if (resolutionComp)
              resolution = resolutionComp->Data();

            auto temperatureRangeComp =
                _ecm.Component<components::TemperatureRange>(_id);
            if (temperatureRangeComp)
            {
              auto info = temperatureRangeComp->Data();
              minTemp = info.min.Kelvin();
              maxTemp = info.max.Kelvin();
            }
            return true;
          });
    });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to thermal topic
  transport::Node node;
  node.Subscribe("/thermal_camera_invalid/image", &thermalCb);

  // Run server
  server.Run(true, 1, false);

 // verify camera properties from sdf
  unsigned int width = 320u;
  unsigned int height = 240u;
  EXPECT_EQ("thermal_camera_invalid", name);
  const sdf::Camera *cameraSdf = sensorSdf.CameraSensor();
  ASSERT_NE(nullptr, cameraSdf);
  EXPECT_EQ(width, cameraSdf->ImageWidth());
  EXPECT_EQ(height, cameraSdf->ImageHeight());
  EXPECT_EQ(sdf::PixelFormatType::L_INT8, cameraSdf->PixelFormat());

  // verify camera properties set through plugin
  // the resolution, min and max are invalid range values. Ign-gazebo should
  // print out warnings and use default values
  EXPECT_DOUBLE_EQ(0.0, resolution);
  EXPECT_DOUBLE_EQ(999.0, minTemp);
  EXPECT_DOUBLE_EQ(-590.0, maxTemp);

  // verify temperature of heat source
  const std::string sphereVisual = "sphere_visual";
  const std::string cylinderVisual = "cylinder_visual";
  const std::string boxVisual = "box_visual";
  EXPECT_EQ(3u, entityTemp.size());
  ASSERT_TRUE(entityTemp.find(sphereVisual) != entityTemp.end());
  ASSERT_TRUE(entityTemp.find(cylinderVisual) != entityTemp.end());
  ASSERT_TRUE(entityTemp.find(boxVisual) != entityTemp.end());
  EXPECT_DOUBLE_EQ(600.0, entityTemp[sphereVisual].Kelvin());
  // the user specified temp is larger than the max value representable by an
  // 8 bit 3 degree resolution camera - this value should be clamped
  EXPECT_DOUBLE_EQ(800.0, entityTemp[cylinderVisual].Kelvin());
  // the user specified temp is less than the min value of the camera - this
  // value should be clamped
  EXPECT_DOUBLE_EQ(-10.0, entityTemp[boxVisual].Kelvin());

  // Run server
  server.Run(true, 10, false);

  // wait for image
  bool received = false;
  for (unsigned int i = 0; i < 20; ++i)
  {
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      received = !g_imageMsgs.empty();
    }
    if (received)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(received);

  // check temperature in actual image output
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    unsigned int leftIdx = height * 0.5 * width;
    unsigned int rightIdx = leftIdx + width-1;
    unsigned int middleIdx = leftIdx + (0.5 * width);
    unsigned int defaultResolution = 3u;
    unsigned int cylinderTemp = g_image[leftIdx] * defaultResolution;
    unsigned int sphereTemp = g_image[rightIdx] * defaultResolution;
    unsigned int boxTemp = g_image[middleIdx] * defaultResolution;
    // default resolution, min, max values used so we should get correct
    // temperature value
    EXPECT_EQ(600u, sphereTemp);
    // 8 bit 3 degree resolution camera - this value should be clamped
    // in the image output to:
    //     2^(bitDepth) - 1 * resolution = 2^8 - 1 * 3 = 765
    EXPECT_EQ(765u, cylinderTemp);
    // the box resolution should be clamped to the camera's default
    // minimum temperature (0)
    EXPECT_EQ(0u, boxTemp);
  }

  g_imageMsgs.clear();
  delete [] g_image;
  g_image = nullptr;
}
