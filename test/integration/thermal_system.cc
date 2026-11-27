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

#include <cstddef>

#include <sdf/Camera.hh>
#include <sdf/Sensor.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/Temperature.hh"
#include "gz/sim/components/TemperatureRange.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test Thermal system
class ThermalTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ThermalTest, TemperatureComponent)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_BINARY_PATH,
      "test", "worlds", "thermal.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  server.Run(true, 1, false);

  auto ecm = server.EcmScope();
  ASSERT_TRUE(ecm.Valid());

  std::map<std::string, math::Temperature> entityTemp;
  std::map<std::string, components::TemperatureRangeInfo>
    entityTempRange;
  std::map<std::string, std::string> heatSignatures;
  ecm->Each<components::Temperature, components::Name>(
      [&](const Entity &_id,
          const components::Temperature *_temp,
          const components::Name *_name) -> bool
      {
        // store temperature data
        entityTemp[_name->Data()] = _temp->Data();

        // verify temperature data belongs to a visual
        EXPECT_NE(nullptr, ecm->Component<components::Visual>(_id));

        return true;
      });

  ecm->Each<components::TemperatureRange, components::SourceFilePath,
    components::Name>(
      [&](const gz::sim::Entity &_id,
          const components::TemperatureRange *_tempRange,
          const components::SourceFilePath *_heatSigURI,
          const components::Name *_name) -> bool
      {
        // store temperature range data
        entityTempRange[_name->Data()] = _tempRange->Data();

        // store heat signature URI data
        heatSignatures[_name->Data()] = _heatSigURI->Data();

        // verify temperature range data belongs to a visual
        EXPECT_NE(nullptr, ecm->Component<components::Visual>(_id));

        return true;
      });

  const std::string sphereVisual = "sphere_visual";
  const std::string cylinderVisual = "cylinder_visual";
  const std::string visual = "visual";
  const std::string heatSignatureCylinderVisual =
    "heat_signature_cylinder_visual";
  const std::string heatSignatureSphereVisual =
    "heat_signature_sphere_visual";
  const std::string heatSignatureSphereVisual2 =
    "heat_signature_sphere_visual_2";
  const std::string heatSignatureTestResource = "duck.png";

  // verify temperature components are created and the values are correct
  EXPECT_EQ(2u, entityTemp.size());
  ASSERT_TRUE(entityTemp.find(sphereVisual) != entityTemp.end());
  ASSERT_TRUE(entityTemp.find(cylinderVisual) != entityTemp.end());
  EXPECT_DOUBLE_EQ(600.0, entityTemp[sphereVisual].Kelvin());
  EXPECT_DOUBLE_EQ(400.0, entityTemp[cylinderVisual].Kelvin());

  EXPECT_EQ(4u, entityTempRange.size());
  ASSERT_TRUE(entityTempRange.find(visual) != entityTempRange.end());
  ASSERT_TRUE(entityTempRange.find(
        heatSignatureCylinderVisual) != entityTempRange.end());
  ASSERT_TRUE(entityTempRange.find(
        heatSignatureSphereVisual) != entityTempRange.end());
  ASSERT_TRUE(entityTempRange.find(
        heatSignatureSphereVisual2) != entityTempRange.end());
  EXPECT_DOUBLE_EQ(310.0, entityTempRange[visual].min.Kelvin());
  EXPECT_DOUBLE_EQ(310.0, entityTempRange[visual].max.Kelvin());
  EXPECT_DOUBLE_EQ(310.0,
      entityTempRange[heatSignatureCylinderVisual].min.Kelvin());
  EXPECT_DOUBLE_EQ(310.0,
      entityTempRange[heatSignatureCylinderVisual].max.Kelvin());
  EXPECT_DOUBLE_EQ(310.0,
      entityTempRange[heatSignatureSphereVisual].min.Kelvin());
  EXPECT_DOUBLE_EQ(500.0,
      entityTempRange[heatSignatureSphereVisual].max.Kelvin());
  EXPECT_DOUBLE_EQ(310.0,
      entityTempRange[heatSignatureSphereVisual2].min.Kelvin());
  EXPECT_DOUBLE_EQ(400.0,
      entityTempRange[heatSignatureSphereVisual2].max.Kelvin());

  EXPECT_EQ(4u, heatSignatures.size());
  ASSERT_TRUE(heatSignatures.find(visual) != heatSignatures.end());
  ASSERT_TRUE(heatSignatures.find(
        heatSignatureCylinderVisual) != heatSignatures.end());
  ASSERT_TRUE(heatSignatures.find(
        heatSignatureSphereVisual) != heatSignatures.end());
  ASSERT_TRUE(heatSignatures.find(
        heatSignatureSphereVisual2) != heatSignatures.end());
  EXPECT_TRUE(heatSignatures[visual].find(
        "RescueRandy_Thermal.png") != std::string::npos);
  EXPECT_TRUE(heatSignatures[heatSignatureCylinderVisual].find(
        heatSignatureTestResource) != std::string::npos);
  EXPECT_TRUE(heatSignatures[heatSignatureSphereVisual].find(
        heatSignatureTestResource) != std::string::npos);
  EXPECT_TRUE(heatSignatures[heatSignatureSphereVisual2].find(
        heatSignatureTestResource) != std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(ThermalTest, ThermalSensorSystem)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_BINARY_PATH,
      "test", "worlds", "thermal.sdf"));

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  server.Run(true, 1, false);

  auto ecm = server.EcmScope();
  ASSERT_TRUE(ecm.Valid());

  // Each only visits entities that have all of these components, so we just
  // need to check the data and make sure there's exactly one thermal camera
  std::size_t cameraCount{0};
  ecm->Each<components::ThermalCamera, components::Name,
            components::TemperatureLinearResolution,
            components::TemperatureRange>(
      [&](const Entity &,
          const components::ThermalCamera *_sensor,
          const components::Name *_name,
          const components::TemperatureLinearResolution *_resolution,
          const components::TemperatureRange *_range)
      {
        ++cameraCount;

        // verify camera properties from sdf
        EXPECT_EQ("thermal_camera_8bit", _name->Data());
        const sdf::Camera *cameraSdf = _sensor->Data().CameraSensor();
        EXPECT_NE(nullptr, cameraSdf);
        if (cameraSdf)
        {
          EXPECT_EQ(320u, cameraSdf->ImageWidth());
          EXPECT_EQ(240u, cameraSdf->ImageHeight());
          EXPECT_EQ(sdf::PixelFormatType::L_INT8, cameraSdf->PixelFormat());
        }

        // verify camera properties set through plugin
        EXPECT_DOUBLE_EQ(3.0, _resolution->Data());
        EXPECT_DOUBLE_EQ(253.15, _range->Data().min.Kelvin());
        EXPECT_DOUBLE_EQ(673.15, _range->Data().max.Kelvin());
        return true;
      });
  EXPECT_EQ(1u, cameraCount);
}
