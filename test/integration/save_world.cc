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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <sstream>
#include <tinyxml2.h>

#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Camera.hh>
#include <sdf/Collision.hh>
#include <sdf/ForceTorque.hh>
#include <sdf/Imu.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Model.hh>
#include <sdf/Lidar.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "helpers/UniqueTestDirectoryEnv.hh"
#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class SdfGeneratorFixture : public InternalFixture<::testing::Test>
{
  public: void LoadWorld(const std::string &_path)
  {
    ServerConfig serverConfig;
    serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH, _path));

    std::cout << "Loading: " << serverConfig.SdfFile() << std::endl;
    this->server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
  }
  public: std::string RequestGeneratedSdf(const std::string &_worldName,
              const msgs::SdfGeneratorConfig &_req = msgs::SdfGeneratorConfig())
  {
    transport::Node node;

    msgs::StringMsg worldGenSdfRes;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/" + _worldName + "/generate_world_sdf"};
    EXPECT_TRUE(node.Request(service, _req, timeout, worldGenSdfRes, result));
    EXPECT_TRUE(result);
    return worldGenSdfRes.data();
  }

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(SdfGeneratorFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WorldWithModelsSpawnedAfterLoad))
{
  this->LoadWorld("test/worlds/save_world.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("inlineM1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack2"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack3"));
  EXPECT_FALSE(this->server->EntityByName("test_ground_plane").has_value());
  EXPECT_FALSE(this->server->EntityByName("spawned_model").has_value());

  auto modelStr = R"(
<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='spawned_model'>
    <link name='link'>
      <visual name='visual'>
        <geometry><sphere><radius>1.0</radius></sphere></geometry>
      </visual>
    </link>
  </model>
</sdf>)";

  // This has to be different from the backpack in order to test SDFormat
  // generation for a Fuel URI that was not known when simulation started.
  const std::string groundPlaneUri =
      "https://fuel.gazebosim.org/1.0/openrobotics/models/ground plane";

  transport::Node node;
  {
    // Spawn from Fuel
    msgs::EntityFactory req;
    req.set_sdf_filename(groundPlaneUri);
    req.set_name("test_ground_plane");

    msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/save_world/create"};

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    req.Clear();
    // Spawn from SDF string
    req.set_sdf(modelStr);
    req.mutable_pose()->mutable_position()->set_x(10);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
  }

  // Run an iteration and check it was created
  server->Run(true, 1, false);
  EXPECT_TRUE(this->server->EntityByName("test_ground_plane").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("test_ground_plane"));
  EXPECT_TRUE(this->server->EntityByName("spawned_model").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("spawned_model"));

  const std::string worldGenSdfRes = this->RequestGeneratedSdf("save_world");
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(6u, world->ModelCount());

  EXPECT_TRUE(world->ModelNameExists("inlineM1"));
  EXPECT_TRUE(world->ModelNameExists("backpack1"));
  EXPECT_TRUE(world->ModelNameExists("backpack2"));
  EXPECT_TRUE(world->ModelNameExists("backpack3"));
  EXPECT_TRUE(world->ModelNameExists("test_ground_plane"));
  EXPECT_TRUE(world->ModelNameExists("spawned_model"));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);

  // Check that the Fuel model spawned after load uses an include tag and has
  // the correct URI
  std::size_t includeCount = 0;
  std::size_t groundPlaneCount = 0;

  for (auto genInclude = genWorld->FirstChildElement("include"); genInclude;
       genInclude = genInclude->NextSiblingElement("include"), ++includeCount)
  {
    auto name = genInclude->FirstChildElement("name");
    ASSERT_NE(nullptr, name);

    if (std::strcmp(name->GetText(), "test_ground_plane") == 0)
    {
      auto genUri = genInclude->FirstChildElement("uri");
      ASSERT_NE(nullptr, genUri);
      EXPECT_STREQ(groundPlaneUri.c_str(), genUri->GetText());
      ++groundPlaneCount;
    }
  }

  EXPECT_EQ(4u, includeCount);
  EXPECT_EQ(1u, groundPlaneCount);

  // Check that spawned_model is included in the generated world as an expanded
  // model
  std::size_t modelCount = 0;
  std::size_t spawnedModelCount = 0;
  for (auto genModel = genWorld->FirstChildElement("model"); genModel;
       genModel = genModel->NextSiblingElement("model"), ++modelCount)
  {
    auto name = genModel->Attribute("name");
    ASSERT_NE(nullptr, name);

    if (std::strcmp(name, "spawned_model") == 0)
    {
      ++spawnedModelCount;
    }
  }

  EXPECT_EQ(2u, modelCount);
  EXPECT_EQ(1u, spawnedModelCount);
}

/////////////////////////////////////////////////
// Test segfaults on Mac at startup, possible collision with test above?
TEST_F(SdfGeneratorFixture,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ModelSpawnedWithNewName))
{
  this->LoadWorld("test/worlds/save_world.sdf");

  auto modelStr = R"(
<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='spawned_model'>
    <link name='link'/>
  </model>
</sdf>)";

  transport::Node node;
  msgs::EntityFactory req;
  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/save_world/create"};

  req.set_sdf(modelStr);
  req.set_name("new_model_name");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());
  // Run an iteration and check it was created
  server->Run(true, 1, false);
  EXPECT_TRUE(this->server->EntityByName("new_model_name").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("new_model_name"));

  const std::string worldGenSdfRes = this->RequestGeneratedSdf("save_world");
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_TRUE(world->ModelNameExists("new_model_name"));
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithNestedModel)
{
  this->LoadWorld("test/worlds/nested_model.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("model_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("collision_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("visual_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("model_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("collision_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("visual_01"));

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("nested_model_world");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->ModelCount());

  EXPECT_TRUE(world->ModelNameExists("model_00"));
  EXPECT_FALSE(world->ModelNameExists("model_01"));

  auto *model00 = world->ModelByName("model_00");
  ASSERT_NE(nullptr, model00);
  EXPECT_EQ(1u, model00->LinkCount());
  EXPECT_EQ(1u, model00->ModelCount());

  auto *link00 = model00->LinkByName("link_00");
  ASSERT_NE(nullptr, link00);
  EXPECT_EQ(1u, link00->CollisionCount());
  EXPECT_NE(nullptr, link00->CollisionByName("collision_00"));
  EXPECT_EQ(1u, link00->VisualCount());
  EXPECT_NE(nullptr, link00->VisualByName("visual_00"));

  auto *model01 = model00->ModelByName("model_01");
  ASSERT_NE(nullptr, model01);
  EXPECT_EQ(1u, model01->LinkCount());

  auto *link01 = model01->LinkByName("link_01");
  ASSERT_NE(nullptr, link01);
  EXPECT_EQ(1u, link01->CollisionCount());
  EXPECT_NE(nullptr, link01->CollisionByName("collision_01"));
  EXPECT_EQ(1u, link01->VisualCount());
  EXPECT_NE(nullptr, link01->VisualByName("visual_01"));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);
}


/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, ModelWithNestedIncludes)
{
  std::string path =
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models");
  common::setenv("GZ_SIM_RESOURCE_PATH", path);

  this->LoadWorld("test/worlds/model_nested_include.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("ground_plane"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("L0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M2"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M3"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("coke"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("L1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("include_nested_new_name"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("sphere"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C"));

  msgs::SdfGeneratorConfig req;
  req.mutable_global_entity_gen_config()
     ->mutable_save_fuel_version()->set_data(true);

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("model_nested_include_world", req);

  // check that model w/ nested includes are not expanded
  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);

  auto model = genWorld->FirstChildElement("model");  // ground_plane
  ASSERT_NE(nullptr, model);
  model = model->NextSiblingElement("model");  // M1
  ASSERT_NE(nullptr, model);

  // M1's child include
  auto include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  auto uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ("include_nested", std::string(uri->GetText()));

  auto name = include->FirstChildElement("name");
  ASSERT_NE(nullptr, name);
  ASSERT_NE(nullptr, name->GetText());
  EXPECT_EQ("include_nested_new_name", std::string(name->GetText()));

  auto pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  std::stringstream ss(pose->GetText());
  gz::math::Pose3d p;
  ss >> p;
  EXPECT_EQ(gz::math::Pose3d(1, 2, 3, 0, 0, 0), p);

  // M2
  model = model->FirstChildElement("model");
  ASSERT_NE(nullptr, model);

  // M2's child include
  include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ("sphere", std::string(uri->GetText()));

  name = include->FirstChildElement("name");
  EXPECT_EQ(nullptr, name);

  pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  ss = std::stringstream(pose->GetText());
  ss >> p;
  EXPECT_EQ(gz::math::Pose3d(0, 2, 2, 0, 0, 0), p);

  // M3
  model = model->FirstChildElement("model");
  ASSERT_NE(nullptr, model);

  // M3's child include
  include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ(
    "https://fuel.gazebosim.org/1.0/openrobotics/models/coke can/3",
     std::string(uri->GetText()));

  name = include->FirstChildElement("name");
  ASSERT_NE(nullptr, name);
  ASSERT_NE(nullptr, name->GetText());
  EXPECT_EQ("coke", std::string(name->GetText()));

  pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  ss = std::stringstream(pose->GetText());
  ss >> p;
  EXPECT_EQ(gz::math::Pose3d(2, 2, 2, 0, 0, 0), p);

  // check reloading generated sdf
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithSensors)
{
  this->LoadWorld("test/worlds/non_rendering_sensors.sdf");

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("non_rendering_sensors");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  EXPECT_TRUE(world->ModelNameExists("model"));
  auto *model = world->ModelByName("model");
  ASSERT_NE(nullptr, model);
  auto *link = model->LinkByName("link");
  ASSERT_NE(nullptr, link);

  // altimeter
  {
    auto *sensor = link->SensorByName("altimeter_sensor");
    ASSERT_NE(nullptr, sensor);
    const sdf::Altimeter *altimeter = sensor->AltimeterSensor();
    ASSERT_NE(nullptr, altimeter);
    const sdf::Noise &posNoise = altimeter->VerticalPositionNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, posNoise.Type());
    EXPECT_DOUBLE_EQ(0.1, posNoise.Mean());
    EXPECT_DOUBLE_EQ(0.2, posNoise.StdDev());

    const sdf::Noise &velNoise = altimeter->VerticalVelocityNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, velNoise.Type());
    EXPECT_DOUBLE_EQ(2.3, velNoise.Mean());
    EXPECT_DOUBLE_EQ(4.5, velNoise.StdDev());
  }

  // contact sensor
  {
    // contact sensor does not have an SDF DOM class
    auto *sensor = link->SensorByName("contact_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(4, 5, 6, 0, 0, 0), sensor->RawPose());
    EXPECT_EQ(sdf::SensorType::CONTACT, sensor->Type());
  }

  // force torque
  {
    auto *sensor = link->SensorByName("force_torque_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(10, 11, 12, 0, 0, 0), sensor->RawPose());
    const sdf::ForceTorque *forceTorque = sensor->ForceTorqueSensor();
    ASSERT_NE(nullptr, forceTorque);
    EXPECT_EQ(sdf::ForceTorqueFrame::CHILD, forceTorque->Frame());
    EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
        forceTorque->MeasureDirection());
    const sdf::Noise &forceXNoise = forceTorque->ForceXNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN_QUANTIZED, forceXNoise.Type());
    EXPECT_DOUBLE_EQ(0.02, forceXNoise.Mean());
    EXPECT_DOUBLE_EQ(0.0005, forceXNoise.StdDev());
     const sdf::Noise &torqueYNoise = forceTorque->TorqueYNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, torqueYNoise.Type());
    EXPECT_DOUBLE_EQ(0.009, torqueYNoise.Mean());
    EXPECT_DOUBLE_EQ(0.0000985, torqueYNoise.StdDev());
  }

  // imu
  {
    auto *sensor = link->SensorByName("imu_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(4, 5, 6, 0, 0, 0), sensor->RawPose());
    const sdf::Imu *imu = sensor->ImuSensor();
    ASSERT_NE(nullptr, imu);
    const sdf::Noise &linAccXNoise = imu->LinearAccelerationXNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, linAccXNoise.Type());
    EXPECT_DOUBLE_EQ(0.0, linAccXNoise.Mean());
    EXPECT_DOUBLE_EQ(0.1, linAccXNoise.StdDev());
    EXPECT_DOUBLE_EQ(0.2, linAccXNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(1.0, linAccXNoise.DynamicBiasCorrelationTime());
    const sdf::Noise &linAccYNoise = imu->LinearAccelerationYNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, linAccYNoise.Type());
    EXPECT_DOUBLE_EQ(1.0, linAccYNoise.Mean());
    EXPECT_DOUBLE_EQ(1.1, linAccYNoise.StdDev());
    EXPECT_DOUBLE_EQ(1.2, linAccYNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(2.0, linAccYNoise.DynamicBiasCorrelationTime());
    const sdf::Noise &linAccZNoise = imu->LinearAccelerationZNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, linAccZNoise.Type());
    EXPECT_DOUBLE_EQ(2.0, linAccZNoise.Mean());
    EXPECT_DOUBLE_EQ(2.1, linAccZNoise.StdDev());
    EXPECT_DOUBLE_EQ(2.2, linAccZNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(3.0, linAccZNoise.DynamicBiasCorrelationTime());
    const sdf::Noise &angVelXNoise = imu->AngularVelocityXNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, angVelXNoise.Type());
    EXPECT_DOUBLE_EQ(3.0, angVelXNoise.Mean());
    EXPECT_DOUBLE_EQ(3.1, angVelXNoise.StdDev());
    EXPECT_DOUBLE_EQ(4.2, angVelXNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(4.0, angVelXNoise.DynamicBiasCorrelationTime());
    const sdf::Noise &angVelYNoise = imu->AngularVelocityYNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, angVelYNoise.Type());
    EXPECT_DOUBLE_EQ(4.0, angVelYNoise.Mean());
    EXPECT_DOUBLE_EQ(4.1, angVelYNoise.StdDev());
    EXPECT_DOUBLE_EQ(5.2, angVelYNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(5.0, angVelYNoise.DynamicBiasCorrelationTime());
    const sdf::Noise &angVelZNoise = imu->AngularVelocityZNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, angVelZNoise.Type());
    EXPECT_DOUBLE_EQ(5.0, angVelZNoise.Mean());
    EXPECT_DOUBLE_EQ(5.1, angVelZNoise.StdDev());
    EXPECT_DOUBLE_EQ(6.2, angVelZNoise.DynamicBiasStdDev());
    EXPECT_DOUBLE_EQ(6.0, angVelZNoise.DynamicBiasCorrelationTime());

    EXPECT_EQ("ENU", imu->Localization());
    EXPECT_EQ("linka", imu->CustomRpyParentFrame());
    EXPECT_EQ(math::Vector3d::UnitY, imu->CustomRpy());
    EXPECT_EQ("linkb", imu->GravityDirXParentFrame());
    EXPECT_EQ(math::Vector3d::UnitZ, imu->GravityDirX());
    EXPECT_FALSE(imu->OrientationEnabled());
  }

  // logical camera
  {
    // logical camera sensor does not have an SDF DOM class
    auto *sensor = link->SensorByName("logical_camera_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(7, 8, 9, 0, 0, 0), sensor->RawPose());
    EXPECT_EQ(sdf::SensorType::LOGICAL_CAMERA, sensor->Type());
  }

  // magnetometer
  {
    auto *sensor = link->SensorByName("magnetometer_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(10, 11, 12, 0, 0, 0), sensor->RawPose());
    const sdf::Magnetometer *magnetometer = sensor->MagnetometerSensor();
    ASSERT_NE(nullptr, magnetometer);
    const sdf::Noise &xNoise = magnetometer->XNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, xNoise.Type());
    EXPECT_DOUBLE_EQ(0.1, xNoise.Mean());
    EXPECT_DOUBLE_EQ(0.2, xNoise.StdDev());
    const sdf::Noise &yNoise = magnetometer->YNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, yNoise.Type());
    EXPECT_DOUBLE_EQ(1.2, yNoise.Mean());
    EXPECT_DOUBLE_EQ(2.3, yNoise.StdDev());
    const sdf::Noise &zNoise = magnetometer->ZNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, zNoise.Type());
    EXPECT_DOUBLE_EQ(3.4, zNoise.Mean());
    EXPECT_DOUBLE_EQ(5.6, zNoise.StdDev());
  }

  // air pressure
  {
    auto *sensor = link->SensorByName("air_pressure_sensor");
    ASSERT_NE(nullptr, sensor);
    EXPECT_EQ(math::Pose3d(10, 20, 30, 0, 0, 0), sensor->RawPose());
    const sdf::AirPressure *airPressure = sensor->AirPressureSensor();
    ASSERT_NE(nullptr, airPressure);
    EXPECT_DOUBLE_EQ(123.4, airPressure->ReferenceAltitude());
    const sdf::Noise &noise = airPressure->PressureNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN, noise.Type());
    EXPECT_DOUBLE_EQ(3.4, noise.Mean());
    EXPECT_DOUBLE_EQ(5.6, noise.StdDev());
  }

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithRenderingSensors)
{
  this->LoadWorld("test/worlds/sensor.sdf");

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("camera_sensor");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // camera
  {
    EXPECT_TRUE(world->ModelNameExists("camera"));
    auto *model = world->ModelByName("camera");
    ASSERT_NE(nullptr, model);
    EXPECT_EQ(1u, model->LinkCount());

    auto *link = model->LinkByName("link");
    ASSERT_NE(nullptr, link);
    math::MassMatrix3d massMatrix(0.1,
       math::Vector3d( 0.000166667, 0.000166667, 0.000166667),
       math::Vector3d::Zero);
    math::Inertiald inertial(massMatrix, math::Pose3d::Zero);
    EXPECT_EQ(inertial, link->Inertial());

    auto *cameraSensor = link->SensorByName("camera");
    ASSERT_NE(nullptr, cameraSensor);
    EXPECT_EQ("camera", cameraSensor->Topic());
    const sdf::Camera *camera = cameraSensor->CameraSensor();
    ASSERT_NE(nullptr, camera);
    EXPECT_DOUBLE_EQ(1.047, camera->HorizontalFov().Radian());
    EXPECT_EQ(320u, camera->ImageWidth());
    EXPECT_EQ(240u, camera->ImageHeight());
    EXPECT_DOUBLE_EQ(0.1, camera->NearClip());
    EXPECT_DOUBLE_EQ(100, camera->FarClip());
    const sdf::Noise &noise = camera->ImageNoise();
    EXPECT_EQ(sdf::NoiseType::GAUSSIAN_QUANTIZED, noise.Type());
    EXPECT_DOUBLE_EQ(0.01, noise.Mean());
    EXPECT_DOUBLE_EQ(0.0002, noise.StdDev());
  }

  EXPECT_TRUE(world->ModelNameExists("default_topics"));
  auto *model = world->ModelByName("default_topics");
  ASSERT_NE(nullptr, model);
  // gpu lidar
  {
    auto *gpuLidarLink = model->LinkByName("gpu_lidar_link");
    ASSERT_NE(nullptr, gpuLidarLink);
    auto *gpuLidarSensor = gpuLidarLink->SensorByName("gpu_lidar");
    const sdf::Lidar *lidar = gpuLidarSensor->LidarSensor();
    EXPECT_EQ(640u, lidar->HorizontalScanSamples());
    EXPECT_DOUBLE_EQ(1.0, lidar->HorizontalScanResolution());
    EXPECT_NEAR(-1.396263, lidar->HorizontalScanMinAngle().Radian(), 1e-5);
    EXPECT_NEAR(1.396263, lidar->HorizontalScanMaxAngle().Radian(), 1e-5);
    EXPECT_EQ(1u, lidar->VerticalScanSamples());
    EXPECT_DOUBLE_EQ(0.01, lidar->VerticalScanResolution());
    EXPECT_DOUBLE_EQ(0.0, lidar->VerticalScanMinAngle().Radian());
    EXPECT_DOUBLE_EQ(0.0, lidar->VerticalScanMaxAngle().Radian());
    EXPECT_DOUBLE_EQ(0.08, lidar->RangeMin());
    EXPECT_DOUBLE_EQ(10.0, lidar->RangeMax());
    EXPECT_DOUBLE_EQ(0.01, lidar->RangeResolution());
  }

  // depth camera
  {
    auto *depthLink = model->LinkByName("depth_camera_link");
    ASSERT_NE(nullptr, depthLink);
    auto *depthSensor = depthLink->SensorByName("depth_camera");
    ASSERT_NE(nullptr, depthSensor);
    const sdf::Camera *camera = depthSensor->CameraSensor();
    ASSERT_NE(nullptr, camera);
    EXPECT_DOUBLE_EQ(1.05, camera->HorizontalFov().Radian());
    EXPECT_EQ(256u, camera->ImageWidth());
    EXPECT_EQ(256u, camera->ImageHeight());
    EXPECT_EQ("R_FLOAT32", camera->PixelFormatStr());
    EXPECT_DOUBLE_EQ(0.1, camera->NearClip());
    EXPECT_DOUBLE_EQ(10, camera->FarClip());
    EXPECT_DOUBLE_EQ(0.05, camera->DepthNearClip());
    EXPECT_DOUBLE_EQ(9.0, camera->DepthFarClip());
  }

  // rgbd camera
  {
    auto *rgbdLink = model->LinkByName("rgbd_camera_link");
    ASSERT_NE(nullptr, rgbdLink);
    auto *rgbdSensor = rgbdLink->SensorByName("rgbd_camera");
    ASSERT_NE(nullptr, rgbdSensor);
    const sdf::Camera *camera = rgbdSensor->CameraSensor();
    ASSERT_NE(nullptr, camera);
    EXPECT_DOUBLE_EQ(1.05, camera->HorizontalFov().Radian());
    EXPECT_EQ(256u, camera->ImageWidth());
    EXPECT_EQ(256u, camera->ImageHeight());
    EXPECT_DOUBLE_EQ(0.1, camera->NearClip());
    EXPECT_DOUBLE_EQ(10, camera->FarClip());
  }

  // thermal camera
  {
    auto *thermalLink = model->LinkByName("thermal_camera_link");
    ASSERT_NE(nullptr, thermalLink);
    auto *thermalSensor = thermalLink->SensorByName("thermal_camera");
    ASSERT_NE(nullptr, thermalSensor);
    const sdf::Camera *camera = thermalSensor->CameraSensor();
    ASSERT_NE(nullptr, camera);
    EXPECT_DOUBLE_EQ(1.15, camera->HorizontalFov().Radian());
    EXPECT_EQ(300u, camera->ImageWidth());
    EXPECT_EQ(200u, camera->ImageHeight());
    EXPECT_DOUBLE_EQ(0.14, camera->NearClip());
    EXPECT_DOUBLE_EQ(120.0, camera->FarClip());
  }

  // segmentation camera
  {
    auto *segmentationLink = model->LinkByName("segmentation_camera_link");
    ASSERT_NE(nullptr, segmentationLink);
    auto *segmentationSensor =
        segmentationLink->SensorByName("segmentation_camera");
    ASSERT_NE(nullptr, segmentationSensor);
    const sdf::Camera *camera = segmentationSensor->CameraSensor();
    ASSERT_NE(nullptr, camera);
    EXPECT_DOUBLE_EQ(1.0, camera->HorizontalFov().Radian());
    EXPECT_EQ(320u, camera->ImageWidth());
    EXPECT_EQ(240u, camera->ImageHeight());
    EXPECT_DOUBLE_EQ(0.11, camera->NearClip());
    EXPECT_DOUBLE_EQ(20.0, camera->FarClip());
    EXPECT_EQ("panoptic", camera->SegmentationType());
  }

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithLights)
{
  this->LoadWorld("test/worlds/lights.sdf");

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("lights");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);

  // directional light in the world
  {
    const sdf::Light *light = world->LightByIndex(0u);
    EXPECT_EQ("directional", light->Name());
    EXPECT_EQ(sdf::LightType::DIRECTIONAL, light->Type());
    EXPECT_EQ(gz::math::Pose3d(0, 0, 10, 0, 0, 0),
        light->RawPose());
    EXPECT_EQ(std::string(), light->PoseRelativeTo());
    EXPECT_TRUE(light->CastShadows());
    EXPECT_EQ(gz::math::Color(0.8f, 0.8f, 0.8f, 1),
        light->Diffuse());
    EXPECT_EQ(gz::math::Color(0.2f, 0.2f, 0.2f, 1),
        light->Specular());
    EXPECT_DOUBLE_EQ(100, light->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.9, light->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.01, light->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.001, light->QuadraticAttenuationFactor());
    EXPECT_EQ(gz::math::Vector3d(0.5, 0.2, -0.9),
        light->Direction());
  }
  // point light in the world
  {
    const sdf::Light *light = world->LightByIndex(1u);
    EXPECT_EQ("point", light->Name());
    EXPECT_EQ(sdf::LightType::POINT, light->Type());
    EXPECT_EQ(gz::math::Pose3d(0, -1.5, 3, 0, 0, 0),
        light->RawPose());
    EXPECT_FALSE(light->CastShadows());
    EXPECT_EQ(gz::math::Color(1.0f, 0.0f, 0.0f, 1),
        light->Diffuse());
    EXPECT_EQ(gz::math::Color(0.1f, 0.1f, 0.1f, 1),
        light->Specular());
    EXPECT_DOUBLE_EQ(4, light->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.2, light->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.5, light->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.01, light->QuadraticAttenuationFactor());
  }
  // spot light in the world
  {
    const sdf::Light *light = world->LightByIndex(2u);
    EXPECT_EQ("spot", light->Name());
    EXPECT_EQ(sdf::LightType::SPOT, light->Type());
    EXPECT_EQ(gz::math::Pose3d(0, 1.5, 3, 0, 0, 0),
        light->RawPose());
    EXPECT_EQ(std::string(), light->PoseRelativeTo());
    EXPECT_FALSE(light->CastShadows());
    EXPECT_EQ(gz::math::Color(0.0f, 1.0f, 0.0f, 1),
        light->Diffuse());
    EXPECT_EQ(gz::math::Color(0.2f, 0.2f, 0.2f, 1),
        light->Specular());
    EXPECT_DOUBLE_EQ(5, light->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.3, light->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.4, light->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.001, light->QuadraticAttenuationFactor());
    EXPECT_EQ(gz::math::Vector3d(0.0, 0.0, -1.0),
        light->Direction());
    EXPECT_DOUBLE_EQ(0.1, light->SpotInnerAngle().Radian());
    EXPECT_DOUBLE_EQ(0.5, light->SpotOuterAngle().Radian());
    EXPECT_DOUBLE_EQ(0.8, light->SpotFalloff());
  }

  // get model
  EXPECT_TRUE(world->ModelNameExists("sphere"));
  auto *model = world->ModelByName("sphere");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(1u, model->LinkCount());

  // get link
  auto *link = model->LinkByName("sphere_link");
  ASSERT_NE(nullptr, link);

  // light attached to link
  {
    const sdf::Light *light = link->LightByName("link_light_point");
    EXPECT_EQ("link_light_point", light->Name());
    EXPECT_EQ(gz::math::Pose3d(0.0, 0.0, 1.0, 0, 0, 0),
        light->RawPose());
    EXPECT_EQ(sdf::LightType::POINT, light->Type());
    EXPECT_FALSE(light->CastShadows());
    EXPECT_EQ(gz::math::Color(0.0f, 0.0f, 1.0f, 1),
        light->Diffuse());
    EXPECT_EQ(gz::math::Color(0.1f, 0.1f, 0.1f, 1),
        light->Specular());
    EXPECT_DOUBLE_EQ(2, light->AttenuationRange());
    EXPECT_DOUBLE_EQ(0.05, light->ConstantAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.02, light->LinearAttenuationFactor());
    EXPECT_DOUBLE_EQ(0.01, light->QuadraticAttenuationFactor());
  }

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, ModelWithJoints)
{
  this->LoadWorld(gz::common::joinPaths("test", "worlds",
      "joint_sensor.sdf"));

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("joint_sensor");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->ModelCount());

  EXPECT_TRUE(world->ModelNameExists("model"));
  auto *model = world->ModelByName("model");
  ASSERT_NE(nullptr, model);
  EXPECT_EQ(2u, model->LinkCount());
  auto *link1 = model->LinkByName("link1");
  ASSERT_NE(nullptr, link1);
  auto *link2 = model->LinkByName("link2");
  ASSERT_NE(nullptr, link2);
  EXPECT_EQ(1u, model->JointCount());
  auto *joint = model->JointByName("joint");
  ASSERT_NE(nullptr, joint);

  EXPECT_EQ("link1", joint->ParentName());
  EXPECT_EQ("link2", joint->ChildName());
  EXPECT_EQ(sdf::JointType::REVOLUTE2, joint->Type());

  // Get the first axis
  const sdf::JointAxis *axis = joint->Axis();
  ASSERT_NE(nullptr, axis);
  ASSERT_NE(nullptr, axis->Element());

  // Get the second axis
  const sdf::JointAxis *axis2 = joint->Axis(1);
  ASSERT_NE(nullptr, axis2);

  EXPECT_EQ(gz::math::Vector3d::UnitZ, axis->Xyz());
  EXPECT_EQ(gz::math::Vector3d::UnitY, axis2->Xyz());

  EXPECT_EQ("", axis->XyzExpressedIn());
  EXPECT_TRUE(axis2->XyzExpressedIn().empty());

  EXPECT_DOUBLE_EQ(-0.5, axis->Lower());
  EXPECT_DOUBLE_EQ(0.5, axis->Upper());
  EXPECT_DOUBLE_EQ(-1.0, axis2->Lower());
  EXPECT_DOUBLE_EQ(1.0, axis2->Upper());

  EXPECT_DOUBLE_EQ(123.4, axis->Effort());
  EXPECT_DOUBLE_EQ(0.5, axis2->Effort());

  EXPECT_DOUBLE_EQ(12.0, axis->MaxVelocity());
  EXPECT_DOUBLE_EQ(200.0, axis2->MaxVelocity());

  EXPECT_DOUBLE_EQ(0.1, axis->Damping());
  EXPECT_DOUBLE_EQ(0.0, axis2->Damping());

  EXPECT_DOUBLE_EQ(0.2, axis->Friction());
  EXPECT_DOUBLE_EQ(0.0, axis2->Friction());

  EXPECT_DOUBLE_EQ(1.3, axis->SpringReference());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringReference());

  EXPECT_DOUBLE_EQ(10.6, axis->SpringStiffness());
  EXPECT_DOUBLE_EQ(0.0, axis2->SpringStiffness());

  // sensor
  const sdf::Sensor *forceTorqueSensor =
    joint->SensorByName("force_torque_sensor");
  ASSERT_NE(nullptr, forceTorqueSensor);
  EXPECT_EQ("force_torque_sensor", forceTorqueSensor->Name());
  EXPECT_EQ(sdf::SensorType::FORCE_TORQUE, forceTorqueSensor->Type());
  EXPECT_EQ(gz::math::Pose3d(10, 11, 12, 0, 0, 0),
      forceTorqueSensor->RawPose());
  auto forceTorqueSensorObj = forceTorqueSensor->ForceTorqueSensor();
  ASSERT_NE(nullptr, forceTorqueSensorObj);
  EXPECT_EQ(sdf::ForceTorqueFrame::PARENT, forceTorqueSensorObj->Frame());
  EXPECT_EQ(sdf::ForceTorqueMeasureDirection::PARENT_TO_CHILD,
      forceTorqueSensorObj->MeasureDirection());

  EXPECT_DOUBLE_EQ(0.0, forceTorqueSensorObj->ForceXNoise().Mean());
  EXPECT_DOUBLE_EQ(0.1, forceTorqueSensorObj->ForceXNoise().StdDev());
  EXPECT_DOUBLE_EQ(1.0, forceTorqueSensorObj->ForceYNoise().Mean());
  EXPECT_DOUBLE_EQ(1.1, forceTorqueSensorObj->ForceYNoise().StdDev());
  EXPECT_DOUBLE_EQ(2.0, forceTorqueSensorObj->ForceZNoise().Mean());
  EXPECT_DOUBLE_EQ(2.1, forceTorqueSensorObj->ForceZNoise().StdDev());

  EXPECT_DOUBLE_EQ(3.0, forceTorqueSensorObj->TorqueXNoise().Mean());
  EXPECT_DOUBLE_EQ(3.1, forceTorqueSensorObj->TorqueXNoise().StdDev());
  EXPECT_DOUBLE_EQ(4.0, forceTorqueSensorObj->TorqueYNoise().Mean());
  EXPECT_DOUBLE_EQ(4.1, forceTorqueSensorObj->TorqueYNoise().StdDev());
  EXPECT_DOUBLE_EQ(5.0, forceTorqueSensorObj->TorqueZNoise().Mean());
  EXPECT_DOUBLE_EQ(5.1, forceTorqueSensorObj->TorqueZNoise().StdDev());
}

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("save_world_test_cache"));
  return RUN_ALL_TESTS();
}
