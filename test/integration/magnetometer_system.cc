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

#include <ignition/msgs/magnetometer.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test MagnetometerTest system
class MagnetometerTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();
    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};


std::vector<msgs::Magnetometer> magnetometerMsgs;

/////////////////////////////////////////////////
void magnetometerCb(const msgs::Magnetometer &_msg)
{
  magnetometerMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
// The test checks the detected field from a rotated magnetometer
TEST_F(MagnetometerTest, RotatedMagnetometer)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/magnetometer.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "magnetometer_sensor";

  // Create a system that records magnetometer data
  Relay testSystem;
  math::Vector3d worldMagneticField;
  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Magnetometer,
                  components::Name,
                  components::WorldPose,
                  components::MagneticField>(
            [&](const ignition::gazebo::Entity &,
                const components::Magnetometer *,
                const components::Name *_name,
                const components::WorldPose *_worldPose,
                const components::MagneticField *_magneticField) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);
              worldMagneticField = _magneticField->Data();
              poses.push_back(_worldPose->Data());

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to magnetometer topic
  transport::Node node;
  node.Subscribe(
      "/model/magnetometer_sensor/model/magnetometer_model/"
      "link/link/sensor/magnetometer_sensor/magnetometer",
      &magnetometerCb);

  // step world and verify magnetometer's detected field
  // Run server
  size_t iters = 10u;
  server.Run(true, iters, false);
  EXPECT_EQ(iters, poses.size());

  ignition::math::Vector3d field = poses.back().Rot().Inverse().RotateVector(
        worldMagneticField);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->x(),
      field.X(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->y(),
      field.Y(), TOL);
  EXPECT_NEAR(magnetometerMsgs.back().mutable_field_tesla()->z(),
      field.Z(), TOL);
}
