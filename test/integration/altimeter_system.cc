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

#include <ignition/msgs/altimeter.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test AltimeterTest system
class AltimeterTest : public ::testing::Test
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


std::vector<msgs::Altimeter> altMsgs;

/////////////////////////////////////////////////
void altimeterCb(const msgs::Altimeter &_msg)
{
  altMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling altimeter
TEST_F(AltimeterTest, ModelFalling)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/altimeter.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "altimeter_sensor";

  // Create a system that records altimeter data
  Relay testSystem;
  std::vector<math::Pose3d> poses;
  std::vector<math::Vector3d> velocities;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Altimeter, components::Name,
                  components::WorldPose,
                  components::WorldLinearVelocity>(
            [&](const ignition::gazebo::Entity &,
                const components::Altimeter *,
                const components::Name *_name,
                const components::WorldPose *_worldPose,
                const components::WorldLinearVelocity *_worldLinearVel) -> bool
            {
              EXPECT_EQ(_name->Data(), sensorName);

              poses.push_back(_worldPose->Data());
              velocities.push_back(_worldLinearVel->Data());

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to altimeter topic
  transport::Node node;
  node.Subscribe(
      "/model/altimeter_model/link/link/sensor/altimeter_sensor/altimeter",
      &altimeterCb);

  // Run server
  size_t iters100 = 100u;
  server.Run(true, iters100, false);
  EXPECT_EQ(iters100, poses.size());

  // check altimeter world pose
  // verify the altimeter model z pos is decreasing
  EXPECT_GT(poses.front().Pos().Z(), poses.back().Pos().Z());
  EXPECT_GT(poses.back().Pos().Z(), 0.0);
  // velocity should be negative in z
  EXPECT_GT(velocities.front().Z(), velocities.back().Z());
  EXPECT_LT(velocities.back().Z(), 0.0);

  // check altimeter sensor data
  // vertical position = world position - intial position
  // so since altimeter is falling, vertical position should be negative
  EXPECT_GT(altMsgs.front().vertical_position(),
      altMsgs.back().vertical_position());
  EXPECT_LT(altMsgs.back().vertical_position(), 0.0);
  // vertical velocity should be negative
  EXPECT_GT(altMsgs.front().vertical_velocity(),
      altMsgs.back().vertical_velocity());
  EXPECT_LT(altMsgs.back().vertical_velocity(), 0.0);

  // Run server for longer period of time so the altimeter falls then rests
  // on the ground plane
  size_t iters1000 = 1000u;
  server.Run(true, iters1000, false);
  EXPECT_EQ(iters100 + iters1000, poses.size());

  // check altimeter world pose
  // altimeter should be on the ground
  // note that altimeter's parent link is at an 0.05 offset
  // set high tol due to instablity
  EXPECT_NEAR(poses.back().Pos().Z(), 0.05, 1e-2);
  // velocity should now be zero as the model is resting on the ground
  EXPECT_NEAR(velocities.back().Z(), 0u, 1e-3);

  // check altimeter sensor data
  // altimeter vertical position = 0.05 (world pos) - 3.05 (initial position)
  EXPECT_LT(altMsgs.back().vertical_position(), -3.0);
  // velocity should be zero
  EXPECT_NEAR(altMsgs.back().vertical_velocity(), 0u, 1e-3);
}
