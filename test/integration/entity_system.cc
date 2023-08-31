/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test DiffDrive system
class EntitySystemTest : public InternalFixture<::testing::TestWithParam<int>>
{
  /// \param[in] _sdfFile SDF file to load.
  /// \param[in] _cmdVelTopic Command velocity topic.
  protected: void TestPublishCmd(const std::string &_sdfFile,
                                 const std::string &_cmdVelTopic)
  {
    // Start server
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);

    Server server(serverConfig);
    EXPECT_FALSE(server.Running());
    EXPECT_FALSE(*server.Running(0));

    Entity vehicleEntity = kNullEntity;

    // Create a system that records the vehicle poses
    test::Relay testSystem;

    std::vector<math::Pose3d> poses;
    testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
      const sim::EntityComponentManager &_ecm)
      {
        auto id = _ecm.EntityByComponents(
          components::Model(),
          components::Name("vehicle"));
        EXPECT_NE(kNullEntity, id);
        vehicleEntity = id;

        auto poseComp = _ecm.Component<components::Pose>(id);
        ASSERT_NE(nullptr, poseComp);

        poses.push_back(poseComp->Data());
      });
    server.AddSystem(testSystem.systemPtr);

    // Run server and check that vehicle didn't move
    server.Run(true, 1000, false);
    EXPECT_EQ(1000u, poses.size());

    for (const auto &pose : poses)
    {
      EXPECT_EQ(poses[0], pose);
    }
    poses.clear();

    // Publish command and check that vehicle still does not move
    transport::Node node;
    auto pub = node.Advertise<msgs::Twist>(_cmdVelTopic);
    msgs::Twist msg;
    const double desiredLinVel = 10.5;
    msgs::Set(msg.mutable_linear(),
              math::Vector3d(desiredLinVel, 0, 0));
    msgs::Set(msg.mutable_angular(),
              math::Vector3d(0.0, 0, 0));
    pub.Publish(msg);
    server.Run(true, 1000, false);
    for (const auto &pose : poses)
    {
      EXPECT_EQ(poses[0], pose);
    }
    poses.clear();

    // send request to add diff_drive system
    EXPECT_NE(kNullEntity, vehicleEntity);
    msgs::EntityPlugin_V req;
    auto ent = req.mutable_entity();
    ent->set_id(vehicleEntity);
    auto plugin = req.add_plugins();
    plugin->set_name("gz::sim::systems::DiffDrive");
    plugin->set_filename("gz-sim-diff-drive-system");
    std::stringstream innerxml;
    innerxml
        << "<left_joint>left_wheel_joint</left_joint>\n"
        << "<right_joint>right_wheel_joint</right_joint>\n"
        << "<wheel_separation>1.25</wheel_separation>\n"
        << "<wheel_radius>0.3</wheel_radius>\n"
        << "<max_linear_acceleration>1</max_linear_acceleration>\n"
        << "<min_linear_acceleration>-1</min_linear_acceleration>\n"
        << "<max_angular_acceleration>2</max_angular_acceleration>\n"
        << "<min_angular_acceleration>-2</min_angular_acceleration>\n"
        << "<max_linear_velocity>0.5</max_linear_velocity>\n"
        << "<min_linear_velocity>-0.5</min_linear_velocity>\n"
        << "<max_angular_velocity>1</max_angular_velocity>\n"
        << "<min_angular_velocity>-1</min_angular_velocity>\n";
    plugin->set_innerxml(innerxml.str());

    msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/diff_drive/entity/system/add"};

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    // run once for the system to be added
    server.Run(true, 1, false);
    poses.clear();

    // publish twist msg and verify that the vehicle now moves forward
    pub.Publish(msg);
    server.Run(true, 1000, false);
    for (unsigned int i = 1; i < poses.size(); ++i)
    {
      EXPECT_GT(poses[i].Pos().X(), poses[i-1].Pos().X());
    }
  }
};

/////////////////////////////////////////////////
// See https://github.com/simsim/gz-sim/issues/1175
TEST_P(EntitySystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PublishCmd))
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/diff_drive_no_plugin.sdf",
      "/model/vehicle/cmd_vel");
}

/////////////////////////////////////////////////
TEST_P(EntitySystemTest, SystemInfo)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/empty.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // get info on systems available
  msgs::Empty req;
  msgs::EntityPlugin_V res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/empty/system/info"};
  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);

  // verify plugins are not empty
  EXPECT_FALSE(res.plugins().empty());

  // check for a few known plugins that we know should exist in gazebo
  std::set<std::string> knownPlugins;
  knownPlugins.insert("user-commands-system");
  knownPlugins.insert("physics-system");
  knownPlugins.insert("scene-broadcaster-system");
  knownPlugins.insert("sensors-system");

  for (const auto &plugin : res.plugins())
  {
    for (const auto &kp : knownPlugins)
    {
      if (plugin.filename().find(kp) != std::string::npos)
      {
        knownPlugins.erase(kp);
        break;
      }
    }
  }
  // verify all known plugins are found and removed from the set
  EXPECT_TRUE(knownPlugins.empty());
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, EntitySystemTest,
    ::testing::Range(1, 2));
