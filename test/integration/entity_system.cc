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
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;
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
    testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
      const gazebo::EntityComponentManager &_ecm)
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
    plugin->set_name("ignition::gazebo::systems::DiffDrive");
    plugin->set_filename("ignition-gazebo-diff-drive-system");
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
    std::string service{"/entity/system/add"};

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
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(EntitySystemTest, IGN_UTILS_TEST_DISABLED_ON_WIN32(PublishCmd))
{
  TestPublishCmd(
      std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/diff_drive_no_plugin.sdf",
      "/model/vehicle/cmd_vel");
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, EntitySystemTest,
    ::testing::Range(1, 2));
