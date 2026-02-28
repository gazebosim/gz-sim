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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/convert/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test TouchPlugin system
class TouchPluginTest : public InternalFixture<::testing::Test>
{
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) + _sdfFile);
    server = std::make_unique<Server>(serverConfig);
    using namespace std::chrono_literals;
    server->SetUpdatePeriod(0ns);

    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));
  }

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(TouchPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(OneLink))
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool whiteTouched{false};
  auto whiteTouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        whiteTouched = true;
      });

  transport::Node node;
  node.Subscribe("/white_touches_only_green/touched", whiteTouchCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(whiteTouched);

  // Let white box fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(whiteTouched);

  // Give it time to touch for 3 seconds
  server->Run(true, 3100, false);

  // Check it has touched for long enough
  EXPECT_TRUE(whiteTouched);

  // Wait more and check it doesn't notify again
  whiteTouched = false;
  server->Run(true, 3100, false);
  EXPECT_FALSE(whiteTouched);

  // Enable plugin again
  msgs::Boolean req;
  req.set_data(true);
  bool executed = node.Request("/white_touches_only_green/enable", req);

  EXPECT_TRUE(executed);

  // Wait and see it notifies again
  whiteTouched = false;
  server->Run(true, 3100, false);

  // TODO(anyone) Not fair to only wait for "true"
  for (int sleep = 0; sleep < 50 && !whiteTouched; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
// Known to fail on OSX, see
// https://github.com/gazebosim/gz-sim/issues/22
#if !defined (__APPLE__)
  EXPECT_TRUE(whiteTouched);
#endif
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(MultiLink))
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool redTouched{false};
  auto redTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        redTouched = true;
      });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/red_and_yellow_touch_only_green/touched", redTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(redTouched);

  // Let red and yellow boxes fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(redTouched);

  // Give it time to touch for 2 seconds
  server->Run(true, 2100, false);

  // Check it has touched for long enough
  EXPECT_TRUE(redTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(StartDisabled))
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  // Subscribe to plugin notifications
  bool blueTouched{false};
  auto blueTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        blueTouched = true;
      });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/blue_touches_only_green/touched", blueTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(blueTouched);

  // Let the box fall on top of green box and touch for a while
  server->Run(true, 3000, false);

  // Verify we don't get a notification
  EXPECT_FALSE(blueTouched);

  // Enable plugin
  msgs::Boolean req;
  req.set_data(true);
  bool executed = node.Request("/blue_touches_only_green/enable", req);

  EXPECT_TRUE(executed);

  // Wait and see it notifies now
  blueTouched = false;
  server->Run(true, 3000, false);

  for (int sleep = 0; sleep < 50 && !blueTouched; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
// Known to fail on OSX, see
// https://github.com/gazebosim/gz-sim/issues/22
#if !defined (__APPLE__)
  EXPECT_TRUE(blueTouched);
#endif
}

//////////////////////////////////////////////////
// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(TouchPluginTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(RemovalOfParentModel))
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool redTouched{false};
  auto redTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        redTouched = true;
      });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/red_and_yellow_touch_only_green/touched", redTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(redTouched);

  // Let red and yellow boxes fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(redTouched);

  // Remove the model containing the touch plugin
  server->RequestRemoveEntity("red_yellow_box");

  // Give it time to touch for 2 seconds
  server->Run(true, 2100, false);

  // Check that we don't get a touched event
  EXPECT_FALSE(redTouched);
}


//////////////////////////////////////////////////
/// Tests whether the plugin works when it is spawned after other entities have
/// already been created and vice versa
/// This test uses depends on the user_commands system
TEST_F(TouchPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnedEntities))
{
  std::string whiteBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.6">
      <model name="white_box">
        <pose>0 0 4 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.5 0.5 0.5</size>
              </box>
            </geometry>
          </collision>
          <sensor name="white_box_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
        <plugin
          filename="gz-sim-touchplugin-system"
          name="gz::sim::systems::TouchPlugin">
          <target>green_box_for_white</target>
          <time>0.2</time>
          <namespace>white_touches_only_green</namespace>
          <enabled>true</enabled>
        </plugin>
      </model>
  </sdf>)EOF";

  std::string greenBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.6">
      <model name="green_box_for_white">
        <static>1</static>
        <pose>0 0 0.5 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
  </sdf>)EOF";

  transport::Node node;

  bool whiteTouched{false};
  auto whiteTouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        whiteTouched = true;
      });

  node.Subscribe("/white_touches_only_green/touched", whiteTouchCb);

  // Request entity spawn
  msgs::EntityFactory req;
  unsigned int timeout = 5000;
  std::string service{"/world/empty/create"};

  msgs::Boolean res;
  bool result;

  auto testFunc = [&](const std::string &_box1, const std::string &_box2)
  {
    this->server.reset();
    this->StartServer("/test/worlds/empty.sdf");

    whiteTouched = false;
    req.set_sdf(_box1);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    // Run the server to actually create the entities
    server->Run(true, 100, false);

    req.set_sdf(_box2);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 100, false);

    // Check boxes haven't touched yet
    EXPECT_FALSE(whiteTouched);

    // Let white box fall on top of green box
    server->Run(true, 500, false);

    // Check it hasn't touched for long enough
    EXPECT_FALSE(whiteTouched);

    // Give it time to touch for at least 0.2 seconds
    server->Run(true, 500, false);

    // TODO(anyone) Not fair to only wait for "true"
    for (int sleep = 0; sleep < 50 && !whiteTouched; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    EXPECT_TRUE(whiteTouched);
  };

  // Test different spawn orders
  testFunc(whiteBox, greenBox);
  testFunc(greenBox, whiteBox);
}

TEST_F(TouchPluginTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(CollisionFilter))
{
  // Spawn white box over one of the 2 collisions in the green box.
  // There are 3 touch plugins in the green box, and each one specifies a
  // different collision. One of the touch plugin specifies a collision
  // that does not exist so it should not trigger any events.
  // Drop the white box over one of the collisions in the
  // green box and verify that the correct touch event is published. Repeat
  // and drop the white box onto the other collision.
  std::string whiteBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.9">
      <model name="white_box">
        <pose>0 0 8 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.5 0.5 0.5</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
  </sdf>)EOF";

  std::string greenBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.9">
      <model name="green_box_for_white">
        <static>1</static>
        <pose>0 0 0.5 0 0 0</pose>
        <link name="link">
          <collision name="collision_01">
            <pose>0 1 0 0 0 0</pose>
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <collision name="collision_02">
            <pose>0 -1 0 0 0 0</pose>
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
          <sensor name="collision_sensor" type="contact">
            <contact>
              <collision>collision_01</collision>
              <collision>collision_02</collision>
            </contact>
          </sensor>
        </link>
        <plugin
          filename="gz-sim-touchplugin-system"
          name="gz::sim::systems::TouchPlugin">
          <target>white_box</target>
          <collision>collision_01</collision>
          <time>0.2</time>
          <namespace>white_touches_green_collision_01</namespace>
          <enabled>true</enabled>
        </plugin>
        <plugin
          filename="gz-sim-touchplugin-system"
          name="gz::sim::systems::TouchPlugin">
          <target>white_box</target>
          <collision>collision_02</collision>
          <time>0.2</time>
          <namespace>white_touches_green_collision_02</namespace>
          <enabled>true</enabled>
        </plugin>
        <plugin
          filename="gz-sim-touchplugin-system"
          name="gz::sim::systems::TouchPlugin">
          <target>white_box</target>
          <collision>collision_03</collision>
          <time>0.2</time>
          <namespace>white_touches_green_collision_03</namespace>
          <enabled>true</enabled>
        </plugin>
      </model>
  </sdf>)EOF";

  transport::Node node;

  bool col01Touched{false};
  auto col01TouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        col01Touched = true;
      });
  node.Subscribe("/white_touches_green_collision_01/touched",
                 col01TouchCb);

  bool col02Touched{false};
  auto col02TouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        col02Touched = true;
      });
  node.Subscribe("/white_touches_green_collision_02/touched",
                 col02TouchCb);

  // Request entity spawn
  msgs::EntityFactory req;
  unsigned int timeout = 5000;
  std::string service{"/world/empty/create"};
  std::string setPoseService{"/world/empty/set_pose"};
  std::string col01EnableService{"/white_touches_green_collision_01/enable"};
  std::string col02EnableService{"/white_touches_green_collision_02/enable"};

  msgs::Boolean res;
  bool result;

  this->server.reset();
  this->StartServer("/test/worlds/empty.sdf");

  req.set_sdf(greenBox);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server->Run(true, 100, false);

  req.set_sdf(whiteBox);
  gz::msgs::Set(req.mutable_pose(), math::Pose3d(0, 0, 8, 0, 0, 0));
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());
  server->Run(true, 100, false);

  // Verify that the /white_touches_green_collision_03/touched service is not
  // available because the contact sensor collision, collision_03, does not
  // exist
  std::vector<std::string> serviceList;
  node.ServiceList(serviceList);
  EXPECT_EQ(std::find(serviceList.begin(), serviceList.end(),
                      "/white_touches_green_collision_03/touched"),
            serviceList.end());

  auto testFunc = [&](const math::Pose3d &_pose)
  {
    col01Touched = false;
    col02Touched = false;

    // Enable the touch plugins
    msgs::Empty emptyRes;
    msgs::Boolean enableReq;
    enableReq.set_data(true);
    EXPECT_TRUE(node.Request(col01EnableService, enableReq, timeout, emptyRes,
                             result));
    EXPECT_TRUE(node.Request(col02EnableService, enableReq, timeout, emptyRes,
                             result));

    // Process the service calls
    server->Run(true, 100, false);

    // Spawn white box at specified box above collision
    msgs::Pose poseReq;
    poseReq.set_name("white_box");
    msgs::Set(poseReq.mutable_position(), _pose.Pos());
    msgs::Set(poseReq.mutable_orientation(), _pose.Rot());
    EXPECT_TRUE(node.Request(setPoseService, poseReq, timeout, res, result));

    // Check boxes haven't touched yet
    EXPECT_FALSE(col01Touched);
    EXPECT_FALSE(col02Touched);

    // Let white box fall on top of green box at the specified pose
    server->Run(true, 500, false);

    // Check it hasn't touched for long enough
    EXPECT_FALSE(col01Touched);
    EXPECT_FALSE(col02Touched);

    // Give it time to touch for at least 0.2 seconds
    server->Run(true, 1000, false);

    // Only one touched event should be received
    if (_pose.Pos().Y() > 0.0)
    {
      for (int sleep = 0; sleep < 50 && !col01Touched; ++sleep)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      EXPECT_TRUE(col01Touched);
      EXPECT_FALSE(col02Touched);
    }
    else
    {
      for (int sleep = 0; sleep < 50 && !col02Touched; ++sleep)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      EXPECT_FALSE(col01Touched);
      EXPECT_TRUE(col02Touched);
    }
  };

  math::Pose3d spawnPoseOverCol01(0, 1.0, 8, 0, 0, 0);
  math::Pose3d spawnPoseOverCol02(0, -1.0, 8, 0, 0, 0);
  testFunc(spawnPoseOverCol01);
  testFunc(spawnPoseOverCol02);
}
