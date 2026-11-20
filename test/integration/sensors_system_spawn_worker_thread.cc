/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include <gz/msgs/image.pb.h>

#include <chrono>
#include <string>

#include <gz/transport/Node.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Subscription.hh"

using namespace gz;
using namespace std::chrono_literals;

//////////////////////////////////////////////////
class SensorsSpawnWorkerThreadFixture :
  public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Verify that spawning a rendering sensor dynamically while Server::Run
// executes on a worker thread works when Server was constructed on the main
// thread (critical on macOS where rendering context init must happen on the
// main thread).
TEST_F(SensorsSpawnWorkerThreadFixture, SpawnCameraOnWorkerThread)
{
  const std::string worldSdf = R"(
    <?xml version="1.0" ?>
    <sdf version="1.9">
      <world name="spawn_camera_worker_thread">
        <gz:policies>
          <include_server_config_plugins>false</include_server_config_plugins>
        </gz:policies>
        <plugin filename="gz-sim-user-commands-system"
                name="gz::sim::systems::UserCommands"/>
        <plugin filename="gz-sim-sensors-system"
                name="gz::sim::systems::Sensors">
          <render_engine>ogre2</render_engine>
        </plugin>
        <light type="directional" name="sun">
          <direction>-0.5 0.1 -0.9</direction>
          <diffuse>0.8 0.8 0.8 1</diffuse>
        </light>
      </world>
    </sdf>
  )";

  const std::string cameraModelSdf = R"(
    <?xml version="1.0" ?>
    <sdf version="1.9">
      <model name="spawned_camera_model">
        <static>true</static>
        <link name="camera_link">
          <sensor name="camera" type="camera">
            <camera>
              <horizontal_fov>1.047</horizontal_fov>
              <image>
                <width>64</width>
                <height>64</height>
              </image>
              <clip>
                <near>0.1</near>
                <far>100</far>
              </clip>
            </camera>
            <always_on>1</always_on>
            <update_rate>30</update_rate>
            <topic>/spawned_camera</topic>
          </sensor>
        </link>
      </model>
    </sdf>
  )";

  // Construct Server and run one paused iteration on the main thread
  sim::ServerConfig serverConfig;
  serverConfig.SetSdfString(worldSdf);

  sim::Server server(serverConfig);
  EXPECT_TRUE(server.RunOnce(true));

  transport::Node node;
  Subscription<msgs::Image> imageSub;
  imageSub.Subscribe(node, "/spawned_camera", 1u);

  // Run the simulation loop on a worker thread
  ASSERT_TRUE(server.Run(false /*blocking*/, 0, false /*paused*/));

  // Dynamically spawn a camera model while Server::Run executes off main thread
  msgs::EntityFactory req;
  req.set_sdf(cameraModelSdf);
  msgs::Boolean rep;
  bool result = false;
  ASSERT_TRUE(node.Request("/world/spawn_camera_worker_thread/create",
      req, 5000, rep, result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  // Wait for at least one camera image to be published
  ASSERT_TRUE(imageSub.WaitForMessages(1u, 15s));
  auto msg = imageSub.ReadLastMessage();
  EXPECT_EQ(64u, msg.width());
  EXPECT_EQ(64u, msg.height());

  server.Stop();
}
