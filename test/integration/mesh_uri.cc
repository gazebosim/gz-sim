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

#include <gz/msgs/entity_factory.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

std::mutex mutex;
int g_count = 0;
msgs::Image g_image;

/////////////////////////////////////////////////
void cameraCb(const msgs::Image & _msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  std::lock_guard<std::mutex> lock(mutex);
  g_image = _msg;
  g_count++;
}

std::string meshModelStr(bool _static = false)
{
  // SDF string consisting of a box mesh. Note that "unit_box"
  // is a mesh that comes pre-registered with MeshManager so
  // it should be able to load this as a mesh.
  // Similarly the user can create a common::Mesh object and add that
  // to the MeshManager by calling MeshManager::Instance()->AddMesh;
  return std::string("<?xml version=\"1.0\" ?>") +
      "<sdf version='1.6'>" +
      "<model name='spawned_model'>" +
      "<link name='link'>" +
      "<visual name='visual'>" +
      "<geometry><mesh><uri>name://unit_box</uri></mesh></geometry>" +
      "</visual>" +
      "<collision name='visual'>" +
      "<geometry><mesh><uri>name://unit_box</uri></mesh></geometry>" +
      "</collision>" +
      "</link>" +
      "<static>" + std::to_string(_static) + "</static>" +
      "</model>" +
      "</sdf>";
}

//////////////////////////////////////////////////
class MeshUriTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// Test spawning mesh by name and verify that the mesh is correctly
// spawned in both rendering and physics. Cameras should see the spawned
// mesh and spawend object should collide with one another on the physics
// side.
TEST_F(MeshUriTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnMeshByName))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "camera_sensor_scene_background.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  auto entityCount = ecm->EntityCount();

  // Subscribe to the camera topic
  transport::Node node;
  g_count = 0;
  node.Subscribe("/camera", &cameraCb);

  // Run and make sure we received camera images
  server.Run(true, 100, false);
  for (int i = 0; i < 100 && g_count <= 0; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_LT(0u, g_count);

  unsigned char *imageBuffer = nullptr;
  unsigned int midIdx = 0u;
  unsigned int bufferSize = 0u;
  // Empty scene - camera should see just red background
  {
    std::lock_guard<std::mutex> lock(mutex);
    bufferSize = g_image.width() *g_image.height() * 3u;
    imageBuffer = new unsigned char[bufferSize];
    memcpy(imageBuffer, g_image.data().c_str(), bufferSize);

    midIdx = (g_image.height() * 0.5 * g_image.width()  +
      g_image.width() * 0.5) * 3;
  }
  unsigned int r = imageBuffer[midIdx];
  unsigned int g = imageBuffer[midIdx + 1];
  unsigned int b = imageBuffer[midIdx + 2];
  EXPECT_EQ(255u, r);
  EXPECT_EQ(0u, g);
  EXPECT_EQ(0u, b);

  // Spawn a static box (mesh) model in front of the camera
  msgs::EntityFactory req;
  req.set_sdf(meshModelStr(true));

  auto pose = req.mutable_pose();
  auto pos = pose->mutable_position();
  pos->set_x(3);
  pos->set_z(1);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/sensors/create"};

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been created yet
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("spawned_model")));

  // Run an iteration and check it was created
  server.Run(true, 1, false);
  EXPECT_EQ(entityCount + 4, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  auto model = ecm->EntityByComponents(components::Model(),
      components::Name("spawned_model"));
  EXPECT_NE(kNullEntity, model);
  auto poseComp = ecm->Component<components::Pose>(model);
  EXPECT_NE(nullptr, poseComp);
  EXPECT_EQ(math::Pose3d(3, 0, 1, 0, 0, 0), poseComp->Data());

  // Verify camera now sees a white box
  g_count = 0u;
  server.Run(true, 100, false);
  for (int i = 0; i < 100 && g_count <= 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_LT(0u, g_count);

  {
    std::lock_guard<std::mutex> lock(mutex);
    memcpy(imageBuffer, g_image.data().c_str(), bufferSize);
  }
  r = imageBuffer[midIdx];
  g = imageBuffer[midIdx + 1];
  b = imageBuffer[midIdx + 2];
  EXPECT_EQ(255u, r);
  EXPECT_EQ(255u, g);
  EXPECT_EQ(255u, b);

  // Spawn another box over the static one
  const std::string spawnedModel2Name = "spawned_model2";
  req.set_sdf(meshModelStr());
  req.set_name(spawnedModel2Name);
  pose = req.mutable_pose();
  pos = pose->mutable_position();
  pos->set_x(3);
  pos->set_z(5);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been created yet
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name(spawnedModel2Name)));

  // Run an iteration and check it was created
  server.Run(true, 1, false);
  EXPECT_EQ(entityCount + 4, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  auto model2 = ecm->EntityByComponents(components::Model(),
      components::Name(spawnedModel2Name));
  EXPECT_NE(kNullEntity, model2);

  poseComp = ecm->Component<components::Pose>(model2);
  EXPECT_NE(nullptr, poseComp);
  EXPECT_EQ(math::Pose3d(3, 0, 5, 0, 0, 0), poseComp->Data());

  // run and spawned model 2 should fall onto the previous box
  server.Run(true, 1000, false);
  poseComp = ecm->Component<components::Pose>(model2);
  EXPECT_NE(nullptr, poseComp);
  EXPECT_EQ(math::Pose3d(3, 0, 2, 0, 0, 0), poseComp->Data()) <<
    poseComp->Data();

  delete [] imageBuffer;
  imageBuffer = nullptr;
}
