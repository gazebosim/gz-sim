/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <string>
#include <unordered_set>
#include <vector>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/rendering/Events.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace std::chrono_literals;
namespace components = gz::sim::components;

std::unordered_set<sim::Entity> g_sensorEntityIds;
rendering::ScenePtr g_scene;

/////////////////////////////////////////////////
void OnPostRender()
{
  if (!g_scene)
  {
    g_scene = rendering::sceneFromFirstRenderEngine();
  }
  ASSERT_TRUE(g_scene);

  EXPECT_LT(0u, g_scene->SensorCount());
  for (unsigned int i = 0; i < g_scene->SensorCount(); ++i)
  {
    auto sensor = g_scene->SensorByIndex(i);
    ASSERT_TRUE(sensor);
    EXPECT_TRUE(sensor->HasUserData("gazebo-entity"));
    auto variant = sensor->UserData("gazebo-entity");

    const uint64_t *value = std::get_if<uint64_t>(&variant);
    ASSERT_TRUE(value);
    g_sensorEntityIds.insert(*value);
  }
}

/////////////////////////////////////////////////
void testSensorEntityIds(const sim::EntityComponentManager &_ecm,
    const std::unordered_set<sim::Entity> &_ids)
{
  EXPECT_FALSE(_ids.empty());
  for (const auto & id : _ids)
  {
    auto sensorComp = _ecm.Component<sim::components::Sensor>(id);
    EXPECT_TRUE(sensorComp);
  }
}

/////////////////////////////////////////////////
void testSensorTopicComponents(const sim::EntityComponentManager &_ecm,
    const std::unordered_set<sim::Entity> &_ids,
    const std::vector<std::string> &_topics)
{
  EXPECT_FALSE(_ids.empty());
  for (const auto & id : _ids)
  {
    auto sensorTopicComp = _ecm.Component<sim::components::SensorTopic>(id);
    EXPECT_TRUE(sensorTopicComp);
    std::string topicStr = "/" + sensorTopicComp->Data();
    EXPECT_FALSE(topicStr.empty());

    // verify that the topic string stored in sensor topic component
    // exits in the list of topics
    // For rendering sensors, they may advertize more than one topics but
    // the the sensor topic component will only contain one of them, e.g.
    //   * <topic_prefix>/image - stored in sensor topic component
    //   * <topic_prefix>/camera_info
    bool foundTopic = false;
    for (auto it = _topics.begin(); it != _topics.end(); ++it)
    {
      std::string topic = *it;
      if (topic.find(topicStr) == 0u)
      {
        foundTopic = true;
        break;
      }
    }
    EXPECT_TRUE(foundTopic);
  }
}

//////////////////////////////////////////////////
class SensorsFixture : public InternalFixture<InternalFixture<::testing::Test>>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetFilename("libMockSystem.so");
    sdfPlugin.SetName("gz::sim::MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
};

//////////////////////////////////////////////////
void testDefaultTopics(const std::vector<std::string> &_topics)
{
  // TODO(anyone) This should be a new test, but running multiple tests with
  // sensors is not currently working
  std::vector<transport::MessagePublisher> publishers;
  std::vector<transport::MessagePublisher> subscribers;
  transport::Node node;

  // Sensors are created in a separate thread, so we sleep here to give them
  // time
  int sleep{0};
  int maxSleep{30};
  for (; sleep < maxSleep &&
      !node.TopicInfo(_topics.front(), publishers, subscribers);
      ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_LT(sleep, maxSleep);

  for (const std::string &topic : _topics)
  {
    bool result = node.TopicInfo(topic, publishers, subscribers);

    EXPECT_TRUE(result) << "Could not get topic info for " << topic;
    EXPECT_EQ(1u, publishers.size());
    publishers.clear();
  }
}

/////////////////////////////////////////////////
/// This test checks that that the sensors system handles cases where entities
/// are removed and then added back
TEST_F(SensorsFixture, HandleRemovedEntities)
{
  gz::sim::ServerConfig serverConfig;

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/sensor.sdf";

  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  const sdf::Model *sdfModel = sdfWorld->ModelByIndex(0);

  sim::Server server(serverConfig);

  common::ConnectionPtr postRenderConn;

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    };
  this->mockSystem->configureCallback =
    [&](const sim::Entity &,
           const std::shared_ptr<const sdf::Element> &,
           sim::EntityComponentManager &,
           sim::EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<sim::events::PostRender>(
          std::bind(&::OnPostRender));
    };

  server.AddSystem(this->systemPtr);
  server.Run(true, 50, false);
  ASSERT_NE(nullptr, ecm);

  std::string prefix{"/world/camera_sensor/model/default_topics/"};
  std::vector<std::string> topics{
      prefix + "link/camera_link/sensor/camera/image",
      prefix + "link/camera_link/sensor/camera/camera_info",
      prefix + "link/gpu_lidar_link/sensor/gpu_lidar/scan",
      prefix + "link/depth_camera_link/sensor/depth_camera/depth_image",
      prefix + "link/depth_camera_link/sensor/depth_camera/camera_info",
      prefix + "link/rgbd_camera_link/sensor/rgbd_camera/image",
      prefix + "link/rgbd_camera_link/sensor/rgbd_camera/depth_image",
      prefix + "link/gpu_lidar_link/sensor/gpu_lidar/scan",
      prefix + "link/thermal_camera_link/sensor/thermal_camera/image",
      prefix + "link/segmentation_camera_link/sensor/segmentation_camera/"
             + "segmentation/colored_map",
      prefix + "link/segmentation_camera_link/sensor/segmentation_camera/"
             + "segmentation/labels_map",
      prefix + "link/segmentation_camera_link/sensor/segmentation_camera/"
             + "segmentation/camera_info",
      "/camera"
  };

  testDefaultTopics(topics);
  testSensorEntityIds(*ecm, g_sensorEntityIds);
  testSensorTopicComponents(*ecm, g_sensorEntityIds, topics);

  g_sensorEntityIds.clear();
  g_scene.reset();

  // We won't use the event manager but it's required to create an
  // SdfEntityCreator
  sim::EventManager dummyEventMgr;
  sim::SdfEntityCreator creator(*ecm, dummyEventMgr);

  unsigned int runs = 100;
  unsigned int runIterations = 2;
  for (unsigned int i = 0; i < runs; ++i)
  {
    {
      auto modelEntity = ecm->EntityByComponents(
          components::Model(), components::Name(sdfModel->Name()));
      EXPECT_NE(sim::kNullEntity, modelEntity);

      // Remove the first model in the world
      creator.RequestRemoveEntity(modelEntity, true);
    }

    server.Run(true, runIterations, false);

    {
      auto modelEntity = ecm->EntityByComponents(components::Model(),
          components::Name(sdfModel->Name()));

      // Since the model is removed, we should get a null entity
      EXPECT_EQ(sim::kNullEntity, modelEntity);
    }

    // Create the model again
    auto newModelEntity = creator.CreateEntities(sdfModel);
    creator.SetParent(newModelEntity,
                      ecm->EntityByComponents(components::World()));

    // This would throw if entities that have not been properly removed from the
    // scene manager in the Sensor system
    EXPECT_NO_THROW(server.Run(true, runIterations, false));

    {
      auto modelEntity = ecm->EntityByComponents(components::Model(),
          components::Name(sdfModel->Name()));
      EXPECT_NE(sim::kNullEntity, modelEntity);
    }
  }
}
