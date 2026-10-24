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

#include "CameraVideoRecorder.hh"

#include <string>

#include <gtest/gtest.h>

#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>
#include <sdf/World.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Sensor.hh"

#include "helpers/UnitTestUtil.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Build an SDF world with a camera sensor plugin
/// \param[in] _ns Model namespace. Can be empty.
/// \param[in] _topic Camera sensor topic. Can be empty.
/// \param[in] _service Video recorder service. Can be empty.
/// \return SDF world string
std::string MakeSdf(const std::string &_ns, const std::string &_topic,
    const std::string &_service)
{
  const std::string ns = _ns.empty() ? "" : " namespace='" + _ns + "'";
  const std::string topic = _topic.empty() ? "" :
      "<topic>" + _topic + "</topic>";
  const std::string service = _service.empty() ? "" :
      "<service>" + _service + "</service>";

  return R"(<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="camera_video_recorder_test_world">
    <model name="test_model")" + ns + R"(>
      <link name="test_link">
        <sensor name="camera" type="camera">
          )" + topic + R"(
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>8</width>
              <height>8</height>
            </image>
            <clip>
              <near>0.1</near>
              <far>10</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <plugin filename="gz-sim-camera-video-recorder-system"
                  name="gz::sim::systems::CameraVideoRecorder">
            )" + service + R"(
          </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>)";
}

/////////////////////////////////////////////////
/// \brief Load a camera sensor plugin from an SDF string
/// \param[in] _sdfString SDF world string
/// \param[in] _ecm Entity component manager
/// \param[in] _eventMgr Event manager
/// \param[out] _entity Camera sensor entity
/// \param[out] _pluginSdf Camera sensor plugin SDF
void LoadCameraSensorContext(const std::string &_sdfString,
    EntityComponentManager &_ecm, EventManager &_eventMgr, Entity &_entity,
    sdf::Plugin &_pluginSdf)
{
  sdf::Root root;
  root.LoadSdfString(_sdfString);

  ASSERT_EQ(1u, root.WorldCount());
  const sdf::World *worldSdf = root.WorldByIndex(0);
  ASSERT_NE(nullptr, worldSdf);

  ASSERT_EQ(1u, worldSdf->ModelCount());
  const sdf::Model *modelSdf = worldSdf->ModelByIndex(0);
  ASSERT_NE(nullptr, modelSdf);

  ASSERT_EQ(1u, modelSdf->LinkCount());
  const sdf::Link *linkSdf = modelSdf->LinkByIndex(0);
  ASSERT_NE(nullptr, linkSdf);

  ASSERT_EQ(1u, linkSdf->SensorCount());
  const sdf::Sensor *sensorSdf = linkSdf->SensorByIndex(0);
  ASSERT_NE(nullptr, sensorSdf);

  ASSERT_FALSE(sensorSdf->Plugins().empty());
  _pluginSdf = sensorSdf->Plugins()[0];

  SdfEntityCreator entityCreator(_ecm, _eventMgr);
  ASSERT_NE(kNullEntity,
      entityCreator.CreateEntitiesWithoutLoadingPlugins(modelSdf));

  _entity = _ecm.EntityByComponents(components::Sensor(),
      components::Name("camera"));
  ASSERT_NE(kNullEntity, _entity);
}

/////////////////////////////////////////////////
/// \brief Configure a CameraVideoRecorder system and check resolved names
/// \param[in] _sdfString SDF world string
/// \param[in] _expected Expected resolved topic and service names
void TestTopicNames(const std::string &_sdfString,
    const CameraVideoRecorder::TopicNames &_expected)
{
  EntityComponentManager ecm;
  EventManager eventMgr;
  Entity entity = kNullEntity;
  sdf::Plugin pluginSdf;

  LoadCameraSensorContext(_sdfString, ecm, eventMgr, entity, pluginSdf);

  CameraVideoRecorder system;
  system.Configure(entity, pluginSdf.Element(), ecm, eventMgr);

  const auto names = system.ResolvedTopicNames();
  EXPECT_EQ(_expected.service, names.service);
  EXPECT_EQ(_expected.sensorTopic, names.sensorTopic);
}

/////////////////////////////////////////////////
TEST(CameraVideoRecorderTest, AbsoluteTopicsWithNamespace)
{
  CameraVideoRecorder::TopicNames expected;
  expected.service = "/absolute/record_video";
  expected.sensorTopic = "/absolute/image";

  TestTopicNames(MakeSdf("ns", "/absolute/image", "/absolute/record_video"),
      expected);
}

/////////////////////////////////////////////////
TEST(CameraVideoRecorderTest, RelativeTopicsWithNamespace)
{
  CameraVideoRecorder::TopicNames expected;
  expected.service = "ns/test_record_video";
  expected.sensorTopic = "ns/test_image";

  TestTopicNames(MakeSdf("ns", "test_image", "test_record_video"), expected);
}

/////////////////////////////////////////////////
TEST(CameraVideoRecorderTest, DefaultTopicsWithNamespace)
{
  CameraVideoRecorder::TopicNames expected;
  expected.service = "ns/record_video";
  expected.sensorTopic = "ns/image";

  TestTopicNames(MakeSdf("ns", "", ""), expected);
}

/////////////////////////////////////////////////
TEST(CameraVideoRecorderTest, RelativeTopicsWithoutNamespace)
{
  CameraVideoRecorder::TopicNames expected;
  expected.service = "test_record_video";
  expected.sensorTopic = "test_image";

  TestTopicNames(MakeSdf("", "test_image", "test_record_video"), expected);
}

/////////////////////////////////////////////////
TEST(CameraVideoRecorderTest, DefaultTopicsWithoutNamespace)
{
  CameraVideoRecorder::TopicNames expected;
  expected.service =
      "model/test_model/link/test_link/sensor/camera/record_video";
  expected.sensorTopic = "model/test_model/link/test_link/sensor/camera/image";

  TestTopicNames(MakeSdf("", "", ""), expected);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
