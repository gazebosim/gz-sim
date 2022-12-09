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

#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/LogicalAudio.hh"
#include "gz/sim/components/Pose.hh"
#include "test_config.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/Types.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test LogicalAudio system plugin
class LogicalAudioTest : public InternalFixture<::testing::Test>
{
};

// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(LogicalAudioTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LogicalAudioDetections))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/logical_audio_sensor_plugin.sdf";
  serverConfig.SetSdfFile(sdfFile);

  // start server
  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // helper variables for checking the validity of the ECM
  const math::Pose3d sourcePose(0, 0, 0, 0, 0, 0);
  const auto zeroSeconds = std::chrono::seconds(0);
  const math::Pose3d micClosePose(0.5, 0, 0, 0, 0, 0);
  const math::Pose3d micFarPose(0, 0, 0, 0, 0, 0);
  std::chrono::steady_clock::duration sourceStartTime;
  bool firstTime{true};

  // flags that verify the ECM was checked for the source and microphones
  bool checkedSource{false};
  bool checkedMicClose{false};
  bool checkedMicFar{false};

  // make a test system and check the ECM for the source and microphones
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const UpdateInfo &_info,
                             EntityComponentManager &/*_ecm*/)
      {
        if (firstTime)
          sourceStartTime = _info.simTime;
        firstTime = false;
      });
  testSystem.OnPostUpdate([&](const UpdateInfo &/*_info*/,
                              const EntityComponentManager &_ecm)
      {
        // make sure the source is stored correctly in the ECM
        _ecm.Each<components::LogicalAudioSource,
                  components::LogicalAudioSourcePlayInfo,
                  components::Pose>(
          [&](const Entity &/*_entity*/,
              const components::LogicalAudioSource *_source,
              const components::LogicalAudioSourcePlayInfo *_playInfo,
              const components::Pose *_pose)
          {
            EXPECT_EQ(_source->Data().id, 1u);
            EXPECT_EQ(_source->Data().attFunc,
                logical_audio::AttenuationFunction::LINEAR);
            EXPECT_EQ(_source->Data().attShape,
                logical_audio::AttenuationShape::SPHERE);
            EXPECT_DOUBLE_EQ(_source->Data().innerRadius, 3.0);
            EXPECT_DOUBLE_EQ(_source->Data().falloffDistance, 8.0);
            EXPECT_DOUBLE_EQ(_source->Data().emissionVolume, 0.9);

            EXPECT_TRUE(_playInfo->Data().playing);
            EXPECT_EQ(_playInfo->Data().playDuration, zeroSeconds);
            EXPECT_EQ(_playInfo->Data().startTime, sourceStartTime);

            EXPECT_EQ(_pose->Data(), sourcePose);

            checkedSource = true;
            return true;
          });

        // make sure the microphones are stored correctly in the ECM
        _ecm.Each<components::LogicalMicrophone,
                  components::Pose>(
          [&](const Entity &/*_entity*/,
              const components::LogicalMicrophone *_mic,
              const components::Pose *_pose)
          {
            if (_mic->Data().id == 2u)
            {
              EXPECT_EQ(_mic->Data().id, 2u);
              EXPECT_DOUBLE_EQ(_mic->Data().volumeDetectionThreshold, 0.1);
              EXPECT_EQ(_pose->Data(), micClosePose);
              checkedMicClose = true;
            }
            else if (_mic->Data().id == 1u)
            {
              EXPECT_EQ(_mic->Data().id, 1u);
              EXPECT_DOUBLE_EQ(_mic->Data().volumeDetectionThreshold, 0.0);
              EXPECT_EQ(_pose->Data(), micFarPose);
              checkedMicFar = true;
            }
            return true;
          });
      });
  server.AddSystem(testSystem.systemPtr);

  // subscribe to the close microphone's detection topic
  const std::string closeTopic =
    "/model/mic_model_close/sensor/mic_2/detection";
  bool receivedClose{false};
  msgs::Double msg;
  msg.Clear();
  std::function<void(const msgs::Double &)> closeCb =
      [&receivedClose, &msg](const msgs::Double &_msg)
      {
        // only need one message
        if (receivedClose)
          return;

        msg = _msg;
        receivedClose = true;
      };
  transport::Node node;
  auto subscribedClose = node.Subscribe(closeTopic, closeCb);
  EXPECT_TRUE(subscribedClose);

  // subscribe to the far microphone's detection topic
  const std::string farTopic = "/model/mic_model_far/sensor/mic_1/detection";
  bool receivedFar{false};
  std::function<void(const msgs::Double &)> farCb =
    [&receivedFar](const msgs::Double &/*_msg*/)
    {
      receivedFar = true;
    };
  auto subscribedFar = node.Subscribe(farTopic, farCb);
  EXPECT_TRUE(subscribedFar);

  // make sure the microphone topics being subscribed to are being advertised
  std::vector<std::string> allTopics;
  node.TopicList(allTopics);
  bool closeTopicAdvertised{false};
  bool farTopicAdvertised{false};
  for (const auto & topic : allTopics)
  {
    if (topic == closeTopic)
      closeTopicAdvertised = true;
    else if (topic == farTopic)
      farTopicAdvertised = true;
  }
  EXPECT_TRUE(closeTopicAdvertised);
  EXPECT_TRUE(farTopicAdvertised);

  // make sure close microphone detection occurred, and that the far microphone
  // didn't detect anything
  server.Run(true, 100, false);
  // (wait on gz-transport for close detection message to be received.
  // Don't exit when a close microphone detection is received because we want to
  // make sure a far microphone detection is never received)
  for (auto sleep = 0; sleep < 30; ++sleep)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(firstTime);
  EXPECT_TRUE(checkedSource);
  EXPECT_TRUE(checkedMicClose);
  EXPECT_TRUE(checkedMicFar);
  EXPECT_TRUE(receivedClose);
  EXPECT_FALSE(receivedFar);
  EXPECT_EQ(msg.header().data(0).key(),
      "world/logical_audio_sensor/model/source_model/sensor/source_1");
}

// See: https://github.com/gazebosim/gz-sim/issues/630
TEST_F(LogicalAudioTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(LogicalAudioServices))
{
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/logical_audio_sensor_plugin_services.sdf";
  serverConfig.SetSdfFile(sdfFile);

  // start server
  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // services to test
  const std::string playService =
    "/model/model_not_playing/sensor/source_2/play";
  const std::string stopService =
    "/model/model_playing/sensor/source_1/stop";
  const unsigned int timeout = 1000;

  bool firstTime{true};
  bool checkedSource1BeforeChange{false};
  bool checkedSource2BeforeChange{false};
  bool checkedSource1AfterChange{false};
  bool checkedSource2AfterChange{false};

  transport::Node node;

  // make a test system to test logical audio source's play/stop services
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &/*_info*/,
                              const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::LogicalAudioSource,
                  components::LogicalAudioSourcePlayInfo>(
          [&](const Entity &/*_entity*/,
              const components::LogicalAudioSource *_source,
              const components::LogicalAudioSourcePlayInfo *_playInfo)
          {
            if (firstTime)
            {
              // used for service calls
              msgs::Boolean response;
              bool result;

              // check source playing state before service call
              if (_source->Data().id == 1u)
              {
                EXPECT_TRUE(_playInfo->Data().playing);
                checkedSource1BeforeChange = true;

                // call the stop service
                auto executed = node.Request(stopService, timeout, response,
                    result);
                EXPECT_TRUE(executed);
                EXPECT_TRUE(response.data());
                EXPECT_TRUE(result);
              }
              else if (_source->Data().id == 2u)
              {
                EXPECT_FALSE(_playInfo->Data().playing);
                checkedSource2BeforeChange = true;

                // call the play service
                auto executed = node.Request(playService, timeout, response,
                    result);
                EXPECT_TRUE(executed);
                EXPECT_TRUE(response.data());
                EXPECT_TRUE(result);
              }
            }
            else
            {
              // check source playing state after service call
              if (_source->Data().id == 1u)
              {
                EXPECT_FALSE(_playInfo->Data().playing);
                checkedSource1AfterChange = true;
              }
              else if (_source->Data().id == 2u)
              {
                EXPECT_TRUE(_playInfo->Data().playing);
                checkedSource2AfterChange = true;
              }
            }

            return true;
          });

          firstTime = false;
      });
  server.AddSystem(testSystem.systemPtr);

  // make sure the play/stop services exist
  std::vector<std::string> allServices;
  node.ServiceList(allServices);
  bool playServiceAdvertised{false};
  bool stopServiceAdvertised{false};
  for (const auto & service : allServices)
  {
    if (service == playService)
      playServiceAdvertised = true;
    else if (service == stopService)
      stopServiceAdvertised = true;
  }
  EXPECT_TRUE(playServiceAdvertised);
  EXPECT_TRUE(stopServiceAdvertised);

  // run the server to test the play/stop services
  server.Run(true, 100, false);
  EXPECT_FALSE(firstTime);
  EXPECT_TRUE(checkedSource1BeforeChange);
  EXPECT_TRUE(checkedSource2BeforeChange);
  EXPECT_TRUE(checkedSource1AfterChange);
  EXPECT_TRUE(checkedSource2AfterChange);
}
