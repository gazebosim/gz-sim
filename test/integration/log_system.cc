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
#include <ignition/msgs/pose_v.pb.h>

#include <climits>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Batch.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/MsgIter.hh>
#include <ignition/transport/log/QualifiedTime.hh>
#include <ignition/math/Pose3.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

class LogSystemTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }

  // Create a temporary directory in build path for recorded data
  public: void CreateLogsDir()
  {
    // Configure to use binary path as cache
    if (common::exists(this->logsDir))
    {
      common::removeAll(this->logsDir);
    }
    common::createDirectories(this->logsDir);
    common::createDirectories(this->logPlaybackDir);
  }

  // Change path of recorded log file in SDF string loaded from file
  public: void ChangeLogPath(sdf::Root &_sdfRoot, const std::string &_sdfPath,
     const std::string &_pluginName, const std::string &_logDest)
  {
    EXPECT_EQ(_sdfRoot.Load(_sdfPath).size(), 0lu);
    EXPECT_GT(_sdfRoot.WorldCount(), 0lu);
    const sdf::World * sdfWorld = _sdfRoot.WorldByIndex(0);
    EXPECT_TRUE(sdfWorld->Element()->HasElement("plugin"));

    sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");
    while (pluginElt != nullptr)
    {
      if (pluginElt->HasAttribute("name"))
      {
        // Change log path to build directory
        if (pluginElt->GetAttribute("name")->GetAsString().find(_pluginName)
          != std::string::npos)
        {
          if (pluginElt->HasElement("path"))
          {
            sdf::ElementPtr pathElt = pluginElt->GetElement("path");
            pathElt->Set(_logDest);
          }
          else
          {
            sdf::ElementPtr pathElt = pluginElt->AddElement("path");
            pathElt->Set(_logDest);
          }
        }
      }

      // Go to next plugin
      pluginElt = pluginElt->GetNextElement("plugin");
    }
  }

  // Temporary directory in binary build path for recorded data
  public: std::string logsDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
      "test_logs");

  /// \brief Path to recorded log file
  public: std::string logDir = common::joinPaths(logsDir, "test_logs_record");

  /// \brief Path to log file for playback
  public: std::string logPlaybackDir =
      common::joinPaths(logsDir, "test_logs_playback");
};

/////////////////////////////////////////////////
TEST_F(LogSystemTest, RecordAndPlayback)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // Record
  {
    // World with moving entities
    const auto recordSdfPath = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      "log_record_dbl_pendulum.sdf");

    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));
    Server recordServer(recordServerConfig);

    // Run for a few seconds to record different poses
    recordServer.Run(true, 1000, false);
  }

  // Verify file is created
  auto logFile = common::joinPaths(this->logDir, "state.tlog");
  EXPECT_TRUE(common::exists(logFile));

  // move the log file to the playback directory
  auto logPlaybackFile = common::joinPaths(this->logPlaybackDir, "state.tlog");
  common::moveFile(logFile, logPlaybackFile);
  EXPECT_TRUE(common::exists(logPlaybackFile));

  // World file to load
  const auto playSdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_playback.sdf");

  // Change log path in world SDF to build directory
  sdf::Root playSdfRoot;
  this->ChangeLogPath(playSdfRoot, playSdfPath, "LogPlayback",
      this->logPlaybackDir);
  ASSERT_EQ(1u, playSdfRoot.WorldCount());

  const auto sdfWorld = playSdfRoot.WorldByIndex(0);
  EXPECT_EQ("default", sdfWorld->Name());

  // Load log file recorded above
  transport::log::Log log;
  log.Open(logPlaybackFile);
  auto batch = log.QueryMessages();

  // Pass changed SDF to server
  ServerConfig playServerConfig;
  playServerConfig.SetSdfString(playSdfRoot.Element()->ToString(""));

  // Keep track of total number of pose comparisons
  int nTotal{0};
  bool hasSdfMessage{false};
  bool hasState{false};

  // Set start time for the batch to load in initial iteration
  auto recordedIter = batch.begin();

  // Start server
  Server playServer(playServerConfig);

  // Callback function for entities played back
  // Compare current pose being played back with the pose with the closest
  //   timestamp in the recorded file.
  std::function<void(const msgs::Pose_V &)> msgCb =
      [&](const msgs::Pose_V &_playedMsg) -> void
  {
    playServer.SetPaused(true);

    ASSERT_TRUE(_playedMsg.has_header());
    ASSERT_TRUE(_playedMsg.header().has_stamp());
    EXPECT_EQ(0, _playedMsg.header().stamp().sec());

    // Check SDF message
    if (recordedIter->Type() == "ignition.msgs.StringMsg")
    {
      EXPECT_TRUE(recordedIter->Topic().find("/sdf"));

      msgs::StringMsg sdfMsg;
      sdfMsg.ParseFromString(recordedIter->Data());

      EXPECT_FALSE(sdfMsg.data().empty());

      hasSdfMessage = true;
      ++recordedIter;
    }

    // Check initial state message
    if (recordedIter->Type() == "ignition.msgs.SerializedState")
    {
      EXPECT_EQ(recordedIter->Topic(), "/world/log_pendulum/changed_state");

      msgs::SerializedStateMap stateMsg;
      stateMsg.ParseFromString(recordedIter->Data());

      EXPECT_EQ(28, stateMsg.entities_size());

      hasState = true;
      ++recordedIter;
    }

    if (recordedIter != batch.end())
    {
      EXPECT_EQ("ignition.msgs.Pose_V", recordedIter->Type());

      // Get next recorded message
      msgs::Pose_V recordedMsg;
      recordedMsg.ParseFromString(recordedIter->Data());

      ASSERT_TRUE(recordedMsg.has_header());
      ASSERT_TRUE(recordedMsg.header().has_stamp());
      EXPECT_EQ(0, recordedMsg.header().stamp().sec());

      // Check time stamps are close
      EXPECT_LT(abs(_playedMsg.header().stamp().nsec() -
            recordedMsg.header().stamp().nsec()), 100000000);

      // Maps entity to recorded pose
      // Key: entity. Value: pose
      std::map <Entity, msgs::Pose> entityRecordedPose;
      // Loop through all recorded poses, update map
      for (int i = 0; i < recordedMsg.pose_size(); ++i)
      {
        msgs::Pose pose = recordedMsg.pose(i);
        entityRecordedPose.insert_or_assign(pose.id(), pose);
      }

      // Has 4 dynamic entities
      EXPECT_EQ(4, _playedMsg.pose().size());
      EXPECT_EQ(4u, entityRecordedPose.size());

      // Loop through all entities and compare played poses to recorded ones
      for (int i = 0; i < _playedMsg.pose_size(); ++i)
      {
        auto posePlayed = msgs::Convert(_playedMsg.pose(i));
        auto poseRecorded = msgs::Convert(entityRecordedPose.at(
              _playedMsg.pose(i).id()));

        auto diff = posePlayed - poseRecorded;

        EXPECT_NEAR(abs(diff.Pos().X()), 0, 0.1);
        EXPECT_NEAR(abs(diff.Pos().Y()), 0, 0.1);
        EXPECT_NEAR(abs(diff.Pos().Z()), 0, 0.1);

        EXPECT_NEAR(abs(diff.Rot().W()), 1, 0.1);
        EXPECT_NEAR(abs(diff.Rot().X()), 0, 0.1);
        EXPECT_NEAR(abs(diff.Rot().Y()), 0, 0.1);
        EXPECT_NEAR(abs(diff.Rot().Z()), 0, 0.1);
      }
      playServer.SetPaused(false);

      ++recordedIter;
      nTotal++;
    }
  };

  // Subscribe to ignition topic and compare to logged poses
  transport::Node node;
  // TODO(louise) The world name should match the recorded world
  node.Subscribe("/world/default/dynamic_pose/info", msgCb);

  int playbackSteps = 500;
  int poseHz = 60;
  int expectedPoseCount = playbackSteps * 1e-3 / (1.0/poseHz);
  // Run for a few seconds to play back different poses
  playServer.Run(true, playbackSteps, false);

  int sleep = 0;
  int maxSleep = 16;
  for (; nTotal < expectedPoseCount && sleep < maxSleep; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_TRUE(hasSdfMessage);
  EXPECT_TRUE(hasState);

  #if !defined (__APPLE__)
  /// \todo(anyone) there seems to be a race condition that sometimes cause an
  /// additional messages to be published by the scene broadcaster
  // 60Hz
  EXPECT_TRUE(nTotal == expectedPoseCount || nTotal == expectedPoseCount + 1);
  #endif

  common::removeAll(this->logsDir);
}
