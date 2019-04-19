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
  public: void CreateCacheDir()
  {
    // Configure to use binary path as cache
    if (common::exists(this->cacheDir))
    {
      common::removeAll(this->cacheDir);
    }
    common::createDirectories(this->cacheDir);
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
  public: std::string cacheDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
      "test_cache");
};

/////////////////////////////////////////////////
TEST_F(LogSystemTest, RecordAndPlayback)
{
  // Run a world and record to a log file

  this->CreateCacheDir();
  std::string logDest = common::joinPaths(this->cacheDir, "log");

  ServerConfig recordServerConfig;
  recordServerConfig.SetResourceCache(this->cacheDir);

  const auto recordSdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_record_dbl_pendulum.sdf");

  // Change log path in SDF to build directory
  sdf::Root recordSdfRoot;
  this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord", logDest);

  // Pass changed SDF to server
  recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

  // Start server
  Server recordServer(recordServerConfig);
  // Run for a few seconds to record different poses
  recordServer.Run(true, 1000, false);

  // Verify file is created
  EXPECT_TRUE(common::exists(common::joinPaths(logDest, "state.tlog")));


  // Run a world, load recorded log file, and check recorded poses are same
  //   as poses in live physics

  // World file to load
  const auto playSdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_playback.sdf");
  // Change log path in world SDF to build directory
  sdf::Root playSdfRoot;
  this->ChangeLogPath(playSdfRoot, playSdfPath, "LogPlayback",  logDest);
  const sdf::World * sdfWorld = playSdfRoot.WorldByIndex(0);
  // Playback pose topic
  std::string poseTopic = "/world/" +
    sdfWorld->Element()->GetAttribute("name")->GetAsString() + "/pose/info";


  // Load log file recorded above
  std::string logName = common::joinPaths(logDest, "state.tlog");
  EXPECT_TRUE(common::exists(logName));
  transport::log::Log log;
  log.Open(logName);

  // Original world file recorded by CreateLogFile test above
  const auto logSdfName = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_record_dbl_pendulum.sdf");
  EXPECT_TRUE(common::exists(logSdfName));
  sdf::Root logSdfRoot;
  EXPECT_EQ(logSdfRoot.Load(logSdfName).size(), 0lu);
  EXPECT_GT(logSdfRoot.WorldCount(), 0lu);
  const sdf::World * logSdfWorld = logSdfRoot.WorldByIndex(0);
  // Recorded pose topic
  std::string logPoseTopic = "/world/" +
    logSdfWorld->Element()->GetAttribute("name")->GetAsString() + "/pose/info";

  ServerConfig playServerConfig;
  playServerConfig.SetResourceCache(this->cacheDir);

  // Pass changed SDF to server
  playServerConfig.SetSdfString(playSdfRoot.Element()->ToString(""));

  int nDiffs = 0;
  int nTotal = 0;

  // Set start time for the batch to load in initial iteration
  auto begin = std::chrono::nanoseconds(0);

  // Start server
  Server playServer(playServerConfig);

  // Callback function for entities played back
  // Compare current pose being played back with the pose with the closest
  //   timestamp in the recorded file.
  auto msgCb = [&](const msgs::Pose_V &_msg) -> void
  {
    playServer.SetPaused(true);

    // Filter recorded topics with current sim time
    std::chrono::nanoseconds end =
      std::chrono::seconds(_msg.header().stamp().sec()) +
      std::chrono::nanoseconds(_msg.header().stamp().nsec());

    auto beginQT = transport::log::QualifiedTime(begin);
    auto endQT = transport::log::QualifiedTime(end);
    auto timeRange = transport::log::QualifiedTimeRange(beginQT, endQT);
    transport::log::TopicList topicList(logPoseTopic, timeRange);

    transport::log::Batch batch = log.QueryMessages(topicList);
    transport::log::MsgIter iter = batch.begin();
    // If no messages
    if (iter == batch.end())
    {
      playServer.SetPaused(false);
      return;
    }

    msgs::Pose_V posevMsg;
    posevMsg.ParseFromString(iter->Data());

    // Maps entity to recorded pose
    // Key: entity. Value: pose
    std::map <Entity, msgs::Pose> idToPose;
    // Loop through all recorded poses, update map
    for (int i = 0; i < posevMsg.pose_size(); ++i)
    {
      msgs::Pose pose = posevMsg.pose(i);
      idToPose.insert_or_assign(pose.id(), pose);
    }

    // Loop through all links and compare played poses to recorded ones
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      math::Pose3d posePlayed = msgs::Convert(_msg.pose(i));
      math::Pose3d poseRecorded = msgs::Convert(idToPose.at(_msg.pose(i).id()));

      auto diff = posePlayed - poseRecorded;

      EXPECT_NEAR(abs(diff.Pos().X()), 0, 0.1);
      EXPECT_NEAR(abs(diff.Pos().Y()), 0, 0.1);
      EXPECT_NEAR(abs(diff.Pos().Z()), 0, 0.1);

      EXPECT_NEAR(abs(diff.Rot().W()), 1, 0.1);
      EXPECT_NEAR(abs(diff.Rot().X()), 0, 0.1);
      EXPECT_NEAR(abs(diff.Rot().Y()), 0, 0.1);
      EXPECT_NEAR(abs(diff.Rot().Z()), 0, 0.1);

      // Omit comparing W of quaternion, which is 1 for identity
      if (abs(diff.Pos().X()) > 0.1 ||
          abs(diff.Pos().Y()) > 0.1 ||
          abs(diff.Pos().Z()) > 0.1 ||
          abs(diff.Rot().X()) > 0.1 ||
          abs(diff.Rot().Y()) > 0.1 ||
          abs(diff.Rot().Z()) > 0.1)
      {
        igndbg << begin.count() << " to " << end.count() << std::endl;
        // Print difference between timestamps
        igndbg << posevMsg.header().stamp().sec() * 1000000000 +
          posevMsg.header().stamp().nsec() << std::endl;

        igndbg << _msg.pose(i).name() << std::endl;
        igndbg << abs(diff.Pos().X()) << " "
                  << abs(diff.Pos().Y()) << " "
                  << abs(diff.Pos().Z()) << " "
                  << abs(diff.Rot().X()) << " "
                  << abs(diff.Rot().Y()) << " "
                  << abs(diff.Rot().Z()) << std::endl;
        nDiffs++;
      }
      nTotal++;
    }

    // Update begin time range for next time step
    begin = std::chrono::nanoseconds(end);

    playServer.SetPaused(false);
  };

  // Subscribe to ignition topic and compare to logged poses
  transport::Node node;
  // Have to create an lvalue here for Node::Subscribe to work.
  auto callbackFunc = std::function<void(const msgs::Pose_V &)>(msgCb);
  node.Subscribe(poseTopic, callbackFunc);

  // Run for a few seconds to play back different poses
  playServer.Run(true, 1000, false);

  igndbg << nDiffs << " out of " << nTotal
    << " poses played back do not match those recorded\n";

  common::removeAll(this->cacheDir);
}
