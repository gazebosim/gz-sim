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

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Batch.hh>
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

  public: void CreateCacheDir()
  {
    // Configure to use binary path as cache
    this->cacheDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
      "test_cache");
    if (common::exists(cacheDir))
    {
      common::removeAll(cacheDir);
    }
    common::createDirectories(cacheDir);
  }

  public: std::string cacheDir = "";
};

/////////////////////////////////////////////////
void ChangeLogPath(sdf::Root &_sdfRoot, const std::string &_sdfPath,
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

/////////////////////////////////////////////////
// This test checks that a file is created by log recorder
TEST_F(LogSystemTest, CreateLogFile)
{
  this->CreateCacheDir();
  std::string logDest = common::joinPaths(this->cacheDir, "log");

  ServerConfig serverConfig;
  serverConfig.SetResourceCache(this->cacheDir);

  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_record_dbl_pendulum.sdf");

  // Change log path in SDF to build directory
  sdf::Root sdfRoot;
  ChangeLogPath(sdfRoot, sdfPath, "LogRecord", logDest);

  // Pass changed SDF to server
  serverConfig.SetSdfString(sdfRoot.Element()->ToString(""));

  // Start server
  Server server(serverConfig);
  // Run for a few seconds to record different poses
  server.Run(true, 3000, false);

  // Verify file is created
  EXPECT_TRUE(common::exists(common::joinPaths(logDest, "state.tlog")));
}

/////////////////////////////////////////////////
// This test checks that poses are played back correctly
TEST_F(LogSystemTest, PosePlayback)
{
  // Configure to use binary path as cache
  std::string recordCacheDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
    "test_cache");

  // Log file directory created by CreateLogFile test above
  EXPECT_TRUE(common::exists(recordCacheDir));
  std::string logDest = common::joinPaths(recordCacheDir, "log");

  // World file to load
  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_playback.sdf");
  // Change log path in world SDF to build directory
  sdf::Root sdfRoot;
  ChangeLogPath(sdfRoot, sdfPath, "LogPlayback",  logDest);
  const sdf::World * sdfWorld = sdfRoot.WorldByIndex(0);
  // Playback pose topic
  std::string poseTopic = "/world/" +
    sdfWorld->Element()->GetAttribute("name")->GetAsString() + "/pose/info";


  // Load log file recorded by CreateLogFile test above
  std::string logName = common::joinPaths(logDest, "state.tlog");
  EXPECT_TRUE(common::exists(logName));
  transport::log::Log log;
  log.Open(logName);

  // Recorded SDF world file
  std::string logSdfName = common::joinPaths(logDest, "state.sdf");
  EXPECT_TRUE(common::exists(logSdfName));
  sdf::Root logSdfRoot;
  EXPECT_EQ(logSdfRoot.Load(logSdfName).size(), 0lu);
  EXPECT_GT(logSdfRoot.WorldCount(), 0lu);
  const sdf::World * logSdfWorld = logSdfRoot.WorldByIndex(0);
  // Recorded pose topic
  std::string logPoseTopic = "/world/" +
    logSdfWorld->Element()->GetAttribute("name")->GetAsString() + "/pose/info";

  // Set start time range
  auto begin = std::chrono::nanoseconds(0);


  ServerConfig serverConfig;
  serverConfig.SetResourceCache(recordCacheDir);

  // Pass changed SDF to server
  serverConfig.SetSdfString(sdfRoot.Element()->ToString(""));

  // Start server
  Server server(serverConfig);


  // Callback function for entities played back
  auto msgCb = [&](const msgs::Pose_V &_msg) -> void
  {
    server.SetPaused(true);

    // Look for recorded topics with current sim time
    std::chrono::nanoseconds end =
      std::chrono::seconds(_msg.header().stamp().sec()) +
      std::chrono::nanoseconds(_msg.header().stamp().nsec());
    auto timeRange = transport::log::QualifiedTimeRange(begin, end);

    // Access selective recorded messages in .tlog file
    transport::log::TopicList topicList(logPoseTopic);
    transport::log::Batch batch = log.QueryMessages(topicList);
    transport::log::MsgIter iter = batch.begin();
    // If no messages
    if (iter == batch.end())
      return;

    // Skip until last timestamp in range, closest to current time
    msgs::Pose_V posevMsg;
    for (; iter != batch.end(); ++iter)
    {
      // Convert recorded binary bytes in string into a ign-msgs msg
      posevMsg.ParseFromString(iter->Data());
    }

    // Maps entity to pose recorded
    // Key: entity. Value: pose
    std::map <Entity, msgs::Pose> idToPose;
    // Loop through all recorded poses, update map
    for (int i = 0; i < posevMsg.pose_size(); ++i)
    {
      msgs::Pose pose = posevMsg.pose(i);
      idToPose.insert_or_assign(pose.id(), pose);
    }

    // Loop through all played poses and compare to recorded ones
    //std::cerr << _msg.pose_size() << std::endl;
    for (int i = 0; i < _msg.pose_size(); ++i)
    {
      math::Pose3d posePlayed = msgs::Convert(_msg.pose(i));
      math::Pose3d poseRecorded = msgs::Convert(idToPose.at(_msg.pose(i).id()));

      /*
      double dist = sqrt(pow(posePlayed.Pos().X() - poseRecorded.Pos().X(), 2) + 
        pow(posePlayed.Pos().Y() - poseRecorded.Pos().Y(), 2) + 
        pow(posePlayed.Pos().Z() - poseRecorded.Pos().Z(), 2));

      if (dist >= 0.3)
      {
        std::cerr << _msg.pose(i).name() << std::endl;
        std::cerr << posePlayed << std::endl;
        std::cerr << poseRecorded << std::endl;
      }

      // Allow small tolerance to difference between recorded and played back
      EXPECT_LT(dist, 0.3);
      */


      auto diff = posePlayed - poseRecorded;
      std::cerr << diff << std::endl;

      EXPECT_EQ(diff, math::Pose3d());

      /*
      EXPECT_NEAR(abs(diff.Pos().X()), 0.1);
      EXPECT_NEAR(abs(diff.Pos().Y()), 0.1);
      EXPECT_NEAR(abs(diff.Pos().Z()), 0.1);

      EXPECT_NEAR(abs(diff.Rot().W()), 0.1);
      EXPECT_NEAR(abs(diff.Rot().X()), 0.1);
      EXPECT_NEAR(abs(diff.Rot().Y()), 0.1);
      EXPECT_NEAR(abs(diff.Rot().Z()), 0.1);
      */
    }

    // Update begin time range for next time step
    begin = std::chrono::nanoseconds(end);

    server.SetPaused(false);
  };


  // Subscribe to ignition topic and compare to logged poses
  transport::Node node;
  // Have to create an lvalue here for Node::Subscribe to work.
  auto callbackFunc = std::function<void(const msgs::Pose_V &)>(msgCb);
  node.Subscribe(poseTopic, callbackFunc);

  // Run for a few seconds to play back different poses
  server.Run(true, 3000, false);

  common::removeAll(recordCacheDir);
}
