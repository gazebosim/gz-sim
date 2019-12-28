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

#include <algorithm>
#include <climits>
#include <numeric>
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

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                    "ignition::gazebo::MockSystem", nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};

//////////////////////////////////////////////////
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

  public: void RemoveLogsDir()
  {
    common::removeAll(this->logsDir);
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
            sdf::ElementPtr pathElt = std::make_shared<sdf::Element>();
            pathElt->SetName("path");
            pluginElt->AddElementDescription(pathElt);
            pathElt = pluginElt->GetElement("path");
            pathElt->AddValue("string", "", false, "");
            pathElt->Set<std::string>(_logDest);

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
TEST_F(LogSystemTest, LogDefaults)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // No path specified on command line and in SDF, should use default path.
  {
    // Change log path in SDF to empty
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        " ");

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set record path to empty
    recordServerConfig.SetLogRecordPath("");

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 1000, false);

    EXPECT_FALSE(ignLogDirectory().empty());
    // This should be $HOME/.ignition/..., default path set in LogRecord.cc
    std::string home;
    common::env(IGN_HOMEDIR, home);
    EXPECT_TRUE(ignLogDirectory().compare(0, home.length(), home) == 0);
    EXPECT_TRUE(common::exists(ignLogDirectory()));
  }

  // Different paths specified on cmmand line and in SDF, should use the path
  //   from command line.
  {
    // Change log path in SDF
    sdf::Root recordSdfRoot;
    const std::string sdfPath = common::joinPaths(this->logsDir, "sdfPath");
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        sdfPath);

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // This tells server to call AddRecordPlugin() where the logic happens
    recordServerConfig.SetUseLogRecord(true);

    // Mock command line arg. Set record path to something different from in SDF
    const std::string cliPath = common::joinPaths(this->logsDir, "cliPath");
    recordServerConfig.SetLogRecordPath(cliPath);
    recordServerConfig.SetLogRecordPathFromCmdLine(true);

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 1000, false);

    EXPECT_TRUE(common::exists(cliPath));
    EXPECT_FALSE(common::exists(sdfPath));
  }
}

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

  // Check it has SDF message
  auto batch = log.QueryMessages(transport::log::TopicPattern(
      std::regex(".*/sdf")));
  auto recordedIter = batch.begin();
  EXPECT_NE(batch.end(), recordedIter);

  EXPECT_EQ("ignition.msgs.StringMsg", recordedIter->Type());
  EXPECT_TRUE(recordedIter->Topic().find("/sdf"));

  msgs::StringMsg sdfMsg;
  sdfMsg.ParseFromString(recordedIter->Data());
  EXPECT_FALSE(sdfMsg.data().empty());
  EXPECT_EQ(batch.end(), ++recordedIter);

  // Check initial state message
  batch = log.QueryMessages(transport::log::TopicPattern(
      std::regex(".*/changed_state")));
  recordedIter = batch.begin();
  EXPECT_NE(batch.end(), recordedIter);

  EXPECT_EQ("ignition.msgs.SerializedStateMap", recordedIter->Type());
  EXPECT_EQ(recordedIter->Topic(), "/world/log_pendulum/changed_state");

  msgs::SerializedStateMap stateMsg;
  stateMsg.ParseFromString(recordedIter->Data());
  EXPECT_EQ(28, stateMsg.entities_size());
  EXPECT_EQ(batch.end(), ++recordedIter);

  // Pass changed SDF to server
  ServerConfig playServerConfig;
  playServerConfig.SetSdfString(playSdfRoot.Element()->ToString(""));

  // Keep track of total number of pose comparisons
  int nTotal{0};

  // Check poses
  batch = log.QueryMessages(transport::log::TopicPattern(
      std::regex(".*/dynamic_pose/info")));
  recordedIter = batch.begin();
  EXPECT_NE(batch.end(), recordedIter);
  EXPECT_EQ("ignition.msgs.Pose_V", recordedIter->Type());

  // First pose at 1ms time, both from log clock and header
  EXPECT_EQ(1000000, recordedIter->TimeReceived().count());

  msgs::Pose_V recordedMsg;
  recordedMsg.ParseFromString(recordedIter->Data());
  ASSERT_TRUE(recordedMsg.has_header());
  ASSERT_TRUE(recordedMsg.header().has_stamp());
  EXPECT_EQ(0, recordedMsg.header().stamp().sec());
  EXPECT_EQ(1000000, recordedMsg.header().stamp().nsec());

  // Start server
  Server playServer(playServerConfig);

  // Callback function for entities played back
  // Compare current pose being played back with the pose with the closest
  //   timestamp in the recorded file.
  std::function<void(const msgs::Pose_V &)> msgCb =
      [&](const msgs::Pose_V &_playedMsg) -> void
  {
    // Playback continues even after the log file is over
    if (batch.end() == recordedIter)
      return;

    ASSERT_TRUE(_playedMsg.has_header());
    ASSERT_TRUE(_playedMsg.header().has_stamp());
    EXPECT_EQ(0, _playedMsg.header().stamp().sec());

    // Get next recorded message
    EXPECT_EQ("ignition.msgs.Pose_V", recordedIter->Type());
    recordedMsg.ParseFromString(recordedIter->Data());

    ASSERT_TRUE(recordedMsg.has_header());
    ASSERT_TRUE(recordedMsg.header().has_stamp());
    EXPECT_EQ(0, recordedMsg.header().stamp().sec());

    // Log clock timestamp matches message timestamp
    EXPECT_EQ(recordedMsg.header().stamp().nsec(),
        recordedIter->TimeReceived().count());

    // Dynamic poses are throttled according to real time during playback,
    // so we can't guarantee the exact timestamp as recorded.
    EXPECT_NEAR(_playedMsg.header().stamp().nsec(),
        recordedMsg.header().stamp().nsec(), 100000000);

    // Loop through all recorded poses, update map
    std::map<std::string, msgs::Pose> entityRecordedPose;
    for (int i = 0; i < recordedMsg.pose_size(); ++i)
    {
      entityRecordedPose[recordedMsg.pose(i).name()] = recordedMsg.pose(i);
    }

    // Has 4 dynamic entities
    EXPECT_EQ(4, _playedMsg.pose().size());
    EXPECT_EQ(4u, entityRecordedPose.size());

    // Loop through all entities and compare played poses to recorded ones
    for (int i = 0; i < _playedMsg.pose_size(); ++i)
    {
      auto posePlayed = msgs::Convert(_playedMsg.pose(i));
      auto poseRecorded = msgs::Convert(
          entityRecordedPose[_playedMsg.pose(i).name()]);

      EXPECT_NEAR(posePlayed.Pos().X(), poseRecorded.Pos().X(), 0.1)
          << _playedMsg.pose(i).name();
      EXPECT_NEAR(posePlayed.Pos().Y(), poseRecorded.Pos().Y(), 0.1)
          << _playedMsg.pose(i).name();
      EXPECT_NEAR(posePlayed.Pos().Z(), poseRecorded.Pos().Z(), 0.1)
          << _playedMsg.pose(i).name();

      EXPECT_NEAR(posePlayed.Rot().Roll(), poseRecorded.Rot().Roll(), 0.1)
          << _playedMsg.pose(i).name();
      EXPECT_NEAR(posePlayed.Rot().Pitch(), poseRecorded.Rot().Pitch(), 0.1)
          << _playedMsg.pose(i).name();
      EXPECT_NEAR(posePlayed.Rot().Yaw(), poseRecorded.Rot().Yaw(), 0.1)
          << _playedMsg.pose(i).name();
    }

    ++recordedIter;
    nTotal++;
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

  #if !defined (__APPLE__)
  /// \todo(anyone) there seems to be a race condition that sometimes cause an
  /// additional messages to be published by the scene broadcaster
  // 60Hz
  EXPECT_TRUE(nTotal == expectedPoseCount || nTotal == expectedPoseCount + 1);
  #endif

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogControl)
{
  auto logPath = common::joinPaths(PROJECT_SOURCE_PATH, "test", "media",
      "rolling_shapes_log");

  ServerConfig config;
  config.SetLogPlaybackPath(logPath);

  Server server(config);

  Relay testSystem;
  math::Pose3d spherePose;
  bool sphereFound{false};
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Pose, components::Name>(
            [&](const Entity &,
                const components::Pose *_pose,
                const components::Name *_name)->bool
            {
              if (_name->Data() == "sphere")
              {
                sphereFound = true;
                spherePose = _pose->Data();
                return false;
              }
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 10, false);

  EXPECT_TRUE(sphereFound);
  sphereFound = false;

  math::Pose3d initialSpherePose(0, 1.5, 0.5, 0, 0, 0);
  EXPECT_EQ(initialSpherePose, spherePose);
  auto latestSpherePose = spherePose;

  transport::Node node;

  // Seek forward (downhill)
  std::vector<int> secs(9);
  std::iota(std::begin(secs), std::end(secs), 1);

  msgs::LogPlaybackControl req;
  msgs::Boolean res;
  bool result{false};
  unsigned int timeout = 1000;
  std::string service{"/world/default/playback/control"};
  for (auto i : secs)
  {
    req.mutable_seek()->set_sec(i);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    // Run 2 iterations because control messages are processed in the end of an
    // update cycle
    server.Run(true, 2, false);

    EXPECT_TRUE(sphereFound);
    sphereFound = false;

    EXPECT_GT(latestSpherePose.Pos().Z(), spherePose.Pos().Z())
        << "Seconds: [" << i << "]";

    latestSpherePose = spherePose;
  }

  // Rewind
  req.Clear();
  req.set_rewind(true);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 2, false);

  EXPECT_TRUE(sphereFound);
  sphereFound = false;

  EXPECT_EQ(initialSpherePose, spherePose);

  latestSpherePose = math::Pose3d(0, 0, -100, 0, 0, 0);

  // Seek backwards (uphill)
  req.Clear();
  std::reverse(std::begin(secs), std::end(secs));
  for (auto i : secs)
  {
    req.mutable_seek()->set_sec(i);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    server.Run(true, 2, false);

    EXPECT_TRUE(sphereFound);
    sphereFound = false;

    EXPECT_LT(latestSpherePose.Pos().Z(), spherePose.Pos().Z())
        << "Seconds: [" << i << "]";

    latestSpherePose = spherePose;
  }
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogOverwrite)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // Path exists, no command line --log-overwrite, should overwrite
  {
    {
      // Change log path in SDF to build directory
      sdf::Root recordSdfRoot;
      this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
          this->logDir);
     
      // Pass changed SDF to server
      ServerConfig recordServerConfig;
      recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));
     
      // Run for a few seconds to record different poses
      Server recordServer(recordServerConfig);
      recordServer.Run(true, 1000, false);
    }

    // Test path exists
    const std::string firstRecordPath = ignLogDirectory();
    EXPECT_TRUE(common::exists(firstRecordPath));

    {
      // Change log path in SDF to build directory
      sdf::Root recordSdfRoot;
      this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
          this->logDir);
    
      // Pass changed SDF to server
      ServerConfig recordServerConfig;
      recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));
    
      // Run for a few seconds to record different poses
      Server recordServer(recordServerConfig);
      recordServer.Run(true, 1000, false);
    }
    
    // Test record path is same as the first.
    EXPECT_TRUE(ignLogDirectory().compare(firstRecordPath) == 0);
  }

  // Path exists, command line --log-overwrite, should overwrite
  {
    {
      // Change log path in SDF to build directory
      sdf::Root recordSdfRoot;
      this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
          this->logDir);
     
      // Pass changed SDF to server
      ServerConfig recordServerConfig;
      recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));
     
      // Run for a few seconds to record different poses
      Server recordServer(recordServerConfig);
      recordServer.Run(true, 1000, false);
    }

    // Test path exists
    const std::string firstRecordPath = ignLogDirectory();
    EXPECT_TRUE(common::exists(firstRecordPath));

    {
      // Change log path in SDF to build directory
      sdf::Root recordSdfRoot;
      this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
          this->logDir);
    
      // Pass changed SDF to server
      ServerConfig recordServerConfig;
      recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));
    
      // Set overwrite flag
      recordServerConfig.SetLogRecordOverwrite(true);
    
      // Run for a few seconds to record different poses
      Server recordServer(recordServerConfig);
      recordServer.Run(true, 1000, false);
    }
    
    // Test record path is same as the first.
    EXPECT_TRUE(ignLogDirectory().compare(firstRecordPath) == 0);
  }

  this->RemoveLogsDir();
}
