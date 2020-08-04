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
#ifndef __APPLE__
#include <filesystem>
#endif
#include <numeric>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools/Zip.hh>
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

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

static const std::string kBinPath(PROJECT_BINARY_PATH);

// TODO(anyone) Support command line options for OSX, see
// https://github.com/ignitionrobotics/ign-gazebo/issues/25
#ifndef __APPLE__
static const std::string kSdfFileOpt =  // NOLINT(runtime/string)
" ";
static const std::string kIgnCommand(
  "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=" + kBinPath + "/lib LD_LIBRARY_PATH=" +
  kBinPath + "/lib:/usr/local/lib:${LD_LIBRARY_PATH} ign gazebo -s ");
#endif

/////////////////////////////////////////////////
std::string customExecStr(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != nullptr)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
// Count the number of entries in a directory, both files and directories
#ifndef __APPLE__
int entryCount(const std::string &_directory)
{
  if (!common::exists(_directory))
    return 0;

  auto it = std::filesystem::directory_iterator(_directory);
  return std::count_if(begin(it), end(it), [](auto &)
      {
        return true;
      });
}

/////////////////////////////////////////////////
// Return a list of entries in the directory
void entryList(const std::string &_directory, std::vector<std::string> &_paths)
{
  _paths.clear();

  if (!common::exists(_directory))
    return;

  for (auto &entry : std::filesystem::directory_iterator(_directory))
  {
    _paths.push_back(entry.path().string());
  }
}

/////////////////////////////////////////////////
// Compare between two lists, return items that are different between the lists
// Side effects: order of elements in _paths1 and _paths2 will be sorted
void entryDiff(std::vector<std::string> &_paths1,
  std::vector<std::string> &_paths2, std::vector<std::string> &_diff)
{
  _diff.clear();

  std::sort(_paths1.begin(), _paths1.end());
  std::sort(_paths2.begin(), _paths2.end());
  std::vector<std::string> pathsUnion;
  pathsUnion.resize(_paths1.size() + _paths2.size());
  auto unionIt = std::set_union(_paths1.begin(),
    _paths1.end(), _paths2.begin(), _paths2.end(), pathsUnion.begin());
  pathsUnion.resize(unionIt - pathsUnion.begin());

  std::vector<std::string> pathsIntersection;
  pathsIntersection.resize(_paths1.size() + _paths2.size());
  auto intersectionIt = std::set_intersection(
    _paths1.begin(), _paths1.end(), _paths2.begin(), _paths2.end(),
    pathsIntersection.begin());
  pathsIntersection.resize(intersectionIt - pathsIntersection.begin());

  _diff.resize(pathsUnion.size() + pathsIntersection.size());
  auto diffIt = std::set_difference(pathsUnion.begin(), pathsUnion.end(),
    pathsIntersection.begin(), pathsIntersection.end(), _diff.begin());
  _diff.resize(diffIt - _diff.begin());
}
#endif

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

  // Append extension to the end of a path, removing the separator at
  //   the end of the path if necessary.
  // \param[in] _path Path to append extension to. May have separator at
  //   the end.
  // \param[in] _ext Extension including dot "." to be appended
  public: std::string AppendExtension(const std::string &_path,
    const std::string &_ext)
  {
    std::string result = _path;

    // Remove the separator at end of path
    if (!std::string(1, _path.back()).compare(common::separator("")))
    {
      result = _path.substr(0, _path.length() - 1);
    }
    result += _ext;

    return result;
  }

  // Remove the test logs directory
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
    const sdf::World *sdfWorld = _sdfRoot.WorldByIndex(0);
    EXPECT_TRUE(sdfWorld->Element()->HasElement("plugin"));

    sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");
    while (pluginElt != nullptr)
    {
      EXPECT_TRUE(pluginElt->HasAttribute("name"));

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

      // Go to next plugin
      pluginElt = pluginElt->GetNextElement("plugin");
    }
  }

  // Run the server to record, passing in compress flag.
  // \param[in] _recordSdfRoot SDF Root element of the world to load
  // \param[in] _cmpPath Path for compressed file
  public: void RunCompress(sdf::Root &_recordSdfRoot,
    const std::string &_cmpPath)
  {
    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(_recordSdfRoot.Element()->ToString(""));

    // Set compress path
    recordServerConfig.SetLogRecordCompressPath(_cmpPath);

    // This tells server to call AddRecordPlugin() where flags are passed to
    //   recorder.
    recordServerConfig.SetUseLogRecord(true);

    // Run server
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Temporary directory in binary build path for recorded data
  public: std::string logsDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
      "test_logs");

  /// \brief Path to recorded log file
  public: std::string logDir = common::joinPaths(logsDir,
      "test_logs_record");

  /// \brief Path to log file for playback
  public: std::string logPlaybackDir =
      common::joinPaths(this->logsDir, "test_logs_playback");
};

/////////////////////////////////////////////////
// Logging behavior when no paths are specified
TEST_F(LogSystemTest, LogDefaults)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // Change environment variable so that test files aren't written to $HOME
  std::string homeOrig;
  common::env(IGN_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeFake.c_str(), 1), 0);

  // Test case 1:
  // No path specified, on both command line and SDF. This does not go through
  // ign.cc, so ignLogDirectory() is not initialized (empty string). Recording
  // should not take place.
  {
    // Change log path in SDF to empty
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        " ");

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set record path to empty
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath("");

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 200, false);
  }

  // Check ignLogDirectory is empty
  EXPECT_TRUE(ignLogDirectory().empty());

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();

#ifndef __APPLE__
  this->CreateLogsDir();

  // Test case 2:
  // No path specified on command line (only --record, no --record-path).
  // No path specified in SDF.
  // Run from command line, which should trigger ign.cc, which should initialize
  // ignLogDirectory() to default timestamp path. Both console and state logs
  // should be recorded here.

  // Store number of files before running
  auto logPath = common::joinPaths(homeFake.c_str(), ".ignition", "gazebo",
      "log");
  int nEntries = entryCount(logPath);
  std::vector<std::string> entriesBefore;
  entryList(logPath, entriesBefore);

  {
    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record " + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Check the diff of list of files and assume there is a single diff, it
  // being the newly created log directory from the run above.
  EXPECT_EQ(nEntries + 1, entryCount(logPath));
  std::vector<std::string> entriesAfter;
  entryList(logPath, entriesAfter);
  std::vector<std::string> entriesDiff;
  entryDiff(entriesBefore, entriesAfter, entriesDiff);
  EXPECT_EQ(1ul, entriesDiff.size());
  // This should be $HOME/.ignition/..., default path
  std::string timestampPath = entriesDiff[0];

  EXPECT_FALSE(timestampPath.empty());
  EXPECT_EQ(0, timestampPath.compare(0, logPath.length(), logPath));
  EXPECT_TRUE(common::exists(timestampPath));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "server_console.log")));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "state.tlog")));
  EXPECT_EQ(2, entryCount(timestampPath));
#endif

  // Revert environment variable after test is done
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeOrig.c_str(), 1), 0);
}

/////////////////////////////////////////////////
// Logging behavior when a path is specified either via the C++ API, SDF, or
// the command line.
TEST_F(LogSystemTest, LogPaths)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // Test case 1:
  // A path is specified in SDF.
  // No path specified in C++ API.
  // LogIgnoreSdfPath is not set.
  // Should take SDF path. State log should be stored here. Console log is not
  // initialized because ign.cc is not triggered.
  {
    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set record path to empty
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath("");

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 200, false);
  }

  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir,
      "state.tlog")));
#ifndef __APPLE__
  EXPECT_EQ(1, entryCount(this->logDir));
#endif

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Change environment variable so that test files aren't written to $HOME
  std::string homeOrig;
  common::env(IGN_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeFake.c_str(), 1), 0);

  // Store number of files before running
  auto logPath = common::joinPaths(homeFake.c_str(), ".ignition", "gazebo",
      "log");
#ifndef __APPLE__
  int nEntries = entryCount(logPath);
  std::vector<std::string> entriesBefore;
  entryList(logPath, entriesBefore);

  // Test case 2:
  // A path is specified in SDF.
  // No path specified on command line (therefore LogIgnoreSdfPath is not set).
  // State log should be stored in SDF path.
  // Console log should be stored to default timestamp path ignLogDirectory
  // because ign.cc is triggered by command line.
  {
    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Save changed SDF to temporary file
    std::string tmpRecordSdfPath = common::joinPaths(this->logsDir,
      "with_record_path.sdf");
    std::ofstream ofs(tmpRecordSdfPath);
    ofs << recordSdfRoot.Element()->ToString("").c_str();
    ofs.close();

    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record " + kSdfFileOpt + tmpRecordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Check state.tlog is stored to path specified in SDF
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir,
      "state.tlog")));
  EXPECT_EQ(1, entryCount(this->logDir));

  // Check the diff of list of files in directory, and assume there is
  // a single diff, it being the newly created log directory from the run above.
  EXPECT_EQ(nEntries + 1, entryCount(logPath));
  std::vector<std::string> entriesAfter;
  entryList(logPath, entriesAfter);
  std::vector<std::string> entriesDiff;
  entryDiff(entriesBefore, entriesAfter, entriesDiff);
  EXPECT_EQ(1ul, entriesDiff.size());
  // This should be $HOME/.ignition/..., default path
  std::string timestampPath = entriesDiff[0];

  EXPECT_FALSE(timestampPath.empty());
  EXPECT_EQ(0, timestampPath.compare(0, logPath.length(), logPath));
  EXPECT_TRUE(common::exists(timestampPath));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "server_console.log")));
  EXPECT_EQ(1, entryCount(timestampPath));
#endif

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 3:
  // A path is specified in SDF.
  // A different path is specified via C++ API.
  // LogIgnoreSdfPath is not set (pure C++ API usage).
  // Should store state.tlog to SDF path. Console log is not initialized
  // because ign.cc is not triggered.
  std::string stateLogPath = this->logDir;

  std::string consoleLogPath = common::joinPaths(this->logsDir, "console");
  common::createDirectories(consoleLogPath);

  {
    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        stateLogPath);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set record path to empty
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath("");

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 200, false);

    // Terminate server to close tlog file, otherwise we get a temporary
    // tlog-journal file
  }

  EXPECT_TRUE(common::exists(common::joinPaths(stateLogPath, "state.tlog")));
#ifndef __APPLE__
  EXPECT_EQ(1, entryCount(stateLogPath));
  EXPECT_EQ(0, entryCount(consoleLogPath));
#endif
  common::removeAll(consoleLogPath);

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 4:
  // A path is specified in SDF.
  // A different path is specified via C++ API.
  // LogIgnoreSdfPath is set (similar to specifying a path on command line).
  // Should take C++ API path. State log should be stored here. Console log is
  // not initialized because ign.cc is not triggered.
  const std::string sdfPath = common::joinPaths(this->logsDir, "sdfPath");
  const std::string cppPath = common::joinPaths(this->logsDir, "cppPath");
  {
    // Change log path in SDF
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        sdfPath);

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // This tells server to call AddRecordPlugin() where the logic happens
    recordServerConfig.SetUseLogRecord(true);

    // Mock command line arg. Set record path to something different from in SDF
    recordServerConfig.SetLogRecordPath(cppPath);
    // Set this flag to simulate path being passed in from command line
    recordServerConfig.SetLogIgnoreSdfPath(true);

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 200, false);

    // Terminate server to close tlog file, otherwise we get a temporary
    // tlog-journal file
  }

  EXPECT_TRUE(common::exists(cppPath));
  EXPECT_TRUE(common::exists(common::joinPaths(cppPath, "state.tlog")));
#ifndef __APPLE__
  EXPECT_EQ(1, entryCount(cppPath));
  EXPECT_FALSE(common::exists(sdfPath));

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 5:
  // A path is specified by --record-path on command line.
  // Both state and console logs should be stored here.
  {
    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + this->logDir + " " + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir, "state.tlog")));
  // \FIXME Apple uses deprecated command line, so some options don't work
  // correctly.
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir,
    "server_console.log")));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 6:
  // A path is specified in SDF.
  // A path is specified by --record-path on command line.
  // Path in SDF should be ignored. Both state and console logs should be
  // stored to --record-path path.
  std::string cliPath = common::joinPaths(this->logDir, "cli");
  {
    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Save changed SDF to temporary file
    std::string tmpRecordSdfPath = common::joinPaths(this->logsDir,
      "with_record_path.sdf");
    // TODO(anyone): Does this work on Apple?
    std::ofstream ofs(tmpRecordSdfPath);
    ofs << recordSdfRoot.Element()->ToString("").c_str();
    ofs.close();

    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + cliPath + " " + kSdfFileOpt + tmpRecordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // \FIXME Apple uses deprecated command line, so some options don't work
  // correctly.
  EXPECT_TRUE(common::exists(common::joinPaths(cliPath, "state.tlog")));
  EXPECT_TRUE(common::exists(common::joinPaths(cliPath,
    "server_console.log")));
  EXPECT_EQ(2, entryCount(cliPath));
#endif

  // Revert environment variable after test is done
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeOrig.c_str(), 1), 0);

  this->RemoveLogsDir();
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

  test::Relay testSystem;
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
#ifndef __APPLE__
  EXPECT_EQ(1, entryCount(this->logsDir));
  EXPECT_EQ(0, entryCount(this->logDir));
#endif

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  ignLogInit(this->logDir, "server_console.log");
  EXPECT_EQ(this->logDir, ignLogDirectory());

  // Record something to create some files
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
    recordServer.Run(true, 100, false);

    // Terminate server to close tlog file, otherwise we get a temporary
    // tlog-journal file
  }

  // Test files exist
  const std::string tlogPath = common::joinPaths(this->logDir, "state.tlog");
  EXPECT_TRUE(common::exists(tlogPath));

  auto clogPath = common::joinPaths(this->logDir, "server_console.log");
  EXPECT_TRUE(common::exists(clogPath));

#ifndef __APPLE__
  // Log files were created
  EXPECT_EQ(2, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));
  std::filesystem::path tlogStdPath = tlogPath;
  auto tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);
#endif

  // Test case 1:
  // Path exists, no overwrite flag. LogRecord.cc should still overwrite by
  // default behavior whenever the specified path already exists.
  // Path is set by SDF.
  {
    EXPECT_TRUE(common::exists(this->logDir));

    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Log files still exist
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

#ifndef __APPLE__
  // No new files were created
  EXPECT_EQ(2, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Test timestamp is newer
  EXPECT_GT(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
  // Update timestamp for next test
  tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);
#endif

  // Test case 2:
  // Path exists, no overwrite flag. LogRecord.cc should still overwrite by
  // default behavior whenever the specified path already exists.
  // Path is set by C++ API.
  {
    // Pass SDF file to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath(this->logDir);
    recordServerConfig.SetLogIgnoreSdfPath(true);

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Log files still exist
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

#ifndef __APPLE__
  // No new files were created
  EXPECT_EQ(2, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Test timestamp is newer
  EXPECT_GT(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
  // Update timestamp for next test
  tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);
#endif

  // Test case 3:
  // Path exists, no overwrite flag. LogRecord.cc should still overwrite by
  // default behavior whenever the specified path already exists.
  // Path is set by SDF.
  // Server is run from command line, ign.cc should initialize new default
  // timestamp directory, where console log should be recorded. State log should
  // be recorded to the path in SDF.

  // Change environment variable so that test files aren't written to $HOME
  std::string homeOrig;
  common::env(IGN_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeFake.c_str(), 1), 0);

  // Store number of files before running
  auto logPath = common::joinPaths(homeFake.c_str(), ".ignition", "gazebo",
      "log");
#ifndef __APPLE__
  int nEntries = entryCount(logPath);
  std::vector<std::string> entriesBefore;
  entryList(logPath, entriesBefore);

  std::string tmpRecordSdfPath = common::joinPaths(this->logsDir,
    "with_record_path.sdf");

  {
    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        this->logDir);
    EXPECT_EQ(1u, recordSdfRoot.WorldCount());

    // Save changed SDF to temporary file
    // TODO(anyone): Does this work on Apple?
    std::ofstream ofs(tmpRecordSdfPath);
    ofs << recordSdfRoot.Element()->ToString("").c_str();
    ofs.close();

    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + kSdfFileOpt + tmpRecordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // State log file still exists
  EXPECT_TRUE(common::exists(tlogPath));

  // Check the diff of list of files and assume there is a single diff, it
  // being the newly created log directory from the run above.
  EXPECT_EQ(nEntries + 1, entryCount(logPath));
  std::vector<std::string> entriesAfter;
  entryList(logPath, entriesAfter);
  std::vector<std::string> entriesDiff;
  entryDiff(entriesBefore, entriesAfter, entriesDiff);
  EXPECT_EQ(1ul, entriesDiff.size());
  // This should be $HOME/.ignition/..., default path
  std::string timestampPath = entriesDiff[0];

  EXPECT_FALSE(timestampPath.empty());
  EXPECT_EQ(0, timestampPath.compare(0, logPath.length(), logPath));
  EXPECT_TRUE(common::exists(timestampPath));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "server_console.log")));
  EXPECT_EQ(1, entryCount(timestampPath));

  // Cleanup
  common::removeFile(tmpRecordSdfPath);
  common::removeAll(homeFake);
  common::removeAll(timestampPath);

  // Revert environment variable after test is done
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeOrig.c_str(), 1), 0);

  // Test case 4:
  // Path exists, command line --log-overwrite, should overwrite by
  // command-line logic in ign.cc
  {
    // Command line triggers ign.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 --log-overwrite "
      + "--record-path " + this->logDir + " " + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Log files still exist
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

  // No new files were created
  EXPECT_EQ(2, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Test timestamp is newer
  EXPECT_GT(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
  // Update timestamp for next test
  tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);

  // Test case 5:
  // Path exists, no --log-overwrite, should create new files by command-line
  // logic in ign.cc
  {
    // Command line triggers ign.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + this->logDir + " " + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Old log files still exist
  EXPECT_TRUE(common::exists(this->logDir));
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

  // On OS X, ign-gazebo-server (server_main.cc) is being used as opposed to
  // ign gazebo. server_main.cc is deprecated and does not have overwrite
  // renaming implemented. So will always overwrite. Will not test (#) type of
  // renaming on OS X until ign gazebo is fixed:
  // https://github.com/ignitionrobotics/ign-gazebo/issues/25

  // New log files were created
  EXPECT_TRUE(common::exists(this->logDir + "(1)"));
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir + "(1)",
      "state.tlog")));
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir + "(1)",
      "server_console.log")));

  // New files were created
  EXPECT_EQ(3, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));
  EXPECT_EQ(2, entryCount(this->logDir + "(1)"));

  // Old timestamp is same
  EXPECT_EQ(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
#endif

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogControlLevels)
{
  auto logPath = common::joinPaths(PROJECT_SOURCE_PATH, "test", "media",
      "levels_log");

  const auto playSdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_playback.sdf");

  // Change log path in world SDF to build directory
  sdf::Root playSdfRoot;
  this->ChangeLogPath(playSdfRoot, playSdfPath, "LogPlayback",
      logPath);

  ServerConfig config;
  config.SetSdfString(playSdfRoot.Element()->ToString(""));

  Server server(config);

  test::Relay testSystem;

  EntityGraph entityGraph;

  testSystem.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        entityGraph = _ecm.Entities();
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 10, false);

  // store the entities at the beginning of playback
  std::set<uint64_t> entitiesAtTime0;
  for (const auto &v : entityGraph.Vertices())
    entitiesAtTime0.insert(v.first);

  // verify there are entities at the beginning of the playback
  EXPECT_TRUE(!entitiesAtTime0.empty());

  double timeA = 8;
  double timeB = 18;

  transport::Node node;

  // Seek forward
  msgs::LogPlaybackControl req;
  msgs::Boolean res;
  bool result{false};
  unsigned int timeout = 1000;
  std::string service{"/world/default/playback/control"};

  req.mutable_seek()->set_sec(timeA);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run 2 iterations because control messages are processed in the end of an
  // update cycle
  server.Run(true, 2, false);

  // store entities at time A
  std::set<uint64_t> entitiesAtTimeA;
  for (const auto &v : entityGraph.Vertices())
    entitiesAtTimeA.insert(v.first);

  // Seek forward again
  req.mutable_seek()->set_sec(timeB);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run 2 iterations because control messages are processed in the end of an
  // update cycle
  server.Run(true, 2, false);

  // Run another iteration for the view updates to propagate the entity graph
  server.Run(true, 1, false);

  // store entities at time B
  std::set<uint64_t> entitiesAtTimeB;
  for (const auto &v : entityGraph.Vertices())
    entitiesAtTimeB.insert(v.first);

  // the entities at time B should be different from time A as levels get
  // loaded and unloaded
  EXPECT_NE(entitiesAtTimeA.size(), entitiesAtTimeB.size());

  // Seek backward to time A
  req.mutable_seek()->set_sec(timeA);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run 2 iterations because control messages are processed in the end of an
  // update cycle
  server.Run(true, 2, false);

  // Run another iteration for the view updates to propagate to the entity graph
  server.Run(true, 1, false);

  // store another set of entities at time A after jumping back in time
  std::set<uint64_t> entitiesAtTimeAA;
  for (const auto &v : entityGraph.Vertices())
    entitiesAtTimeAA.insert(v.first);

  // verify the entities are the same at time A
  EXPECT_EQ(entitiesAtTimeA.size(), entitiesAtTimeAA.size());
  for (auto aIt = entitiesAtTimeA.begin(), aaIt = entitiesAtTimeAA.begin();
      aIt != entitiesAtTimeA.end() && aaIt != entitiesAtTimeAA.end();
      ++aIt, ++aaIt)
  {
    EXPECT_EQ(*aIt, *aaIt);
  }

  // rewind
  req.Clear();
  req.set_rewind(true);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run 2 iterations because control messages are processed in the end of an
  // update cycle
  server.Run(true, 2, false);

  // Run another iteration for the view updates to propagate to the entity graph
  server.Run(true, 1, false);

  // store another set of entities at time 0 after rewind
  std::set<uint64_t> entitiesAtTime00;
  for (const auto &v : entityGraph.Vertices())
    entitiesAtTime00.insert(v.first);

  // verify the entities are the same at beginning of playback
  EXPECT_EQ(entitiesAtTime0.size(), entitiesAtTime00.size());
  for (auto it = entitiesAtTime0.begin(), it2 = entitiesAtTime00.begin();
      it != entitiesAtTime0.end() && it2 != entitiesAtTime00.end();
      ++it, ++it2)
  {
    EXPECT_EQ(*it, *it2);
  }
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogCompress)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // Define paths
  const std::string recordPath = this->logDir;
  const std::string defaultCmpPath = this->AppendExtension(recordPath,
      ".zip");

  // Record and compress to default path
  {
    // World to record
    const auto recordSdfPath = common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "test", "worlds",
        "log_record_dbl_pendulum.sdf");

    // Change log path in SDF to build directory
    sdf::Root recordSdfRoot;
    this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
        recordPath);

    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set compress flag
    recordServerConfig.SetLogRecordCompressPath(defaultCmpPath);

    // This tells server to call AddRecordPlugin() where flags are passed to
    //   recorder.
    recordServerConfig.SetUseLogRecord(true);

    // Run server
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Test compressed to default directory
  EXPECT_TRUE(common::exists(defaultCmpPath));

  std::string customCmpPath = this->AppendExtension(this->logDir,
      "_custom.zip");

  // Record and compress to custom path
  {
    // World to record
    const auto recordSdfPath = common::joinPaths(
        std::string(PROJECT_SOURCE_PATH), "test", "worlds",
        "log_record_dbl_pendulum.sdf");

    // Get SDF root
    sdf::Root recordSdfRoot;
    recordSdfRoot.Load(recordSdfPath);

    // Pass SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set compress path
    recordServerConfig.SetLogRecordCompressPath(customCmpPath);

    // Set record path
    recordServerConfig.SetLogRecordPath(this->logDir);
    recordServerConfig.SetLogIgnoreSdfPath(true);

    // This tells server to call AddRecordPlugin() where flags are passed to
    //   recorder.
    recordServerConfig.SetUseLogRecord(true);

    // Run server for enough time to record a few poses for playback
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 500, false);
  }

  // Test compressed file exists
  EXPECT_TRUE(common::exists(customCmpPath));

  // Move recorded file to playback directory
  // Prefix the zip by the name of the original recorded folder. Playback will
  // extract and assume subdirectory to take on the name of the zip file
  // without extension.
  auto newCmpPath = common::joinPaths(this->logPlaybackDir,
    common::basename(defaultCmpPath));
  common::moveFile(customCmpPath, newCmpPath);
  EXPECT_TRUE(common::exists(newCmpPath));

  // Play back compressed file
  {
    // World with playback plugin
    const auto playSdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
        "test", "worlds", "log_playback.sdf");

    // Change log path in world SDF to build directory
    sdf::Root playSdfRoot;
    this->ChangeLogPath(playSdfRoot, playSdfPath, "LogPlayback",
        newCmpPath);

    // Pass changed SDF to server
    ServerConfig playServerConfig;
    playServerConfig.SetSdfString(playSdfRoot.Element()->ToString(""));

    // Run server
    Server playServer(playServerConfig);
    playServer.Run(true, 100, false);
  }

  // Remove compressed recorded file. Then the directory should still contain
  // the decompressed files and not be empty
  EXPECT_TRUE(common::removeFile(newCmpPath));

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogCompressOverwrite)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // Define paths
  const std::string recordPath = this->logDir;
  const std::string defaultCmpPath = this->AppendExtension(recordPath,
      ".zip");

  // World to record
  const auto recordSdfPath = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      "log_record_dbl_pendulum.sdf");

  // Change log path in SDF to build directory
  sdf::Root recordSdfRoot;
  this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
      recordPath);

  // Compress + overwrite, recorded directory exists, compressed file does not
  {
    // Create recording directory so that it exists
    common::createDirectories(recordPath);

    // Test case pre-condition
    EXPECT_TRUE(common::exists(recordPath));
    EXPECT_FALSE(common::exists(defaultCmpPath));

    this->RunCompress(recordSdfRoot, defaultCmpPath);
  }

  EXPECT_TRUE(common::exists(defaultCmpPath));
  EXPECT_FALSE(common::exists(recordPath));

  // Compress + overwrite, compressed file exists, recorded directory does not
  {
    // Test case pre-condition
    EXPECT_FALSE(common::exists(recordPath));
    EXPECT_TRUE(common::exists(defaultCmpPath));

    this->RunCompress(recordSdfRoot, defaultCmpPath);
  }

  EXPECT_TRUE(common::exists(defaultCmpPath));
  EXPECT_FALSE(common::exists(recordPath));

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogCompressCmdLine)
{
#ifndef __APPLE__
  // Create temp directory to store log
  this->CreateLogsDir();

  // Define paths
  const std::string recordPath = this->logDir;
  const std::string defaultCmpPath = this->AppendExtension(recordPath,
      ".zip");

  // World to record
  const auto recordSdfPath = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      "log_record_dbl_pendulum.sdf");

  // Change log path in SDF to build directory
  sdf::Root recordSdfRoot;
  this->ChangeLogPath(recordSdfRoot, recordSdfPath, "LogRecord",
      recordPath);

  // Compress only, both recorded directory and compressed file exist
  {
    // Create compressed file
    this->RunCompress(recordSdfRoot, defaultCmpPath);

    // Recreate recording directory so that it exists
    common::createDirectories(recordPath);

    // Test case pre-condition
    EXPECT_TRUE(common::exists(recordPath));
    EXPECT_TRUE(common::exists(defaultCmpPath));

    // Command line triggers ign.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 --log-compress "
      + "--record-path " + recordPath + " "
      + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Original files should still exist
  EXPECT_TRUE(common::exists(recordPath));
  EXPECT_TRUE(common::exists(defaultCmpPath));

  // An automatically renamed file should have been created
  EXPECT_TRUE(common::exists(this->AppendExtension(recordPath, "(1).zip")));
  // Automatically renamed directory should have been removed by record plugin
  EXPECT_FALSE(common::exists(recordPath + "(1)"));

  // Compress only, compressed file exists, auto-renamed compressed file
  // e.g. *(1) exists, recorded directory does not
  {
    // Remove directory if exists
    common::removeAll(recordPath);

    // Test case pre-condition
    EXPECT_TRUE(common::exists(defaultCmpPath));
    EXPECT_FALSE(common::exists(recordPath));
    EXPECT_TRUE(common::exists(this->AppendExtension(recordPath, "(1).zip")));

    // Command line triggers ign.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 --log-compress "
      + "--record-path " + recordPath + " "
      + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Original files should stay same as initial condition
  EXPECT_TRUE(common::exists(defaultCmpPath));
  EXPECT_FALSE(common::exists(recordPath));

  // An automatically renamed file should have been created
  EXPECT_TRUE(common::exists(this->AppendExtension(recordPath, "(2).zip")));

  this->RemoveLogsDir();
#endif
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, LogResources)
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_resources.sdf");

  // Change environment variable so that downloaded fuel files aren't written
  // to $HOME
  std::string homeOrig;
  common::env(IGN_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeFake.c_str(), 1), 0);

  const std::string recordPath = this->logDir;
  std::string statePath = common::joinPaths(recordPath, "state.tlog");

#ifndef __APPLE__
  // Log resources from command line
  {
    // Command line triggers ign.cc, which handles initializing ignLogDirectory
    std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 "
      + "--record --record-resources --record-path " + recordPath + " "
      + kSdfFileOpt + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  std::string consolePath = common::joinPaths(recordPath, "server_console.log");
  EXPECT_TRUE(common::exists(consolePath));
  EXPECT_TRUE(common::exists(statePath));

  // Recorded models should exist
  EXPECT_GT(entryCount(recordPath), 2);
  EXPECT_TRUE(common::exists(common::joinPaths(recordPath, homeFake,
      ".ignition", "fuel", "fuel.ignitionrobotics.org", "OpenRobotics",
      "models", "X2 Config 1")));

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();
#endif

  // Log resources from C++ API
  {
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);

    // Set record flags
    recordServerConfig.SetLogRecordPath(recordPath);
    recordServerConfig.SetLogRecordResources(true);

    // This tells server to call AddRecordPlugin() where flags are passed to
    //   recorder.
    recordServerConfig.SetUseLogRecord(true);

    // Run server
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Console log is not created because ignLogDirectory() is not initialized,
  // as ign.cc is not executed by command line.
  EXPECT_TRUE(common::exists(statePath));

  // Recorded models should exist
#ifndef __APPLE__
  EXPECT_GT(entryCount(recordPath), 1);
#endif
  EXPECT_TRUE(common::exists(common::joinPaths(recordPath, homeFake,
      ".ignition", "fuel", "fuel.ignitionrobotics.org", "OpenRobotics",
      "models", "X2 Config 1")));

  // Revert environment variable after test is done
  EXPECT_EQ(setenv(IGN_HOMEDIR, homeOrig.c_str(), 1), 0);

  // Remove artifacts
  this->RemoveLogsDir();
}
