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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/log_playback_control.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/serialized_map.pb.h>

#include <algorithm>
#include <climits>
#ifndef __APPLE__
#include <filesystem>
#endif
#include <numeric>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/Filesystem.hh>
#include <gz/fuel_tools/Zip.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/log/Batch.hh>
#include <gz/transport/log/Log.hh>
#include <gz/transport/log/MsgIter.hh>
#include <gz/transport/log/Playback.hh>
#include <gz/transport/log/QualifiedTime.hh>
#include <gz/math/Pose3.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/LogPlaybackStatistics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

static const std::string kBinPath(PROJECT_BINARY_PATH);

// \todo(anyone) Enable tests for OSX once command line works there
#ifndef __APPLE__
static const std::string kGzCommand(
  "GZ_SIM_SYSTEM_PLUGIN_PATH=" + kBinPath + "/lib LD_LIBRARY_PATH=" +
  kBinPath + "/lib:/usr/local/lib:${LD_LIBRARY_PATH} gz sim -s ");
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
class LogSystemTest : public InternalFixture<::testing::Test>
{
  // Create a temporary directory in build path for recorded data
  public: void CreateLogsDir()
  {
    // Configure to use binary path as cache
    if (common::exists(this->logsDir))
    {
      common::removeAll(this->logsDir);
    }
    common::createDirectories(this->logsDir);
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
  // \param[in] _recordPath Path for SDF state file
  // \param[in] _cmpPath Path for compressed file
  public: void RunCompress(const std::string &_recordSdfPath,
    const std::string &_recordPath, const std::string &_cmpPath)
  {
    // Pass changed SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(_recordSdfPath);

    // Set record path
    recordServerConfig.SetLogRecordPath(_recordPath);

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
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogPlaybackStatistics))
{
  // TODO(anyone) see LogSystemTest.LogControl comment about re-recording
  auto logPath = common::joinPaths(PROJECT_SOURCE_PATH, "test", "media",
      "rolling_shapes_log");

  ServerConfig config;
  config.SetLogPlaybackPath(logPath);

  Server server(config);

  test::Relay testSystem;
  std::chrono::steady_clock::time_point startTime;
  std::chrono::steady_clock::time_point endTime;
  testSystem.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        _ecm.Each<components::LogPlaybackStatistics>(
            [&](const Entity &,
                const components::LogPlaybackStatistics *_logStatComp)->bool
            {
              auto startSeconds =
                _logStatComp->Data().start_time().sec();
              auto startNanoseconds =
                _logStatComp->Data().start_time().nsec();
              auto endSeconds =
                _logStatComp->Data().end_time().sec();
              auto endNanoseconds =
                _logStatComp->Data().end_time().nsec();
              startTime =
                math::secNsecToTimePoint(startSeconds, startNanoseconds);
              endTime =
                math::secNsecToTimePoint(endSeconds, endNanoseconds);
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, 10, false);

  auto startTimePair = math::timePointToSecNsec(startTime);
  auto endTimePair = math::timePointToSecNsec(endTime);

  EXPECT_EQ(0, startTimePair.first);
  EXPECT_EQ(1000000, startTimePair.second);
  EXPECT_EQ(5, endTimePair.first);
  EXPECT_EQ(0, endTimePair.second);
}

/////////////////////////////////////////////////
// Logging behavior when no paths are specified
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogDefaults))
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // Change environment variable so that test files aren't written to $HOME
  std::string homeOrig;
  common::env(GZ_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeFake.c_str()));

  // Test case 1:
  // No path specified on command line. This does not go through
  // gz.cc, recording should take place in the `.gz` directory
  {
    // Load SDF
    sdf::Root recordSdfRoot;
    EXPECT_EQ(recordSdfRoot.Load(recordSdfPath).size(), 0lu);
    EXPECT_GT(recordSdfRoot.WorldCount(), 0lu);

    // Pass SDF to server
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfString(recordSdfRoot.Element()->ToString(""));

    // Set record path to empty
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath("");

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 200, false);
  }

  // We should expect to see "auto_default.log"  and "state.tlog"
  EXPECT_FALSE(gzLogDirectory().empty());
  EXPECT_TRUE(common::exists(
        common::joinPaths(gzLogDirectory(), "auto_default.log")));
  EXPECT_TRUE(common::exists(
        common::joinPaths(gzLogDirectory(), "state.tlog")));

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();

#ifndef __APPLE__
  this->CreateLogsDir();

  // Test case 2:
  // No path specified on command line (only --record, no --record-path).
  // No path specified in SDF.
  // Run from command line, which should trigger gz.cc, which should initialize
  // gzLogDirectory() to default timestamp path. Both console and state logs
  // should be recorded here.

  // Store number of files before running
  auto logPath = common::joinPaths(homeFake.c_str(), ".gz", "sim",
      "log");
  int nEntries = entryCount(logPath);
  std::vector<std::string> entriesBefore;
  entryList(logPath, entriesBefore);

  {
    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record " + recordSdfPath;
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
  // This should be $HOME/.gz/..., default path
  std::string timestampPath = entriesDiff[0];

  EXPECT_FALSE(timestampPath.empty());
  EXPECT_EQ(0, timestampPath.compare(0, logPath.length(), logPath));
  EXPECT_TRUE(common::exists(timestampPath));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "server_console.log")));
  EXPECT_TRUE(common::exists(common::joinPaths(timestampPath,
      "state.tlog")));
  EXPECT_EQ(2, entryCount(timestampPath));

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
#endif

  // Revert environment variable after test is done
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeOrig.c_str()));
}

/////////////////////////////////////////////////
// Logging behavior when a path is specified either via the C++ API, SDF, or
// the command line.
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogPaths))
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  // Test case 1:
  // A path is specified in SDF - a feature removed in Gazebo Dome.
  // No path specified in C++ API.
  // Should ignore SDF path. No default logging directory is initialized for
  // state and console logs because gz.cc is not triggered.
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

  // Check state.tlog is no longer stored to path specified in SDF
  auto stateFile = common::joinPaths(this->logDir, "state.tlog");
  EXPECT_FALSE(common::exists(stateFile)) << stateFile;
#ifndef __APPLE__
  EXPECT_EQ(0, entryCount(this->logDir));
#endif

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Change environment variable so that test files aren't written to $HOME
  std::string homeOrig;
  common::env(GZ_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeFake.c_str()));

  // Store number of files before running
  auto logPath = common::joinPaths(homeFake.c_str(), ".gz", "sim",
      "log");
#ifndef __APPLE__
  int nEntries = entryCount(logPath);
  std::vector<std::string> entriesBefore;
  entryList(logPath, entriesBefore);

  // Test case 2:
  // A path is specified in SDF - a feature removed in Gazebo Dome.
  // SDF path should be ignored.
  // State log and console log should be stored to default timestamp path
  // gzLogDirectory because gz.cc is triggered by command line.
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

    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record " + tmpRecordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Check state.tlog is no longer stored to path specified in SDF
  EXPECT_FALSE(common::exists(common::joinPaths(this->logDir,
      "state.tlog")));
  EXPECT_EQ(0, entryCount(this->logDir));

  // Check the diff of list of files in directory, and assume there is
  // a single diff, it being the newly created log directory from the run above.
  EXPECT_EQ(nEntries + 1, entryCount(logPath));
  std::vector<std::string> entriesAfter;
  entryList(logPath, entriesAfter);
  std::vector<std::string> entriesDiff;
  entryDiff(entriesBefore, entriesAfter, entriesDiff);
  EXPECT_EQ(1ul, entriesDiff.size());
  // This should be $HOME/.gz/..., default path
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

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 3:
  // A path is specified in SDF - a feature removed in Gazebo Dome.
  // Empty path is specified via C++ API.
  // Should ignore SDF path. No default logging directory is initialized for
  // state and console logs because gz.cc is not triggered.
  std::string stateLogPath = this->logDir;

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

  EXPECT_FALSE(common::exists(common::joinPaths(stateLogPath, "state.tlog")));
#ifndef __APPLE__
  EXPECT_EQ(0, entryCount(stateLogPath));
#endif

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();

  // Test case 4:
  // A path is specified in SDF - a feature removed in Gazebo Dome.
  // A different path is specified via C++ API.
  // Should take C++ API path. State log should be stored here. Console log is
  // not initialized because gz.cc is not triggered.
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
    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + this->logDir + " " + recordSdfPath;
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
  // A path is specified in SDF - a feature removed in Gazebo Dome.
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

    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + cliPath + " " + tmpRecordSdfPath;
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
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeOrig.c_str()));

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(RecordAndPlayback))
{
  // Create temp directory to store log
  this->CreateLogsDir();

  int numIterations = 1000;
  // Record
  {
    // World with moving entities
    const auto recordSdfPath = common::joinPaths(
      std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      "log_record_dbl_pendulum.sdf");

    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath(this->logDir);

    // Run for a number of iterations to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, numIterations, false);
  }

  // Verify file is created
  auto logFile = common::joinPaths(this->logDir, "state.tlog");
  EXPECT_TRUE(common::exists(logFile));

  // Path to log file for playback
  std::string logPlaybackDir =
      common::joinPaths(this->logsDir, "test_logs_playback");
  common::createDirectories(logPlaybackDir);

  // Move the log file to the playback directory
  auto logPlaybackFile = common::joinPaths(logPlaybackDir, "state.tlog");
  common::moveFile(logFile, logPlaybackFile);
  EXPECT_TRUE(common::exists(logPlaybackFile));

  // Load log file recorded above for validation
  transport::log::Log log;
  log.Open(logPlaybackFile);

  // Check it has SDF message
  auto batch = log.QueryMessages(transport::log::TopicPattern(
      std::regex(".*/sdf")));
  auto recordedIter = batch.begin();
  EXPECT_NE(batch.end(), recordedIter);

  EXPECT_EQ("gz.msgs.StringMsg", recordedIter->Type());
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

  EXPECT_EQ("gz.msgs.SerializedStateMap", recordedIter->Type());
  EXPECT_EQ(recordedIter->Topic(), "/world/log_pendulum/changed_state");

  msgs::SerializedStateMap stateMsg;
  stateMsg.ParseFromString(recordedIter->Data());
  // entity size = 28 in dbl pendulum + 4 in nested model
  EXPECT_EQ(33, stateMsg.entities_size());
  EXPECT_NE(batch.end(), ++recordedIter);

  // Playback config
  ServerConfig playServerConfig;
  playServerConfig.SetLogPlaybackPath(logPlaybackDir);

  // Start server
  Server playServer(playServerConfig);

  // Simulate a client
  gz::transport::Node node;
  std::atomic<std::size_t> numMsgs = 0;
  std::function<void(const msgs::SerializedStepMap &)> mockClient =
    [&](const msgs::SerializedStepMap &/*_res*/)
  {
    numMsgs++;
  };
  using namespace std::placeholders;
  EXPECT_TRUE(node.Subscribe(
    "/world/default/state", mockClient));

  // Callback function for entities played back
  // Compare current pose being played back with the pose from the stateMsg
  test::Relay playbackPoseTester;
  playbackPoseTester.OnPostUpdate(
      [&](const UpdateInfo &, const EntityComponentManager &_ecm)
      {
        // Playback continues even after the log file is over
        if (batch.end() == recordedIter)
          return;

        // Get next recorded message
        EXPECT_EQ("gz.msgs.SerializedStateMap", recordedIter->Type());
        EXPECT_EQ(recordedIter->Topic(), "/world/log_pendulum/changed_state");
        stateMsg.ParseFromString(recordedIter->Data());

        // Loop through all recorded poses, and check them against the
        // playedback poses.
        for (const auto &entityIter : stateMsg.entities())
        {
          Entity entity = entityIter.second.id();
          auto nameComp = _ecm.Component<components::Name>(entity);
          ASSERT_NE(nullptr, nameComp);
          const std::string &name = nameComp->Data();

          auto poseComp = _ecm.Component<components::Pose>(entity);
          ASSERT_NE(nullptr, poseComp);
          const math::Pose3d &posePlayed = poseComp->Data();

          for (const auto &compIter : entityIter.second.components())
          {
            msgs::SerializedComponent compMsg = compIter.second;
            ASSERT_EQ(components::Pose::typeId, compMsg.type());

            components::Pose pose;
            std::istringstream istr(compMsg.component());
            pose.Deserialize(istr);
            const math::Pose3d &poseRecorded = pose.Data();

            EXPECT_NEAR(posePlayed.Pos().X(), poseRecorded.Pos().X(), 0.1)
              << name;
            EXPECT_NEAR(posePlayed.Pos().Y(), poseRecorded.Pos().Y(), 0.1)
              << name;
            EXPECT_NEAR(posePlayed.Pos().Z(), poseRecorded.Pos().Z(), 0.1)
              << name;

            EXPECT_NEAR(posePlayed.Rot().Roll(),
                        poseRecorded.Rot().Roll(), 0.1) << name;
            EXPECT_NEAR(posePlayed.Rot().Pitch(),
                        poseRecorded.Rot().Pitch(), 0.1) << name;
            EXPECT_NEAR(posePlayed.Rot().Yaw(),
                        poseRecorded.Rot().Yaw(), 0.1) << name;

          }
        }
        ++recordedIter;
      });

  playServer.AddSystem(playbackPoseTester.systemPtr);

  // Playback a subset of the log file and check for poses. Note: the poses are
  // checked in the playbackPoseTester
  playServer.Run(true, 500, false);

  // The client should have received some messages.
  EXPECT_NE(numMsgs, 0);

  // Count the total number of state messages in the log file
  int nTotal{0};
  for (auto it = batch.begin(); it != batch.end(); ++it, ++nTotal) { }

  EXPECT_EQ(numIterations, nTotal);

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogControl))
{
  // TODO(anyone) when re-recording state.tlog file, do not run
  // `gz sim --record rolling_shapes.sdf` with `-r` flag and pause sim
  // before terminating. For some reason, when running with `-r` &/or
  // terminating sim w/o pausing causing strange pose behavior
  // when seeking close to end of file followed by rewind. For more details:
  // https://github.com/gazebosim/gz-sim/pull/839
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
  std::vector<int> secs(5);
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

  // Rewind to zero
  req.Clear();
  req.set_rewind(true);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 3, false);

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
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogOverwrite))
{
  // Create temp directory to store log
  this->CreateLogsDir();
#ifndef __APPLE__
  EXPECT_EQ(0, entryCount(this->logsDir));
  EXPECT_EQ(0, entryCount(this->logDir));
#endif

  // World with moving entities
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_dbl_pendulum.sdf");

  gzLogInit(this->logDir, "server_console.log");
  EXPECT_EQ(this->logDir, gzLogDirectory());

  // Record something to create some files
  {
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath(this->logDir);

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
  EXPECT_EQ(1, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));
  std::filesystem::path tlogStdPath = tlogPath;
  auto tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);
#endif

  // Test case 1:
  // Path exists, no overwrite flag. LogRecord.cc should still overwrite by
  // default behavior whenever the specified path already exists.
  {
    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath(this->logDir);

    // Run for a few seconds to record different poses
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 100, false);
  }

  // Log files still exist
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

#ifndef __APPLE__
  // No new files were created
  EXPECT_EQ(1, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Test timestamp is newer
  EXPECT_GT(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
  // Update timestamp for next test
  tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);

  // Test case 2:
  // Path exists, command line --log-overwrite, should overwrite by
  // command-line logic in gz.cc
  {
    // Command line triggers gz.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 --log-overwrite "
      + "--record-path " + this->logDir + " " + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Log files still exist
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

  // No new files were created
  EXPECT_EQ(1, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));

  // Test timestamp is newer
  EXPECT_GT(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
  // Update timestamp for next test
  tlogPrevTime = std::filesystem::last_write_time(tlogStdPath);

  // Test case 3:
  // Path exists, no --log-overwrite, should create new files by command-line
  // logic in gz.cc
  {
    // Command line triggers gz.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record-path " + this->logDir + " " + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  // Old log files still exist
  EXPECT_TRUE(common::exists(this->logDir));
  EXPECT_TRUE(common::exists(tlogPath));
  EXPECT_TRUE(common::exists(clogPath));

  // On OS X, gz-sim-server (server_main.cc) is being used as opposed to
  // gz sim. server_main.cc is deprecated and does not have overwrite
  // renaming implemented. So will always overwrite. Will not test (#) type of
  // renaming on OS X until gz sim is fixed:
  // https://github.com/gazebosim/gz-sim/issues/25

  // New log files were created
  EXPECT_TRUE(common::exists(this->logDir + "(1)"));
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir + "(1)",
      "state.tlog")));
  EXPECT_TRUE(common::exists(common::joinPaths(this->logDir + "(1)",
      "server_console.log")));

  // New files were created
  EXPECT_EQ(2, entryCount(this->logsDir));
  EXPECT_EQ(2, entryCount(this->logDir));
  EXPECT_EQ(2, entryCount(this->logDir + "(1)"));

  // Old timestamp is same
  EXPECT_EQ(std::filesystem::last_write_time(tlogStdPath), tlogPrevTime);
#endif

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
// TODO(chapulina) Record updated log
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogControlLevels))
{
  auto logPath = common::joinPaths(PROJECT_SOURCE_PATH, "test", "media",
      "levels_log");

  ServerConfig config;
  config.SetLogPlaybackPath(logPath);

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

  int timeA = 8;
  int timeB = 18;

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
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogCompress))
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

    ServerConfig recordServerConfig;
    recordServerConfig.SetSdfFile(recordSdfPath);
    recordServerConfig.SetUseLogRecord(true);
    recordServerConfig.SetLogRecordPath(recordPath);

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

    // This tells server to call AddRecordPlugin() where flags are passed to
    //   recorder.
    recordServerConfig.SetUseLogRecord(true);

    // Run server for enough time to record a few poses for playback
    Server recordServer(recordServerConfig);
    recordServer.Run(true, 500, false);
  }

  // Test compressed file exists
  EXPECT_TRUE(common::exists(customCmpPath));

  // Path to log file for playback
  std::string logPlaybackDir =
      common::joinPaths(this->logsDir, "test_logs_playback");
  common::createDirectories(logPlaybackDir);

  // Move recorded file to playback directory
  // Prefix the zip by the name of the original recorded folder. Playback will
  // extract and assume subdirectory to take on the name of the zip file
  // without extension.
  auto newCmpPath = common::joinPaths(logPlaybackDir,
    common::basename(defaultCmpPath));
  common::moveFile(customCmpPath, newCmpPath);
  EXPECT_TRUE(common::exists(newCmpPath));

  // Play back compressed file
  {
    // Pass changed SDF to server
    ServerConfig playServerConfig;
    playServerConfig.SetLogPlaybackPath(newCmpPath);

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
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogCompressOverwrite))
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

  // Compress + overwrite, recorded directory exists, compressed file does not
  {
    // Create recording directory so that it exists
    common::createDirectories(recordPath);

    // Test case pre-condition
    EXPECT_TRUE(common::exists(recordPath));
    EXPECT_FALSE(common::exists(defaultCmpPath));

    this->RunCompress(recordSdfPath, recordPath, defaultCmpPath);
  }

  EXPECT_TRUE(common::exists(defaultCmpPath));
  EXPECT_FALSE(common::exists(recordPath));

  // Compress + overwrite, compressed file exists, recorded directory does not
  {
    // Test case pre-condition
    EXPECT_FALSE(common::exists(recordPath));
    EXPECT_TRUE(common::exists(defaultCmpPath));

    this->RunCompress(recordSdfPath, recordPath, defaultCmpPath);
  }

  EXPECT_TRUE(common::exists(defaultCmpPath));
  EXPECT_FALSE(common::exists(recordPath)) << recordPath;

  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogCompressCmdLine))
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

  // Compress only, both recorded directory and compressed file exist
  {
    // Create compressed file
    this->RunCompress(recordSdfPath, recordPath, defaultCmpPath);

    // Recreate recording directory so that it exists
    common::createDirectories(recordPath);

    // Test case pre-condition
    EXPECT_TRUE(common::exists(recordPath));
    EXPECT_TRUE(common::exists(defaultCmpPath));

    // Command line triggers gz.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 --log-compress "
      + "--record-path " + recordPath + " " + recordSdfPath;
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

    // Command line triggers gz.cc, which handles creating a unique path if
    // file already exists, so as to not overwrite
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 --log-compress "
      + "--record-path " + recordPath + " " + recordSdfPath;
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
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogResources))
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
  common::env(GZ_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeFake.c_str()));

  const std::string recordPath = this->logDir;
  std::string statePath = common::joinPaths(recordPath, "state.tlog");

#ifndef __APPLE__
  // Log resources from command line
  {
    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record --record-resources --record-path " + recordPath + " "
      + recordSdfPath;
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
      ".gz", "fuel", "fuel.gazebosim.org", "openrobotics",
      "models", "x2 config 1")));

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

  // Console log is not created because gzLogDirectory() is not initialized,
  // as gz.cc is not executed by command line.
  EXPECT_TRUE(common::exists(statePath));

  // Recorded models should exist
#ifndef __APPLE__
  EXPECT_GT(entryCount(recordPath), 1);
#endif
  EXPECT_TRUE(common::exists(common::joinPaths(recordPath, homeFake,
      ".gz", "fuel", "fuel.gazebosim.org", "openrobotics",
      "models", "x2 config 1")));

  // Revert environment variable after test is done
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeOrig.c_str()));

  // Remove artifacts
  this->RemoveLogsDir();
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(LogTopics))
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
  common::env(GZ_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeFake.c_str()));

  const std::string recordPath = this->logDir;
  std::string statePath = common::joinPaths(recordPath, "state.tlog");

#ifndef __APPLE__
  // Log only the /clock topic from command line
  {
    // Command line triggers gz.cc, which handles initializing gzLogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations 5 "
      + "--record-topic /clock "
      + "--record-path " + recordPath + " "
      + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  std::string consolePath = common::joinPaths(recordPath, "server_console.log");
  EXPECT_TRUE(common::exists(consolePath)) << consolePath;
  EXPECT_TRUE(common::exists(statePath)) << statePath;

  // Recorded models should exist
  EXPECT_GT(entryCount(recordPath), 1);

  // Load the state log file into a player.
  transport::log::Playback player(statePath);
  const int64_t addTopicResult = player.AddTopic(std::regex(".*"));

  // There should be 3 topics (clock, sdf, & state)
  EXPECT_EQ(3, addTopicResult);

  int clockMsgCount = 0;
  std::function<void(const msgs::Clock &)> clockCb =
      [&](const msgs::Clock &) -> void
  {
    clockMsgCount++;
  };

  // Subscribe to the clock topic
  transport::Node node;
  node.Subscribe("/clock", clockCb);

  // Begin playback
  transport::log::PlaybackHandlePtr handle =
    player.Start(std::chrono::seconds(5), false);
  handle->WaitUntilFinished();

  // There were five iterations of simulation, so there should be 5 clock
  // messages.
  EXPECT_EQ(5, clockMsgCount);

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();
#endif
}

/////////////////////////////////////////////////
TEST_F(LogSystemTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(RecordPeriod))
{
  // Create temp directory to store log
  this->CreateLogsDir();

  // test world
  const auto recordSdfPath = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
    "log_record_resources.sdf");

  // Change environment variable so that downloaded fuel files aren't written
  // to $HOME
  std::string homeOrig;
  common::env(GZ_HOMEDIR, homeOrig);
  std::string homeFake = common::joinPaths(this->logsDir, "default");
  EXPECT_TRUE(common::setenv(GZ_HOMEDIR, homeFake.c_str()));

  const std::string recordPath = this->logDir;
  std::string statePath = common::joinPaths(recordPath, "state.tlog");

  int numIterations = 100;
#ifndef __APPLE__
  // Log from command line
  {
    // Command line triggers gz.cc, which handles initializing gzlogDirectory
    std::string cmd = kGzCommand + " -r -v 4 --iterations "
      + std::to_string(numIterations) + " "
      + "--record-period 0.002 "
      + "--record-path " + recordPath + " " + recordSdfPath;
    std::cout << "Running command [" << cmd << "]" << std::endl;

    // Run
    std::string output = customExecStr(cmd);
    std::cout << output << std::endl;
  }

  std::string consolePath = common::joinPaths(recordPath, "server_console.log");
  EXPECT_TRUE(common::exists(consolePath)) << consolePath;
  EXPECT_TRUE(common::exists(statePath)) << statePath;

  // Recorded models should exist
  EXPECT_GT(entryCount(recordPath), 1);

  // Verify file is created
  auto logFile = common::joinPaths(recordPath, "state.tlog");
  EXPECT_TRUE(common::exists(logFile));

  // Load the state log file into a player.
  transport::log::Playback player(statePath);
  const int64_t addTopicResult = player.AddTopic(std::regex(".*"));

  // There should be 2 topics (sdf, & state)
  EXPECT_EQ(2, addTopicResult);

  int msgCount = 0;
  std::function<void(const msgs::SerializedStateMap &)> stateCb =
      [&](const msgs::SerializedStateMap &) -> void
  {
    msgCount++;
  };

  // Subscribe to the state topic
  transport::Node node;
  node.Subscribe("/world/default/changed_state", stateCb);

  // Begin playback
  transport::log::PlaybackHandlePtr handle =
    player.Start(std::chrono::seconds(5), false);
  handle->WaitUntilFinished();

  // There were 100 iterations of simulation, and we were recording at 2ms
  // so there should be 50 state messages.
  EXPECT_EQ(50, msgCount);

  // Remove artifacts. Recreate new directory
  this->RemoveLogsDir();
  this->CreateLogsDir();
#endif
}
