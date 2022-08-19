/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "gz.hh"

#include <cstring>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/fuel_tools/FuelClient.hh>
#include <gz/fuel_tools/ClientConfig.hh>
#include <gz/fuel_tools/Result.hh>
#include <gz/fuel_tools/WorldIdentifier.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <sdf/Console.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"

#include "gz/sim/gui/Gui.hh"

using namespace gz;

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE char *ignitionGazeboVersion()
{
  return strdup(IGNITION_GAZEBO_VERSION_FULL);
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE char *gazeboVersionHeader()
{
  return strdup(IGNITION_GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE void cmdVerbosity(
    const char *_verbosity)
{
  int verbosity = std::atoi(_verbosity);
  common::Console::SetVerbosity(verbosity);

  // SDFormat only has 2 levels: quiet / loud. Let sim users suppress all SDF
  // console output with zero verbosity.
  if (verbosity == 0)
  {
    sdf::Console::Instance()->SetQuiet(true);
  }
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE const char *worldInstallDir()
{
  return IGN_GAZEBO_WORLD_INSTALL_DIR;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE const char *findFuelResource(
    char *_pathToResource)
{
  std::string path;
  std::string worldPath;
  fuel_tools::FuelClient fuelClient;

  // Attempt to find cached copy, and then attempt download
  if (fuelClient.CachedWorld(common::URI(_pathToResource), path))
  {
    ignmsg << "Cached world found." << std::endl;
    worldPath = path;
  }
  // cppcheck-suppress syntaxError
  else if (fuel_tools::Result result =
    fuelClient.DownloadWorld(common::URI(_pathToResource), path);
    result)
  {
    ignmsg << "Successfully downloaded world from fuel." << std::endl;
    worldPath = path;
  }
  else
  {
    ignwarn << "Fuel world download failed because " << result.ReadableResult()
        << std::endl;
    return "";
  }

  if (!common::exists(worldPath))
    return "";


  // Find the first sdf file in the world path for now, the later intention is
  // to load an optional world config file first and if that does not exist,
  // continue to load the first sdf file found as done below
  for (common::DirIter file(worldPath);
       file != common::DirIter(); ++file)
  {
    std::string current(*file);
    if (common::isFile(current))
    {
      std::string fileName = common::basename(current);
      std::string::size_type fileExtensionIndex = fileName.rfind(".");
      std::string fileExtension = fileName.substr(fileExtensionIndex + 1);

      if (fileExtension == "sdf")
      {
        return strdup(current.c_str());
      }
    }
  }
  return "";
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE int runServer(const char *_sdfString,
    int _iterations, int _run, float _hz, int _levels, const char *_networkRole,
    int _networkSecondaries, int _record, const char *_recordPath,
    int _recordResources, int _logOverwrite, int _logCompress,
    const char *_playback, const char *_physicsEngine,
    const char *_renderEngineServer, const char *_renderEngineGui,
    const char *_file, const char *_recordTopics, int _waitGui)
{
  std::string startingWorldPath{""};
  sim::ServerConfig serverConfig;

  // Lock until the starting world is received from Gui
  if (_waitGui == 1)
  {
    transport::Node node;
    std::condition_variable condition;
    std::mutex mutex;

    // Create a subscriber just so we can check when the message has propagated
    std::function<void(const msgs::StringMsg &)> topicCb =
        [&startingWorldPath, &mutex, &condition](const auto &_msg)
        {
          std::unique_lock<std::mutex> lock(mutex);
          startingWorldPath = _msg.data();
          condition.notify_all();
        };

    std::string topic{"/gazebo/starting_world"};
    std::unique_lock<std::mutex> lock(mutex);
    igndbg << "Subscribing to [" << topic << "]." << std::endl;
    node.Subscribe(topic, topicCb);
    igndbg << "Waiting for a world to be set from the GUI..." << std::endl;
    condition.wait(lock);
    ignmsg << "Received world [" << startingWorldPath << "] from the GUI."
          << std::endl;
    igndbg << "Unsubscribing from [" << topic << "]." << std::endl;
    node.Unsubscribe(topic);
  }

  // Path for logs
  std::string recordPathMod = serverConfig.LogRecordPath();

  // Path for compressed log, used to check for duplicates
  std::string cmpPath = std::string(recordPathMod);
  if (!std::string(1, cmpPath.back()).compare(common::separator("")))
  {
    // Remove the separator at end of path
    cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
  }
  cmpPath += ".zip";

  // Initialize console log
  if ((_recordPath != nullptr && std::strlen(_recordPath) > 0) ||
    _record > 0 || _recordResources > 0 ||
    (_recordTopics != nullptr && std::strlen(_recordTopics) > 0))
  {
    if (_playback != nullptr && std::strlen(_playback) > 0)
    {
      ignerr << "Both record and playback are specified. Only specify one.\n";
      return -1;
    }

    serverConfig.SetUseLogRecord(true);
    serverConfig.SetLogRecordResources(_recordResources);

    // If a record path is specified
    if (_recordPath != nullptr && std::strlen(_recordPath) > 0)
    {
      recordPathMod = std::string(_recordPath);

      // Update compressed file path to name of recording directory path
      cmpPath = std::string(recordPathMod);
      if (!std::string(1, cmpPath.back()).compare(common::separator(
        "")))
      {
        // Remove the separator at end of path
        cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
      }
      cmpPath += ".zip";

      // Check if path or compressed file with same prefix exists
      if (common::exists(recordPathMod) ||
        common::exists(cmpPath))
      {
        // Overwrite if flag specified
        if (_logOverwrite > 0)
        {
          bool recordMsg = false;
          bool cmpMsg = false;
          // Remove files before initializing console log files on top of them
          if (common::exists(recordPathMod))
          {
            recordMsg = true;
            common::removeAll(recordPathMod);
          }
          if (common::exists(cmpPath))
          {
            cmpMsg = true;
            common::removeFile(cmpPath);
          }

          // Create log file before printing any messages so they can be logged
          ignLogInit(recordPathMod, "server_console.log");

          if (recordMsg)
          {
            ignmsg << "Log path already exists on disk! Existing files will "
              << "be overwritten." << std::endl;
            ignmsg << "Removing existing path [" << recordPathMod << "]\n";
          }
          if (cmpMsg)
          {
            if (_logCompress > 0)
            {
              ignwarn << "Compressed log path already exists on disk! Existing "
                << "files will be overwritten." << std::endl;
            }
            ignmsg << "Removing existing compressed file [" << cmpPath << "]\n";
          }
        }
        // Otherwise rename to unique path
        else
        {
          // Remove the separator at end of path
          if (!std::string(1, recordPathMod.back()).compare(
            common::separator("")))
          {
            recordPathMod = recordPathMod.substr(0, recordPathMod.length()
              - 1);
          }

          std::string recordOrigPrefix = std::string(recordPathMod);
          int count = 1;

          // Keep renaming until path does not exist for both directory and
          // compressed file
          while (common::exists(recordPathMod) ||
            common::exists(cmpPath))
          {
            recordPathMod = recordOrigPrefix +  "(" + std::to_string(count++) +
              ")";

            cmpPath = std::string(recordPathMod);
            // Remove the separator at end of path
            if (!std::string(1, cmpPath.back()).compare(
              common::separator("")))
            {
              cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
            }
            cmpPath += ".zip";
          }

          ignLogInit(recordPathMod, "server_console.log");
          ignwarn << "Log path already exists on disk! "
            << "Recording instead to [" << recordPathMod << "]" << std::endl;
          if (_logCompress > 0)
          {
            ignwarn << "Compressed log path already exists on disk! "
              << "Recording instead to [" << cmpPath << "]" << std::endl;
          }
        }
      }
      else
      {
        ignLogInit(recordPathMod, "server_console.log");
      }
      // TODO(anyone) In Ignition-D, to be moved to outside and after this
      //   if-else statement, after all ignLogInit() calls have been finalized,
      //   so that <path> in SDF will always be ignored in favor of logging both
      //   console logs and LogRecord recordings to common::ignLogDirectory().
      //   In Blueprint and Citadel, LogRecord will record to <path> if no
      //   --record-path is specified on command line.
      serverConfig.SetLogRecordPath(recordPathMod);
      serverConfig.SetLogIgnoreSdfPath(true);
    }
    // Empty record path specified. Use default.
    else
    {
      // Create log file before printing any messages so they can be logged
      ignLogInit(recordPathMod, "server_console.log");
      ignmsg << "Recording states to default path [" << recordPathMod << "]"
             << std::endl;

      serverConfig.SetLogRecordPath(recordPathMod);
    }

    std::vector<std::string> topics = common::split(
        _recordTopics, ":");
    for (const std::string &topic : topics)
    {
      serverConfig.AddLogRecordTopic(topic);
    }
  }
  else
  {
    ignLogInit(serverConfig.LogRecordPath(), "server_console.log");
  }

  if (_logCompress > 0)
  {
    serverConfig.SetLogRecordCompressPath(cmpPath);
  }

  ignmsg << "Ignition Gazebo Server v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Set the SDF string to user
  if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
  {
    if (!serverConfig.SetSdfString(_sdfString))
    {
      ignerr << "Failed to set SDF string [" << _sdfString << "]" << std::endl;
      return -1;
    }
  }

  // This ensures if the server was run stand alone with a world from
  // command line, the correct world would be loaded.
  if(_waitGui == 1)
    serverConfig.SetSdfFile(startingWorldPath);
  else
    serverConfig.SetSdfFile(_file);

  // Set the update rate.
  if (_hz > 0.0)
    serverConfig.SetUpdateRate(_hz);

  // Set whether levels should be used.
  if (_levels > 0)
  {
    ignmsg << "Using the level system\n";
    serverConfig.SetUseLevels(true);
  }

  if (_networkRole && std::strlen(_networkRole) > 0)
  {
    ignmsg << "Using the distributed simulation and levels systems\n";
    serverConfig.SetNetworkRole(_networkRole);
    serverConfig.SetNetworkSecondaries(_networkSecondaries);
    serverConfig.SetUseLevels(true);
  }

  if (_playback != nullptr && std::strlen(_playback) > 0)
  {
    if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
    {
      ignerr << "Both an SDF file and playback flag are specified. "
        << "Only specify one.\n";
      return -1;
    }
    else
    {
      ignmsg << "Playing back states" << _playback << std::endl;
      serverConfig.SetLogPlaybackPath(common::absPath(
        std::string(_playback)));
    }
  }

  if (_physicsEngine != nullptr && std::strlen(_physicsEngine) > 0)
  {
    serverConfig.SetPhysicsEngine(_physicsEngine);
  }

  if (_renderEngineServer != nullptr && std::strlen(_renderEngineServer) > 0)
  {
    serverConfig.SetRenderEngineServer(_renderEngineServer);
  }

  if (_renderEngineGui != nullptr && std::strlen(_renderEngineGui) > 0)
  {
    serverConfig.SetRenderEngineGui(_renderEngineGui);
  }

  // Create the Gazebo server
  sim::Server server(serverConfig);

  // Run the server
  server.Run(true, _iterations, _run == 0);

  igndbg << "Shutting down ign-gazebo-server" << std::endl;
  return 0;
}

//////////////////////////////////////////////////
extern "C" IGNITION_GAZEBO_VISIBLE int runGui(
  const char *_guiConfig, const char *_file, int _waitGui)
{
  // argc and argv are going to be passed to a QApplication. The Qt
  // documentation has a warning about these:
  //  "Warning: The data referred to by argc and argv must stay valid for the
  //  entire lifetime of the QApplication object. In addition, argc must be
  //  greater than zero and argv must contain at least one valid character
  //  string."
  int argc = 1;
  // Converting a string literal to char * is forbidden as of C++11. It can only
  // be converted to a const char *. The const cast is here to prevent a warning
  // since we do need to pass a char* to runGui
  char *argv = const_cast<char *>("ign-gazebo-gui");
  return sim::gui::runGui(
    argc, &argv, _guiConfig, _file, _waitGui);
}
