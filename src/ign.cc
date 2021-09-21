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

#include "ign.hh"

#include <cstring>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/ClientConfig.hh>
#include <ignition/fuel_tools/Result.hh>
#include <ignition/fuel_tools/WorldIdentifier.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

#include "ignition/gazebo/gui/Gui.hh"

//////////////////////////////////////////////////
extern "C" char *ignitionGazeboVersion()
{
  return strdup(IGNITION_GAZEBO_VERSION_FULL);
}

//////////////////////////////////////////////////
extern "C" char *gazeboVersionHeader()
{
  return strdup(IGNITION_GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
extern "C" void cmdVerbosity(
    const char *_verbosity)
{
  ignition::common::Console::SetVerbosity(std::atoi(_verbosity));
}

//////////////////////////////////////////////////
extern "C" const char *worldInstallDir()
{
  return IGN_GAZEBO_WORLD_INSTALL_DIR;
}

//////////////////////////////////////////////////
extern "C" const char *findFuelResource(
    char *_pathToResource)
{
  std::string path;
  std::string worldPath;
  ignition::fuel_tools::FuelClient fuelClient;

  // Attempt to find cached copy, and then attempt download
  if (fuelClient.CachedWorld(ignition::common::URI(_pathToResource), path))
  {
    ignmsg << "Cached world found." << std::endl;
    worldPath = path;
  }
  // cppcheck-suppress syntaxError
  else if (ignition::fuel_tools::Result result =
    fuelClient.DownloadWorld(ignition::common::URI(_pathToResource), path);
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

  if (!ignition::common::exists(worldPath))
    return "";


  // Find the first sdf file in the world path for now, the later intention is
  // to load an optional world config file first and if that does not exist,
  // continue to load the first sdf file found as done below
  for (ignition::common::DirIter file(worldPath);
       file != ignition::common::DirIter(); ++file)
  {
    std::string current(*file);
    if (ignition::common::isFile(current))
    {
      std::string fileName = ignition::common::basename(current);
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
// Populate _serverConfig with the values from the other inputs.
int createServerConfig(ignition::gazebo::ServerConfig &_serverConfig,
    const char *_sdfString,
    float _hz, int _levels, const char *_networkRole,
    int _networkSecondaries, int _record, const char *_recordPath,
    int _recordResources, int _logOverwrite, int _logCompress,
    const char *_playback, const char *_physicsEngine,
    const char *_renderEngineServer, const char *_renderEngineGui,
    const char *_file, const char *_recordTopics, bool _sameProcessAsGUI
    const char *_file, const char *_recordTopics,
    int _headless, bool _sameProcessAsGUI)
{
  // Path for logs
  std::string recordPathMod = _serverConfig.LogRecordPath();

  _serverConfig.SetSameProcessAsGUI(_sameProcessAsGUI);

  // Path for compressed log, used to check for duplicates
  std::string cmpPath = std::string(recordPathMod);
  if (!std::string(1, cmpPath.back()).compare(ignition::common::separator("")))
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

    _serverConfig.SetUseLogRecord(true);
    _serverConfig.SetLogRecordResources(_recordResources);

    // If a record path is specified
    if (_recordPath != nullptr && std::strlen(_recordPath) > 0)
    {
      recordPathMod = std::string(_recordPath);

      // Update compressed file path to name of recording directory path
      cmpPath = std::string(recordPathMod);
      if (!std::string(1, cmpPath.back()).compare(ignition::common::separator(
        "")))
      {
        // Remove the separator at end of path
        cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
      }
      cmpPath += ".zip";

      // Check if path or compressed file with same prefix exists
      if (ignition::common::exists(recordPathMod) ||
        ignition::common::exists(cmpPath))
      {
        // Overwrite if flag specified
        if (_logOverwrite > 0)
        {
          bool recordMsg = false;
          bool cmpMsg = false;
          // Remove files before initializing console log files on top of them
          if (ignition::common::exists(recordPathMod))
          {
            recordMsg = true;
            ignition::common::removeAll(recordPathMod);
          }
          if (ignition::common::exists(cmpPath))
          {
            cmpMsg = true;
            ignition::common::removeFile(cmpPath);
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
            ignition::common::separator("")))
          {
            recordPathMod = recordPathMod.substr(0, recordPathMod.length()
              - 1);
          }

          std::string recordOrigPrefix = std::string(recordPathMod);
          int count = 1;

          // Keep renaming until path does not exist for both directory and
          // compressed file
          while (ignition::common::exists(recordPathMod) ||
            ignition::common::exists(cmpPath))
          {
            recordPathMod = recordOrigPrefix +  "(" + std::to_string(count++) +
              ")";

            cmpPath = std::string(recordPathMod);
            // Remove the separator at end of path
            if (!std::string(1, cmpPath.back()).compare(
              ignition::common::separator("")))
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
    }
    // Empty record path specified. Use default.
    else
    {
      // Create log file before printing any messages so they can be logged
      ignLogInit(recordPathMod, "server_console.log");
      ignmsg << "Recording states to default path [" << recordPathMod << "]"
             << std::endl;
    }
    _serverConfig.SetLogRecordPath(recordPathMod);

    std::vector<std::string> topics = ignition::common::split(
        _recordTopics, ":");
    for (const std::string &topic : topics)
    {
      _serverConfig.AddLogRecordTopic(topic);
    }
  }
  else
  {
    ignLogInit(_serverConfig.LogRecordPath(), "server_console.log");
  }

  if (_logCompress > 0)
  {
    _serverConfig.SetLogRecordCompressPath(cmpPath);
  }

  ignmsg << "Ignition Gazebo Server v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  // Set the SDF string to user
  if (_sdfString != nullptr && std::strlen(_sdfString) > 0)
  {
    if (!_serverConfig.SetSdfString(_sdfString))
    {
      ignerr << "Failed to set SDF string [" << _sdfString << "]" << std::endl;
      return -1;
    }
  }
  _serverConfig.SetSdfFile(_file);

  // Set the update rate.
  if (_hz > 0.0)
    _serverConfig.SetUpdateRate(_hz);

  // Set whether levels should be used.
  if (_levels > 0)
  {
    ignmsg << "Using the level system\n";
    _serverConfig.SetUseLevels(true);
  }

  if (_networkRole && std::strlen(_networkRole) > 0)
  {
    ignmsg << "Using the distributed simulation and levels systems\n";
    _serverConfig.SetNetworkRole(_networkRole);
    _serverConfig.SetNetworkSecondaries(_networkSecondaries);
    _serverConfig.SetUseLevels(true);
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
      _serverConfig.SetLogPlaybackPath(ignition::common::absPath(
        std::string(_playback)));
    }
  }

  if (_physicsEngine != nullptr && std::strlen(_physicsEngine) > 0)
  {
    _serverConfig.SetPhysicsEngine(_physicsEngine);
  }

  serverConfig.SetHeadlessRendering(_headless);

  if (_renderEngineServer != nullptr && std::strlen(_renderEngineServer) > 0)
  {
    _serverConfig.SetRenderEngineServer(_renderEngineServer);
  }

  if (_renderEngineGui != nullptr && std::strlen(_renderEngineGui) > 0)
  {
    _serverConfig.SetRenderEngineGui(_renderEngineGui);
  }

  return 0;
}

//////////////////////////////////////////////////
extern "C" int runServer(const char *_sdfString,
    int _iterations, int _run, float _hz, int _levels, const char *_networkRole,
    int _networkSecondaries, int _record, const char *_recordPath,
    int _recordResources, int _logOverwrite, int _logCompress,
    const char *_playback, const char *_physicsEngine,
    const char *_renderEngineServer, const char *_renderEngineGui,
    const char *_file, const char *_recordTopics)
{
  // Create the Gazebo server
  ignition::gazebo::ServerConfig serverConfig;

  if (createServerConfig(serverConfig,
          _sdfString, _hz, _levels, _networkRole,
          _networkSecondaries, _record, _recordPath,
          _recordResources, _logOverwrite, _logCompress,
          _playback, _physicsEngine, _renderEngineServer,
          _renderEngineGui, _file, _recordTopics, false) == 0)
  {
    ignition::gazebo::Server server(serverConfig);
    // Run the server
    server.Run(true, _iterations, _run == 0);
    igndbg << "Shutting down ign-gazebo-server" << std::endl;
    return 0;
  }

  ignerr << "Something was wrong configuring the server. " <<
    "Shutting down ign-gazebo-server" << std::endl;
  return -1;
}

//////////////////////////////////////////////////
extern "C" int runCombined(const char *_sdfString,
    int _iterations, int _run, float _hz, int _levels, const char *_networkRole,
    int _networkSecondaries, int _record, const char *_recordPath,
    int _recordResources, int _logOverwrite, int _logCompress,
    const char *_playback, const char *_physicsEngine,
    const char *_renderEngineServer, const char *_renderEngineGui,
    const char *_file, const char *_recordTopics, const char *_guiConfig)
{
  ignition::gazebo::ServerConfig serverConfig;

  if (!createServerConfig(serverConfig,
        _sdfString, _hz, _levels, _networkRole,
        _networkSecondaries, _record, _recordPath,
        _recordResources, _logOverwrite, _logCompress,
        _playback, _physicsEngine, _renderEngineServer,
        _renderEngineGui, _file, _recordTopics, true) == 0)
  {
    ignerr << "Unable to create server config\n";
    return -1;
  }

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  // Run the server

  // argc and argv are going to be passed to a QApplication. The Qt
  // documentation has a warning about these:
  //  "Warning: The data referred to by argc and argv must stay valid for the
  //  entire lifetime of the QApplication object. In addition, argc must be
  //  greater than zero and argv must contain at least one valid character
  //  string."
  int argc = 1;
  // Converting a string literal to char * is forbidden as of C++11.
  // It can only be converted to a const char *. The const cast is here to
  // prevent a warning since we do need to pass a char* to runGui
  char *argv = const_cast<char *>("ign-gazebo-gui");

  std::vector<std::shared_ptr<ignition::gazebo::System>> runners;
  auto app = ignition::gazebo::gui::createGui(argc, &argv, _guiConfig, runners);

  if (nullptr != app)
  {
    igndbg << "Found GUI runners: " << runners.size() << std::endl;
    for (auto runner: runners)
    {
      auto config = dynamic_cast<ignition::gazebo::ISystemConfigure*>(runner.get());
      if (config != nullptr) {
        igndbg << "Config";
      }

      auto pre = dynamic_cast<ignition::gazebo::ISystemPreUpdate *>(runner.get());
      if (pre != nullptr) {
        igndbg << "Pre";
      }

      auto post = dynamic_cast<ignition::gazebo::ISystemPostUpdate *>(runner.get());
      if (post != nullptr) {
        igndbg << "Post";
      }
      server.AddSystem(runner);
    }

    server.Run(false, _iterations, _run == 0);
    app->exec();
    return 0;
  }
  else
  {
    return -1;
  }
}

//////////////////////////////////////////////////
extern "C" int runGui(const char *_guiConfig)
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
  return ignition::gazebo::gui::runGui(argc, &argv, _guiConfig);
}
