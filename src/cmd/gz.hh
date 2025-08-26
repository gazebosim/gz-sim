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

#include <string>
#include <vector>

#include "gz/sim/ServerConfig.hh"

/// \brief Set verbosity level
/// \param[in] _verbosity 0 to 4
void cmdVerbosity(const int _verbosity);

/// \brief Get the install directory of world SDFormat files
/// \return String containing the relative path
const std::string worldInstallDir();

/// \brief Check if the file exists
/// \param[in] _file Name of the SDFormat file
/// \return 0 if exists, -1 in case of error
int checkFile(std::string &_file);

/// \brief Parse the SDFormat file into a string
/// \param[in] _file Name of the SDFormat file
/// \param[out] _parsedSdfFile Assign the parsed string
/// \return 0 if successful, -1 in case of error
int parseSdfFile(const std::string &_file, std::string &_parsedSdfFile);

/// \brief Find or download a fuel world provided a URL.
/// \param[in] _pathToResource Path to the fuel world resource, ie,
/// https://staging-fuel.gazebosim.org/1.0/gmas/worlds/ShapesClone
/// \return String containing the path to the local world sdf file
std::string findFuelResource(const std::string &_pathToResource);

/// \brief Create Gazebo server configuration
/// \param[in] _config Gazebo server config
/// \param[in] _sdfString SDF file to run, as a string.
/// \param[in] _hz -z option
/// \param[in] _initialSimTime --initial_sim_time option
/// \param[in] _levels --levels option
/// \param[in] _networkRole --network-role option
/// \param[in] _networkSecondaries --network-secondaries option
/// \param[in] _record --record option
/// \param[in] _recordPath --record-path option
/// \param[in] _recordResources --record-resources option
/// \param[in] _logOverwrite --log-overwrite option
/// \param[in] _logCompress --log-compress option
/// \param[in] _playback --playback option
/// \param[in] _physicsEngine --physics-engine option
/// \param[in] _renderEngineServer --render-engine-server option
/// \param[in] _renderEngineServerApiBackend --render-engine-server-api-backend
/// \param[in] _renderEngineGui --render-engine-gui option
/// \param[in] _renderEngineGuiApiBackend --render-engine-gui-api-backend
/// \param[in] _file Path to file being loaded
/// \param[in] _recordTopics Colon separated list of topics to record. Leave
/// \param[in] _waitGui Flag indicating whether the server waits until
/// it receives a world path from GUI.
/// null to record the default topics.
/// \param[in] _headless True if server rendering should run headless
/// \param[in] _recordPeriod --record-period option
/// \param[in] _seed --seed value to be used for random number generator.
/// \param[in] _waitForAssets True to wait for assets to download before
/// starting simulation.
/// \return 0 if successful, -1 in case of failure
int createServerConfig(gz::sim::ServerConfig &_config, const char *_sdfString,
                        float _hz, double _initialSimTime, int _levels,
                        const char *_networkRole, int _networkSecondaries,
                        int _record, const char *_recordPath,
                        int _recordResources, int _logOverwrite,
                        int _logCompress, const char *_playback,
                        const char *_physicsEngine,
                        const char *_renderEngineServer,
                        const char *_renderEngineServerApiBackend,
                        const char *_renderEngineGui,
                        const char *_renderEngineGuiApiBackend,
                        const char *_file,
                        std::vector<std::string> _recordTopics, int _waitGui,
                        int _headless, float _recordPeriod, int _seed,
                        int _waitForAssets);

#ifdef WITH_GUI
/// \brief Run simulation GUI.
/// \param[in] _guiConfig Path to Gazebo GUI configuration file.
/// \param[in] _file The world file path passed as a command line argument.
/// If set, QuickStart Dialog will not be shown.
/// \param[in] _waitGui Flag indicating whether the server waits until
/// it receives a world path from GUI.
/// \param[in] _renderEngine --render-engine-gui option
/// \param[in] _renderEngineGuiApiBackend --render-engine-gui-api-backend option
/// \return 0 if successful, -1 in case of failure
int runGui(const char *_guiConfig, const char *_file,
           int _waitGui, const char *_renderEngine,
           const char *_renderEngineGuiApiBackend);
#endif
