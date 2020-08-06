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
#ifndef IGNITION_GAZEBO_IGN_HH_
#define IGNITION_GAZEBO_IGN_HH_

#include "ignition/gazebo/Export.hh"

/// \brief External hook to read the library version.
/// \return C-string representing the version. Ex.: 0.1.2
extern "C" IGNITION_GAZEBO_VISIBLE char *ignitionGazeboVersion();

/// \brief Get the Gazebo version header.
/// \return C-string containing the Gazebo version information.
extern "C" IGNITION_GAZEBO_VISIBLE char *gazeboVersionHeader();

/// \brief Set verbosity level
/// \param[in] _verbosity 0 to 4
extern "C" IGNITION_GAZEBO_VISIBLE void cmdVerbosity(
    const char *_verbosity);

extern "C" IGNITION_GAZEBO_VISIBLE const char *worldInstallDir();

/// \brief External hook to run simulation server.
/// \param[in] _sdfString SDF file to run, as a string.
/// \param[in] _iterations --iterations option
/// \param[in] _run -r option
/// \param[in] _hz -z option
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
/// \param[in] _file Path to file being loaded
/// \return 0 if successful, 1 if not.
extern "C" IGNITION_GAZEBO_VISIBLE int runServer(const char *_sdfString,
    int _iterations, int _run, float _hz, int _levels,
    const char *_networkRole, int _networkSecondaries, int _record,
    const char *_recordPath, int _recordResources, int _logOverwrite,
    int _logCompress, const char *_playback,
    const char *_physicsEngine, const char *_file);

/// \brief External hook to run simulation GUI.
/// \param[in] _guiConfig Path to Ignition GUI configuration file.
/// \return 0 if successful, 1 if not.
extern "C" IGNITION_GAZEBO_VISIBLE int runGui(const char *_guiConfig);

/// \brief External hook to find or download a fuel world provided a URL.
/// \param[in] _pathToResource Path to the fuel world resource, ie,
/// https://staging-fuel.ignitionrobotics.org/1.0/gmas/worlds/ShapesClone
/// \return C-string containing the path to the local world sdf file
extern "C" IGNITION_GAZEBO_VISIBLE const char *findFuelResource(
    char *_pathToResource);

#endif
