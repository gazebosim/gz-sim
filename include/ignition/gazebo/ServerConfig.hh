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
#ifndef IGNITION_GAZEBO_SERVERCONFIG_HH_
#define IGNITION_GAZEBO_SERVERCONFIG_HH_

#include <chrono>
#include <memory>
#include <optional> // NOLINT(*)
#include <string>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class ServerConfigPrivate;

    /// \class ServerConfig ServerConfig.hh ignition/gazebo/ServerConfig.hh
    /// \brief Configuration parameters for a Server. An instance of this
    /// object can be used to construct a Server with a particular
    /// configuration.
    class IGNITION_GAZEBO_VISIBLE ServerConfig
    {
      /// \brief Constructor
      public: ServerConfig();

      /// \brief Destructor
      public: ~ServerConfig();

      /// \brief Set an SDF file. The SDF parser will attempt to load the
      /// provided file. If the file fails to load, then the stored SDF file
      /// will remain unchanged and a false value will be returned. You can
      /// override the check using the _force parameter.
      /// \param[in] _file Full path to an SDF file.
      /// \param[in] _force Force the stored SDF file, bypassing the SDF
      /// parser check.
      /// \return True if the provided file was successfully found and
      /// parsed, false otherwise.
      public: bool SetSdfFile(const std::string &_file);

      /// \brief Get the SDF file that has been set. An empty string will be
      /// returned if an SDF file has not been set.
      /// \return The full path to the SDF file, or empty string.
      public: std::string SdfFile() const;

      /// \brief Set the update rate in Hertz. Value <=0 are ignored.
      /// \param[in] _hz The desired update rate of the server in Hertz.
      public: void SetUpdateRate(const double &_hz);

      /// \brief Get the update rate in Hertz.
      /// \return The desired update rate of the server in Hertz, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<double> UpdateRate() const;

      /// \brief Get the update period duration.
      /// \return The desired update period, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<std::chrono::steady_clock::duration>
              UpdatePeriod() const;

      /// \brief Path to where simulation resources, such as models downloaded
      /// from fuel.ignitionrobotics.org, should be stored.
      /// \return Path to a location on disk. An empty string indicates that
      /// the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: const std::string &ResourceCache() const;

      /// \brief Set the path to where simulation resources, such as models
      /// downloaded from fuel.ignitionrobotics.org, should be stored.
      /// \param[in] _path Path to a location on disk. An empty string
      /// indicates that the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: void SetResourceCache(const std::string &_path);

      /// \brief Private data pointer
      private: std::unique_ptr<ServerConfigPrivate> dataPtr;
    };
    }
  }
}

#endif
