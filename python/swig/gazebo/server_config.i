/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module serverconfig
%{
#include <ignition/gazebo/ServerConfig.hh>
%}

%include <std_string.i>

namespace ignition
{
  namespace gazebo
  {
    class ServerConfig
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";

      /// \brief Constructor
      public: ServerConfig();

      /// \brief Copy constructor.
      /// \param[in] _config ServerConfig to copy.
      public: ServerConfig(const ServerConfig &_config);

      /// \brief Destructor
      public: ~ServerConfig();

      /// \brief Set an SDF file to be used with the server.
      ///
      /// Setting the SDF file will override any value set by `SetSdfString`.
      ///
      /// \param[in] _file Full path to an SDF file.
      /// \return (reserved for future use)
      public: bool SetSdfFile(const std::string &_file);

      /// \brief Get the SDF file that has been set. An empty string will be
      /// returned if an SDF file has not been set.
      /// \return The full path to the SDF file, or empty string.
      public: std::string SdfFile() const;
    };
  }
}
