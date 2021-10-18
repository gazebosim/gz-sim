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

%module server
%{
#include <ignition/gazebo/Server.hh>
%}

%include "stdint.i"
%include <std_shared_ptr.i>

%shared_ptr(ignition::gazebo::Server)

namespace ignition
{
  namespace gazebo
  {
    class Server
    {
    %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";

    public: explicit Server(const ServerConfig &_config = ServerConfig());

    /// \brief Destructor
    public: ~Server();

    public: bool Running() const;

    public: bool Run(const bool _blocking = false,
                     const uint64_t _iterations = 0,
                     const bool _paused = true);
    };
  }
}
