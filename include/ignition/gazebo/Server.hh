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
#ifndef IGNITION_GAZEBO_SERVER_HH_
#define IGNITION_GAZEBO_SERVER_HH_

#include <sdf/Model.hh>
#include "ignition/gazebo/Entity.hh"

namespace ignition
{
  namespace gazebo
  {
    // Forware declarations
    class ServerPrivate;

    /// \brief The server instantiates and controls simulation.
    ///
    /// ## Services
    ///
    /// The following are services provided by the Server.
    /// List syntax: *topic_name(request_message) : response_message*
    ///
    /// 1. /ign/gazebo/scene(none) : ignition::msgs::Scene
    ///   + Returns the current scene information.
    ///
    class Server
    {
      /// \brief Constructor
      public: Server();

      /// \brief Destructor
      public: ~Server();

      /// \brief Delete the copy constructor.
      /// \param[in] _server Server that is not copied.
      public: Server(const Server &_server) = delete;

      /// \brief Delete the move constructor
      /// \param[in] _server Server that is not moved.
      public: Server(const Server &&_server) = delete;

      /// \brief Create an entity based on an SDF Model.
      /// \param[in] _model The SDF model to create an Entity from.
      /// \return The Entity identifier.
      public: Entity CreateEntity(const sdf::Model &_model);

      /// \brief Run the server. By default, this is a blocking call. Pass
      /// in false to run the server in a separate thread.
      /// \param[in] _blocking False to run the server in a new thread, and
      /// \param[in] _iterations Number of steps to perform. A value of
      /// zero will run indefinitely.
      /// return immediately.
      public: void Run(const uint64_t _iterations = 0,
                       const bool _blocking = false);

      /// \brief Private data
      private: ServerPrivate *dataPtr;
    };
  }
}

#endif
