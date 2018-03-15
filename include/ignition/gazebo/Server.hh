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

    class Server
    {
      public: Server();

      public: Entity CreateEntity(const sdf::Model &_model);

      public: int Step(const unsigned int _iterations);

      private: ServerPrivate *dataPtr;
    };
  }
}

#endif
