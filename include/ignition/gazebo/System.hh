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

#ifndef GAZEBO_SYSTEM_HH_
#define GAZEBO_SYSTEM_HH_

namespace ignition
{
  namespace gazebo
  {
    // Forward declaration
    class SystemPrivate;

    /// \brief Base class for a System
    ///
    /// A System operates on entities that have certain components. A system
    /// will only operate on an Entity if it has all of the required components.
    class System
    {
      /// \brief Constructor
      public: System();

      /// \brief Destructor
      public: virtual ~System();

      /// \brief Disable copy constructor
      /// \parm[in] _system System to not copy
      private: System(const System &_system) = delete;

      /// \brief Private data class
      private: SystemPrivate *dataPtr = nullptr;
    };
  }
}
#endif
