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
#ifndef IGNITION_GAZEBO_TESTSYSTEM_HH_
#define IGNITION_GAZEBO_TESTSYSTEM_HH_

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/System.hh"

namespace ignition
{
  namespace gazebo
  {
    /// \brief A stand-in system for testing and development.
    /// This will go away in the near future.
    class TestSystem : public System
    {
      /// \brief Constructor
      public: TestSystem();

      /// \brief Destructor
      public: virtual ~TestSystem();

      /// \brief Update the system.
      /// \return True on success.
      public: virtual bool Update() final;
    };
  }
}
#endif
