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
#ifndef IGNITION_GAZEBO_PHYSICS_SYSTEM_HH_
#define IGNITION_GAZEBO_PHYSICS_SYSTEM_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityQueryRegistrar.hh>
#include <ignition/gazebo/EntityQueryResult.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
  namespace gazebo
  {
    // Forward declarations.
    class PhysicsSystemPrivate;

    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class PhysicsSystem PhysicsSystem.hh ignition/gazebo/PhysicsSystem.hh
    /// \brief Base class for a System.
    class IGNITION_GAZEBO_VISIBLE PhysicsSystem : public System
    {
      /// \brief Constructor
      public: explicit PhysicsSystem(const SystemConfig &_config);

      /// \brief Destructor
      public: virtual ~PhysicsSystem();

      public: void Init(EntityQueryRegistrar &_registrar) override final;

      private: void OnUpdate(const EntityQueryResult &_result);

      /// \brief Private data pointer.
      private: std::unique_ptr<PhysicsSystemPrivate> dataPtr;
    };
    }
  }
}
#endif
