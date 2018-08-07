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
#ifndef IGNITION_GAZEBO_WORLD_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_WORLD_COMPONENT_TYPE_HH_

#include <memory>
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
    class WorldComponentPrivate;

    /// \brief A component type that contains world statistic information.
    class IGNITION_GAZEBO_VISIBLE WorldComponent
    {
      /// \brief Constructor
      /// \param[in] _compMgr The entity component manager, which is used to
      /// register the component type.
      public: explicit WorldComponent(const std::string &_name);

      /// \brief Copy Constructor
      /// \param[in] _world WorldComponent to copy.
      public: WorldComponent(const WorldComponent &_world);

      /// \brief Move Constructor
      /// \param[in] _world WorldComponent to move.
      public: WorldComponent(WorldComponent &&_world) noexcept;

      /// \brief Destructor.
      public: virtual ~WorldComponent();

      // Documentation inherited
      public: const std::string &ComponentName() const;

      /// \brief Move assignment operator.
      /// \param[in] _world WorldStatistics component to move.
      /// \return Reference to this.
      public: WorldComponent &operator=(WorldComponent &&_world);

      /// \brief Copy assignment operator.
      /// \param[in] _world WorldStatistics component to copy.
      /// \return Reference to this.
      public: WorldComponent &operator=(const WorldComponent &_world);

      // Documentation inherited
      public: const std::string &Name() const;

      /// \brief Private data pointer.
      private: std::unique_ptr<WorldComponentPrivate> dataPtr;
    };
    }
  }
}
#endif
