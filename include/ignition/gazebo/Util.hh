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
#ifndef IGNITION_GAZEBO_UTIL_HH_
#define IGNITION_GAZEBO_UTIL_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class Util Util.hh ignition/gazebo/Util.hh
    /// \brief Utility class that provides a set of convenient helper functions
    class IGNITION_GAZEBO_VISIBLE Util
    {
      /// \brief Helper function to compute world pose of an entity
      /// \param[in] _entity Entity to get the world pose for
      /// \param[in] _ecm Immutable reference to ECM.
      /// \return World pose of entity
      public: static math::Pose3d WorldPose(const Entity &_entity,
          const EntityComponentManager &_ecm);
    };
    }
  }
}
#endif
