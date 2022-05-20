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
#ifndef GZ_GAZEBO_DETAIL_COMPONENTSTORAGEBASE_HH_
#define GZ_GAZEBO_DETAIL_COMPONENTSTORAGEBASE_HH_

#include "gz/sim/Export.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_GAZEBO_VERSION_NAMESPACE {
    //
    /// \brief All component instances of the same type are stored
    /// sequentially in memory. This is a base class for storing components
    /// of a particular type.
    class GZ_GAZEBO_HIDDEN ComponentStorageBase
    {
      /// \brief Constructor
      public: GZ_DEPRECATED(6) ComponentStorageBase() = default;

      /// \brief Destructor
      public: virtual ~ComponentStorageBase() = default;
    };

    /// \brief Templated implementation of component storage.
    template<typename ComponentTypeT>
    class GZ_GAZEBO_HIDDEN ComponentStorage : public ComponentStorageBase
    {
      /// \brief Constructor
      public: explicit GZ_DEPRECATED(6) ComponentStorage()
              : ComponentStorageBase()
      {
      }
    };
    }
  }
}

#endif
