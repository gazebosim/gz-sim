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
#ifndef IGNITION_GAZEBO_COMPONENT_TYPE_HH_
#define IGNITION_GAZEBO_COMPONENT_TYPE_HH_

#include <string>

#include <ignition/gazebo/config.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    class IGNITION_GAZEBO_VISIBLE ComponentType
    {
      public: ComponentType();

      public: virtual ~ComponentType();

      public: template<typename Type>
              bool Init(const std::string &_name,
                        const ComponentTypeId _typeId)
      {
        this->typeId = _typeId;
        this->name = _name;
        return this->Valid();
      }

      public: const std::string &Name() const;
      public: const ComponentTypeId &TypeId() const;

      public: bool Valid() const;

      private: ComponentTypeId typeId;
      private: std::string name;
    };
    }
  }
}
#endif
