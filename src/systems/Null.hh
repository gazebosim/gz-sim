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
#ifndef IGNITION_GAZEBO_NULL_SYSTEM_HH_
#define IGNITION_GAZEBO_NULL_SYSTEM_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \class Null Null.hh ignition/gazebo/systems/Null.hh
  /// \brief Minimal system implementation
  class IGNITION_GAZEBO_VISIBLE Null:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: Null();

    /// \brief Destructor
    public: ~Null() override;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;
  };
  }
}
}
}
#endif


