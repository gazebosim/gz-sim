/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_FORCE_TORQUE_HH_
#define IGNITION_GAZEBO_SYSTEMS_FORCE_TORQUE_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class ForceTorquePrivate;

  /// \class ForceTorque ForceTorque.hh ignition/gazebo/systems/ForceTorque.hh
  /// \brief This system manages all Force-Torque sensors in simulation.
  /// Each FT sensor reports body-frame eadings over Ignition Transport.
  class ForceTorque:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit ForceTorque();

    /// \brief Destructor
    public: ~ForceTorque() override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<ForceTorquePrivate> dataPtr;
  };
  }
}
}
}
#endif