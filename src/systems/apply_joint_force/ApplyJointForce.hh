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
#ifndef GZ_SIM_SYSTEMS_APPLYJOINTFORCE_HH_
#define GZ_SIM_SYSTEMS_APPLYJOINTFORCE_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ApplyJointForcePrivate;

  /// \brief This system applies a force to the first axis of a specified joint.
  ///
  /// ## Components
  ///
  /// This system uses the following components:
  ///
  /// - gz::sim::components::JointForceCmd: A std::vector of force commands
  ///   of type `double`. Only element `[0]` is used, for applying force to
  ///   the specified joint.
  class ApplyJointForce
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ApplyJointForce();

    /// \brief Destructor
    public: ~ApplyJointForce() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ApplyJointForcePrivate> dataPtr;
  };
  }
}
}
}

#endif
