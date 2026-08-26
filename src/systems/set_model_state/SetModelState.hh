/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_SETMODELSTATE_HH_
#define GZ_SIM_SYSTEMS_SETMODELSTATE_HH_

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
  class SetModelStatePrivate;

  /// \brief This system should be attached to a model and sets a specified
  /// model state during Configure and Reset using the syntax shown below.
  /// Currently joint positions and velocities can be set for named joints.
  /// By default, the units for position are radians for rotational joints
  /// and meters for translational joints. Likewise the units for velocity
  /// are radians / second for rotational joints and meters / second
  /// for translational joints. If the `degrees` attribute is set to "true"
  /// for a given scalar value, the value will be scaled by (π / 180)
  /// before being interpreted using default units.
  /// TODO(scpeters): warn if degrees==true for a translational joint.
  /** \code{.xml}
    <model_state>
      <joint_state name="joint_0">
        <axis_state>
          <position degrees="true">60</position>
          <velocity degrees="true">-30</velocity>
        </axis_state>
      </joint_state>
      <joint_state name="joint_1">
        <axis_state>
          <position>1.0471975512</position>
          <velocity>-0.5235987756</velocity>
        </axis_state>
      </joint_state>
      <joint_state name="multi_axis_joint">
        <axis_state>
          <position degrees="true">60</position>
          <velocity degrees="true">-30</velocity>
        </axis_state>
        <axis2_state>
          <position degrees="true">-30</position>
          <velocity degrees="true">15</velocity>
        </axis2_state>
      </joint_state>
    </model_state>
  \endcode */
  /// ## Components
  ///
  /// This system uses the following components:
  ///
  /// - gz::sim::components::JointPositionReset
  /// - gz::sim::components::JointVelocityReset
  /// TODO(scpeters): add components to reset link pose and velocity
  class SetModelState
      : public System,
        public ISystemConfigure,
        public ISystemReset
  {
    /// \brief Constructor
    public: SetModelState();

    /// \brief Destructor
    public: ~SetModelState() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void Reset(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<SetModelStatePrivate> dataPtr;
  };
  }
}
}
}

#endif
