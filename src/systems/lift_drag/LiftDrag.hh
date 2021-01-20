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

#ifndef IGNITION_GAZEBO_SYSTEMS_LIFT_DRAG_HH_
#define IGNITION_GAZEBO_SYSTEMS_LIFT_DRAG_HH_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class LiftDragPrivate;

  /// \brief The LiftDrag system computes lift and drag forces enabling
  /// simulation of aerodynamic robots.
  ///
  /// The following parameters are used by the system:
  ///
  /// link_name   : Name of the link affected by the group of lift/drag
  ///               properties.
  /// air_density : Density of the fluid this model is suspended in.
  /// area        : Surface area of the link.
  /// a0          : The initial "alpha" or initial angle of attack. a0 is also
  ///               the y-intercept of the alpha-lift coefficient curve.
  /// cla         : The ratio of the coefficient of lift and alpha slope before
  ///               stall. Slope of the first portion of the alpha-lift
  ///               coefficient curve.
  /// cda         : The ratio of the coefficient of drag and alpha slope before
  ///               stall.
  /// cp          : Center of pressure. The forces due to lift and drag will be
  ///               applied here.
  /// forward     : 3-vector representing the forward direction of motion in the
  ///               link frame.
  /// upward      : 3-vector representing the direction of lift or drag.
  /// alpha_stall : Angle of attack at stall point; the peak angle of attack.
  /// cla_stall   : The ratio of coefficient of lift and alpha slope after
  ///               stall.  Slope of the second portion of the alpha-lift
  ///               coefficient curve.
  /// cda_stall   : The ratio of coefficient of drag and alpha slope after
  ///               stall.
  class LiftDrag
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: LiftDrag();

    /// \brief Destructor
    public: ~LiftDrag() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<LiftDragPrivate> dataPtr;
  };
  }
}
}
}

#endif

