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

#ifndef GZ_SIM_SYSTEMS_ADVANCED_LIFT_DRAG_HH_
#define GZ_SIM_SYSTEMS_ADVANCED_LIFT_DRAG_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class AdvancedLiftDragPrivate;

  /// \brief The LiftDrag system computes lift and drag forces enabling
  /// simulation of aerodynamic robots.
  ///
  ///
  /// A tool can be found at the link below that automatically generates
  /// the Advanced Lift Drag plugin for you. All that is needed is to
  /// provide some physical parameters of the model. The README contains
  /// all necessary setup and usage steps.
  ///
  /// https://github.com/PX4/PX4-Autopilot/blob/main/Tools/simulation/
  /// gz/tools/avl_automation/
  ///
  /// Note: Wind calculations can be enabled by setting the wind parameter
  ///       in the world file.
  ///
  /// ## System Parameters
  ///
  /// - `<link_name>`: Name of the link affected by the group of lift/drag
  /// properties. This can be a scoped name to reference links in
  /// nested models. \sa entitiesFromScopedName to learn more
  /// about scoped names.
  /// - `<rho>`: Density of the fluid this model is suspended in.
  /// - `<area>`: Surface area of the link.
  /// - `<CL0>`: Lift Coefficient at zero angle of attack
  /// - `<Cem0>`: Pitching moment coefficient at zero angle of attack
  /// - `<CD0>`: Drag Coefficient at zero angle of attack
  /// - `<CLa>`: dCL/da (slope of CL-alpha curve) Slope of the first
  /// portion of the alpha-lift coefficient curve.
  /// - `<CYa>`: dCy/da (sideforce slope wrt alpha)
  /// - `<Cella>`: dCl/da (roll moment slope wrt alpha)
  /// - `<Cema>`: dCm/da (pitching moment slope wrt alpha - before stall)
  /// - `<Cena>`: dCn/da (yaw moment slope wrt alpha)
  /// - `<CLb>`: dCL/dbeta (lift coefficient slope wrt beta)
  /// - `<CYb>`: dCY/dbeta (side force slope wrt beta)
  /// - `<Cellb>`: dCl/dbeta (roll moment slope wrt beta)
  /// - `<Cemb>`: dCm/dbeta (pitching moment slope wrt beta)
  /// - `<Cenb>`: dCn/dbeta (yaw moment slope wrt beta)
  /// - `<alphaStall>`: angle of attack when airfoil stalls
  /// - `<CemaStall>`: Slope of the Cm-alpha curve after stall
  /// - `<cp>`: 3-vector replacing the center of pressure in the original
  /// LiftDragPlugin.
  /// - `<CDp>`: dCD/dp (drag coefficient slope wrt roll rate)
  /// - `<CYp>`: dCY/dp (sideforce slope wrt roll rate)
  /// - `<CLp>`: dCL/dp (lift coefficient slope wrt roll rate)
  /// - `<Cellp>`: dCl/dp (roll moment slope wrt roll rate)
  /// - `<Cenp>`: dCn/dp (yaw moment slope wrt roll rate)
  /// - `<CDq>`: dCD/dq (drag coefficient slope wrt pitching rate)
  /// - `<CYq>`: dCY/dq (side force slope wrt pitching rate)
  /// - `<CLq>`: dCL/dq (lift coefficient slope wrt pitching rate)
  /// - `<Cellq>`: dCl/dq (roll moment slope wrt pitching rate)
  /// - `<Cemq>`: dCm/dq (pitching moment slope wrt pitching rate)
  /// - `<Cenq>`: dCn/dq (yaw moment slope wrt pitching rate)
  /// - `<CDr>`: dCD/dr (drag coefficient slope wrt yaw rate)
  /// - `<CYr>`: dCY/dr (side force slope wrt yaw rate)
  /// - `<CLr>`: dCL/dr (lift coefficient slope wrt yaw rate)
  /// - `<Cellr>`: dCl/dr (roll moment slope wrt yaw rate)
  /// - `<Cemr>`: dCm/dr (pitching moment slope wrt yaw rate)
  /// - `<Cenr>`: dCn/dr (yaw moment slope wrt yaw rate)
  /// - `<num_ctrl_surfaces>`: Number of control surfaces
  /// - `<controlJoints>`: Vector that points to the joints that connect to the
  /// control surface
  /// - `<ctrl_surface_direction>`: Vectors of control surface deflections
  /// - `<CD_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of drag
  /// - `<CY_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of side force
  /// - `<CL_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of lift
  /// - `<Cell_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of roll moment
  /// - `<Cem_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of pitching moment
  /// - `<Cen_ctrl>`: Vector of the effect of the deflection on the coefficient
  /// of yaw moment
  /// - `<AR>`: Aspect ratio
  /// - `<mac>`: The mean-aerodynamic chord
  /// - `<eff>`: Wing efficiency (This is the Oswald efficiency factor for a
  /// 3D wing)
  /// - `<cda>`: The ratio of the coefficient of drag and alpha slope before
  /// stall.
  /// - `<forward>`: 3-vector representing the forward direction of motion in
  /// the link frame.
  /// - `<upward>`: 3-vector representing the direction of lift or drag.
  /// - `<alphaStall>`: Angle of attack at stall point; peak angle of attack.
  /// - `<alpha>`: The angle of attack
  /// - `<beta>`: The sideslip angle
  /// - `<M>`: The sigmoid blending parameter
  /// - `<CD_fp_k1>`: The first of the flat plate drag model coefficients
  /// - `<CD_fp_k2>`: The second of the flat plate drag model coefficients
  /// - `<sdfConfig>`: Copy of the sdf configuration used for this plugin

  class AdvancedLiftDrag
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: AdvancedLiftDrag();

    /// \brief Destructor
    public: ~AdvancedLiftDrag() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<AdvancedLiftDragPrivate> dataPtr;
  };
  }
}
}
}
#endif
