/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_LighterThanAirDynamics_HH_
#define GZ_SIM_SYSTEMS_LighterThanAirDynamics_HH_

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
  class LighterThanAirDynamicsPrivateData;

  /// \brief This class provides the effect of viscousity on the hull of
  /// lighter-than-air vehicles such as airships. The equations implemented
  /// is based on the work published in [1], which describes a modeling
  /// approach for the nonlinear dynamics simulation of airships and [2]
  /// providing more insight of the modelling of an airship.
  ///
  /// ## System Parameters
  /// * `<air_density>` sets the density of air that surrounds
  /// the buoyant object. [Units: kgm^-3]

  /// * `<force_invisc_coeff` is the first coefficient in Eq (11) in [1]:
  /// \f$ (k_2 - k_1) \cdot \int_{\varepsilon_{0}}^{L} \frac{dS}{d\varepsilon}
  /// \,d\varepsilon \f$

  /// * `<force_visc_coeff>` is the second coefficient in Eq (11) in [1]:
  ///  \f$ \eta C_{DC} \cdot \int_{\varepsilon_{0}}^{L} 2R \,d\varepsilon \f$

  /// * `<moment_invisc_coeff` is the first coefficient in Eq (14) in [1]:
  /// \f$ (k_2 - k_1) \cdot \int_{\varepsilon_{0}}^{L} \frac{dS}{d\varepsilon}
  /// (\varepsilon_{m} - \varepsilon) \,d\varepsilon \f$

  /// * `<moment_visc_coeff>` is the second coefficient in Eq (14) in [1]:
  ///  \f$ \eta C_{DC} \cdot \int_{\varepsilon_{0}}^{L} 2R (\varepsilon_{m} -
  /// \varepsilon) \,d\varepsilon \f$

  /// * `<eps_v>` is the point on the hull where the flow ceases being
  /// potential

  /// * `<axial_drag_coeff>` the actual drag coefficient of the hull

  /// ## Notes
  ///
  /// This class only implements the viscous effects on the hull of an
  /// airship and currently does not take into the account wind.
  /// This class should be used in combination with the boyuancy, added mass
  /// and gravity plugins to simulate the behaviour of an airship.
  /// Its important to provide a collision property to the hull, since this is
  /// from which the buoyancy plugin determines the volume.
  ///
  /// # Citations
  /// [1] Li, Y., & Nahon, M. (2007). Modeling and simulation of airship
  /// dynamics.  Journal of Guidance, Control, and Dynamics, 30(6), 1691–1700.
  ///
  /// [2] Li, Y., Nahon, M., & Sharf, I. (2011). Airship dynamics modeling:
  /// A literature review. Progress in Aerospace Sciences, 47(3), 217–239.

  class LighterThanAirDynamics:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: LighterThanAirDynamics();

    /// \brief Destructor
    public: ~LighterThanAirDynamics() override;

    /// Documentation inherited
    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &/*_eventMgr*/) override;

    /// Documentation inherited
    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    /////////////////////////////////////////////////
    /// \brief Calculates the local velocity at an offset from a origin
    /// \param[in] lin_vel  - The linear body velocity
    /// \param[in] ang_vel  - The angular body velocity
    /// \param[in] dist     - The distance vector from the origin
    /// \return The local velocity at the distance vector
    public: math::Vector3d LocalVelocity(math::Vector3d lin_vel,
                    math::Vector3d ang_vel, math::Vector3d dist);

    /////////////////////////////////////////////////
    /// \brief Calculates dynamic pressure
    /// \param[in] vec  - The linear velocity
    /// \param[in] air_density  - The air density [kg/m^3]
    /// \return The dynamic pressure, q
    public: double DynamicPressure(math::Vector3d vec, double air_density);

    /////////////////////////////////////////////////
    /// \brief Calculates the potential flow aerodynamic forces that a LTA
    /// vehicle experience when moving in a potential fluid. The aerodynamic
    /// force is derived using Kirchoff's equation.
    /// \param[in] lin_vel  - The body linear velocity
    /// \param[in] ang_vel  - The body angular velocity
    /// \param[in] m11  - The left upper [3x3] matrix of the added mass matrix
    /// \param[in] m12  - The right upper [3x3] matrix of the added mass matrix
    /// \param[in] m21  - The left lower [3x3] matrix of the added mass matrix
    /// \param[in] m22  - The right lower [3x3] matrix of the added mass matrix
    /// \return The aerodynamic force.
    public: math::Vector3d AddedMassTorque(math::Vector3d lin_vel,
                            math::Vector3d ang_vel,
                            math::Matrix3d m11, math::Matrix3d m12,
                            math::Matrix3d m21, math::Matrix3d m22);

    /////////////////////////////////////////////////
    /// \brief Calculates the potential flow aerodynamic torques that a LTA
    /// vehicle experience when moving in a potential fluid. The aerodynamic
    /// torques is derived using Kirchoff's equation.
    /// \param[in] lin_vel  - The body linear velocity
    /// \param[in] ang_vel  - The body angular velocity
    /// \param[in] m11  - The left upper [3x3] matrix of the added mass matrix
    /// \param[in] m12  - The right upper [3x3] matrix of the added mass matrix
    /// \return The aerodynamic torque
    public: math::Vector3d AddedMassForce(math::Vector3d lin_vel,
                            math::Vector3d ang_vel,
                            math::Matrix3d m11, math::Matrix3d m12);

    /////////////////////////////////////////////////
    /// \brief Skew-symmetric matrices can be used to represent cross products
    /// as matrix multiplications.
    /// \param[in] mat - A [3x1] vector
    /// \return The skew-symmetric matrix of mat
    public: math::Matrix3d SkewSymmetricMatrix(math::Vector3d mat);

    /// \brief Private data pointer
    private: std::unique_ptr<LighterThanAirDynamicsPrivateData> dataPtr;
  };
}
}
}
}
#endif
