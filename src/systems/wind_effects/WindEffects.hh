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
#ifndef GZ_SIM_SYSTEMS_WINDEFFECTS_HH_
#define GZ_SIM_SYSTEMS_WINDEFFECTS_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class WindEffectsPrivate;

  /// \brief A system that simulates a simple wind model.
  /// The wind is described as a uniform worldwide model.
  /// Its components are computed separately:
  /// - Horizontal amplitude:
  ///      Low pass filtering on user input (complementary gain)
  ///      + small local fluctuations
  ///      + noise on value (noise amplitude is a factor of wind magnitude)
  ///
  /// - Horizontal direction:
  ///      Low pass filtering on user input (complementary gain)
  ///      + small local fluctuations
  ///      + noise on value
  ///
  /// - Vertical amplitude:
  ///      Low pass filtering on user input (complementary gain)
  ///      + small local fluctuations
  ///      + noise on value
  ///
  /// Forces exerted by the wind on model links are approximated from
  /// link mass and velocity with respect to wind velocity, and applied
  /// to the link frame origin. These approximations can be amplified or
  /// attenuated on a per location basis by specifying a piecewise scalar
  /// field for a scaling factor.
  ///
  /// ## System Parameters
  ///
  /// The following parameters are used by the system:
  ///
  /// - `<horizontal><magnitude><time_for_rise>`:
  /// Analogous to the time constant of the low pass filter.
  ///
  /// - `<horizontal><magnitude><sin><amplitude_percent>`:
  /// Fraction of the filtered wind velocity magnitude that is set to be the
  /// amplitude of the sinusoid.
  ///
  /// - `<horizontal><magnitude><sin><period>`:
  /// Period of the sinusoid that is added to the wind velocity magnitude.
  ///
  /// - `<horizontal><magnitude><noise>`:
  /// Parameters for the noise that is added to the wind velocity magnitude.
  ///
  /// - `<horizontal><direction><time_for_rise>`:
  /// Analogous to the time constant of the low pass filter.
  ///
  /// - `<horizontal><direction><sin><amplitude>`:
  /// Amplitude of the sinusoidal that is added on the direction of the wind
  /// velocity.
  ///
  /// - `<horizontal><direction><sin><period>`:
  /// Period of the sinusoid that is added to the wind velocity direction.
  ///
  /// - `<horizontal><direction><noise>`:
  /// Parameters for the noise that is added to the wind velocity direction.
  ///
  /// - `<vertical><time_for_rise>`:
  /// Analogous to the time constant of the low pass filter for the vertical
  /// wind velocity magnitude.
  ///
  /// - `<vertical><noise>`:
  /// Parameters for the noise that is added to the vertical wind velocity
  /// magnitude.
  ///
  /// - `<force_approximation_scaling_factor>`:
  /// Proportionality constant used for wind force approximations as a
  /// piecewise, separable scalar field:
  /// ```
  ///   <force_approximation_scaling_factor>
  ///     <when xlt="0"> <!-- Half space where x < 0 -->
  ///       <k>1</k>
  ///       <px>0 0 0 1</px>  <!-- p(x) = x -->
  ///       <qy>0 0 0 1</qy>  <!-- q(y) = 1 -->
  ///       <rz>0 1 0 1</rz>  <!-- r(z) = z^2 -->
  ///     </when>
  ///   </force_approximation_scaling_factor>
  /// ```
  /// When the scaling factor is to be constant in a region, a numerical
  /// constant may be used in place for the scalar field definition:
  /// ```
  ///   <force_approximation_scaling_factor>
  ///     <!-- First octant -->
  ///     <when xge="0" yge="0" zge="0">1</when>
  ///   </force_approximation_scaling_factor>
  /// ```
  /// To use the same constant or scalar field in all space, region
  /// definition may be dropped:
  /// ```
  ///   <force_approximation_scaling_factor>
  ///     <k>2</k>
  ///     <px>1 0 1 0</px>  <!-- p(x) = x^3 + x -->
  ///     <qy>0 1 0 1</qy>  <!-- q(y) = x^2 + 1 -->
  ///     <rz>1 0 1 0</rz>  <!-- r(z) = z^3 + z -->
  ///   </force_approximation_scaling_factor>
  /// ```
  /// Regions may not overlap.
  ///
  class WindEffects final:
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate
  {
    /// \brief Constructor
    public: WindEffects();

    /// \brief Destructor
    public: ~WindEffects() final;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<WindEffectsPrivate> dataPtr;
  };
  }
}
}
}
#endif
