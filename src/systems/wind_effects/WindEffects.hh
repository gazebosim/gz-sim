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
#ifndef IGNITION_GAZEBO_SYSTEMS_WINDEFFECTS_HH_
#define IGNITION_GAZEBO_SYSTEMS_WINDEFFECTS_HH_

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
  class WindEffectsPrivate;

  /// \brief A system that simulates a simple wind model.
  /// The wind is described as a uniform worldwide model. So it is independent
  /// from model position for simple computations. Its components are computed
  /// separately:
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
  /// The following parameters are used by the system:
  ///
  /// <horizontal><magnitude><time_for_rise>
  /// Analogous to the time constant of the low pass filter.
  ///
  /// <horizontal><magnitude><sin><amplitude_percent>
  /// Fraction of the filtered wind velocity magnitude that is set to be the
  /// amplitude of the sinusoid.
  ///
  /// <horizontal><magnitude><sin><period>
  /// Period of the sinusoid that is added to the wind velocity magnitude.
  ///
  /// <horizontal><magnitude><noise>
  /// Parameters for the noise that is added to the wind velocity magnitude.
  ///
  /// <horizontal><direction><time_for_rise>
  /// Analogous to the time constant of the low pass filter.
  ///
  /// <horizontal><direction><sin><amplitude>
  /// Amplitude of the sinusoidal that is added on the direction of the wind
  /// velocity.
  ///
  /// <horizontal><direction><sin><period>
  /// Period of the sinusoid that is added to the wind velocity direction.
  ///
  /// <horizontal><direction><noise>
  /// Parameters for the noise that is added to the wind velocity direction.
  ///
  /// <vertical><time_for_rise>
  /// Analogous to the time constant of the low pass filter for the vertical
  /// wind velocity magnitude.
  ///
  /// <vertical><noise>
  /// Parameters for the noise that is added to the vertical wind velocity
  /// magnitude.
  class WindEffects:
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
