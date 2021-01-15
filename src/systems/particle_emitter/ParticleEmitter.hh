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
#ifndef IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER_HH_
#define IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER_HH_

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
  class ParticleEmitterPrivate;

  /// \brief A system for creating a particle emitter.
  ///
  /// System parameters
  ///
  /// `<emitter_name>`:
  /// `<type>`:
  /// `<pose>`:
  /// `<size>`:
  /// `<rate>`:
  /// `<duration`>:
  /// `<emitting>`:
  /// `<particle_size>`:
  /// `<lifetime>`:
  /// `<material>`:
  /// `<min_velocity>`:
  /// `<max_velocity>`:
  /// `<color_start>`:
  /// `<color_end>`:
  /// `<scale_rate>`:
  /// `<color_range_image>`:
  class IGNITION_GAZEBO_VISIBLE ParticleEmitter
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ParticleEmitter();

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ParticleEmitterPrivate> dataPtr;
  };
  }
}
}
}

#endif

