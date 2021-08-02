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
#ifndef IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER2_HH_
#define IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER2_HH_

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
  class ParticleEmitter2Private;

  /// \brief A system for running and managing particle emitters. A particle
  /// emitter is defined using the <particle_emitter> SDF element.
  ///
  /// This system will create a transport subscriber for each
  /// <particle_emitter> using the child <topic> name. If a <topic> is not
  /// specified, the following topic naming scheme will be used:
  /// `/model/{model_name}/link/{link_name}/particle_emitter/{emitter_name}/cmd`
  ///
  /// \todo(nkoenig) Plan for ParticleEmitter and ParticleEmitter2:
  ///     1. Deprecate ParticleEmitter in Ignition Fortress.
  ///     2. Remove ParticleEmitter in Ignition G.
  ///     3. Copy ParticleEmitter2 to ParticleEmitter in Ignition G.
  ///     4. Deprecrate ParticleEmitter2 in Ignition G.
  ///     5. Remove ParticleEmitter2 in Ignition H.
  class ParticleEmitter2
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ParticleEmitter2();

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
    private: std::unique_ptr<ParticleEmitter2Private> dataPtr;
  };
  }
}
}
}

#endif

