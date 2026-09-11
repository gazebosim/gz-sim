/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_BUOYANCYENABLE_HH_
#define GZ_SIM_SYSTEMS_BUOYANCYENABLE_HH_

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
  class BuoyancyEnablePrivate;

  /// \brief A model system that registers its own links with the world's
  /// Buoyancy system, so that a world can restrict buoyancy without knowing
  /// any vehicle's name in advance.
  ///
  /// The world's Buoyancy `<enable>` list has to name entities before they
  /// exist, which a world shared by many vehicles cannot do. This plugin
  /// inverts that: the model, which does know which of its links displace
  /// water, calls `/world/<world_name>/buoyancy/enable` for them once the
  /// simulation is running. Attach it to a `<model>`, alongside a world
  /// running the Buoyancy system with
  /// `<enable_by_default>false</enable_by_default>`.
  ///
  /// This plugin deliberately does not implement Reset. A system without it is
  /// reloaded by a world reset, so its Configure runs again and it re-asserts
  /// the links its SDF declares, overriding a disable issued against it at
  /// runtime. The model's declaration is part of the state a reset returns to.
  ///
  /// The names it sends are scoped from the model as it was *spawned*, so a
  /// model spawned under a different name than its file's still registers the
  /// right entities.
  ///
  /// ## System Parameters
  ///
  /// * `<link>` names one link of this model to enable buoyancy for. Repeat it
  /// for each link that displaces water. With no `<link>` element at all the
  /// whole model is registered, which is what a vehicle whose every collision
  /// is a displacement volume wants; name links when the model also carries
  /// contact or sensor collisions that should not float it.
  ///
  /// ## Example
  ///
  /// ```
  /// <plugin filename="gz-sim-buoyancy-enable-system"
  ///         name="gz::sim::systems::BuoyancyEnable">
  ///   <link>base_link</link>
  /// </plugin>
  /// ```
  class BuoyancyEnable
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: BuoyancyEnable();

    /// \brief Destructor
    public: ~BuoyancyEnable() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const UpdateInfo &_info,
                EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<BuoyancyEnablePrivate> dataPtr;
  };
  }
}
}
}

#endif
