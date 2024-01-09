/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef GZ_SIM_SYSTEMS_KINETIC_ENERGY_MONITOR_HH_
#define GZ_SIM_SYSTEMS_KINETIC_ENERGY_MONITOR_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
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
  class KineticEnergyMonitorPrivate;

  /// \brief A system that monitors the kinetic energy of a link in a model
  /// and publishes when there is a lost of kinetic energy during a timestep
  /// that surpasses a specific threshold.
  /// This system can be used to detect when a model could be damaged.
  ///
  /// ## System Parameters
  ///
  /// - `<link_name>`: Name of the link to monitor. This name must match
  /// a name of link within the model.
  ///
  /// - `<kinetic_energy_threshold>`: Threshold, in Joule (J), after which
  /// a message is generated on `<topic>` with the kinetic energy value that
  /// surpassed the threshold.
  ///
  /// - `<topic>`: Custom topic that this system will publish to when kinetic
  /// energy surpasses the threshold. This element if optional, and the
  /// default value is `/model/{name_of_model}/kinetic_energy`.
  ///
  /// ## Example Usage
  ///
  /** \verbatim
   <model name="sphere">
      <pose>0 0 5 0 0 0</pose>
      <link name="sphere_link">
        <inertial>
          <inertia>
            <ixx>3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>3</iyy>
            <iyz>0</iyz>
            <izz>3</izz>
          </inertia>
          <mass>3.0</mass>
        </inertial>
        <collision name="sphere_collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="sphere_visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
            <specular>0 0 1 1</specular>
          </material>
        </visual>
      </link>
      <plugin
        filename="gz-sim-kinetic-energy-monitor-system"
        name="gz::sim::systems::KineticEnergyMonitor">
        <base_link_name>sphere_link</base_link_name>
        <kinetic_energy_threshold>100</kinetic_energy_threshold>
      </plugin>
    </model>
  \endverbatim */
  class KineticEnergyMonitor final:
    public System,
    public ISystemConfigure,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: KineticEnergyMonitor();

    /// \brief Destructor
    public: ~KineticEnergyMonitor() final;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<KineticEnergyMonitorPrivate> dataPtr;
  };
}
}
}
}
#endif
