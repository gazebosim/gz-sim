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

#ifndef GZ_SIM_SYSTEMS_OPTICAL_TACTILE_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_OPTICAL_TACTILE_PLUGIN_HH_

#include <memory>
#include <gz/sim/System.hh>
#include "Visualization.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class OpticalTactilePluginPrivate;

  /// \brief Plugin that implements an optical tactile sensor
  ///
  /// It requires that contact sensor and depth camera be placed in at least
  /// one link on the model on which this plugin is attached.
  ///
  /// \todo(anyone)
  /// Currently, the contacts returned from the physics engine (which tends to
  /// be sparse) and the normal forces separately computed from the depth
  /// camera (which is dense, resolution adjustable) are disjoint. It is
  /// left as future work to combine the two sets of data.
  ///
  /// ## System Parameters
  ///
  /// - `<enabled>`: Set this to true so the plugin works from the start and
  /// doesn't need to be enabled. This element is optional, and the
  /// default value is true.
  ///
  /// - `<namespace>`: Namespace for transport topics/services. If there are
  /// more than one optical tactile plugins, their namespaces should
  /// be different. This element is optional, and the default value is
  /// "optical_tactile_sensor".
  ///   - `/<namespace>/enable`: Service used to enable and disable the plugin.
  ///   - `/<namespace>/normal_forces`: Topic where a message is
  ///     published each time the normal forces are computed
  ///
  /// - `<visualization_resolution>`: Number n of pixels to skip when
  /// visualizing the forces. One vector representing a normal
  /// force is computed for every nth pixel. This
  /// element must be positive and it is optional.
  /// The default value is 30.
  ///
  /// - `<force_length>`: Length in meters of the forces visualized if
  /// <visualize_forces> is set to true. This parameter is
  /// optional, and the default value is 0.01.
  ///
  /// - `<extended_sensing>`: Extended sensing distance in meters. The sensor
  /// will output data coming from its collision geometry plus
  /// this distance. This element is optional, and the
  /// default value is 0.001.
  ///
  /// - `<visualize_sensor>`: Whether to visualize the sensor or not. This
  /// element is optional, and the default value is false.
  ///
  /// - `<visualize_contacts>`: Whether to visualize the contacts from the
  /// contact sensor based on physics. This element is optional,
  /// and the default value is false.
  ///
  /// - `<visualize_forces>`: Whether to visualize normal forces computed from
  /// the depth camera. This element is optional, and the
  /// default value is false.

  class OpticalTactilePlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: OpticalTactilePlugin();

    /// \brief Destructor
    public: ~OpticalTactilePlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<OpticalTactilePluginPrivate> dataPtr;
  };
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
