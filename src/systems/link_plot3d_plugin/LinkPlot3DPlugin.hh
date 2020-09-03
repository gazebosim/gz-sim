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

#ifndef IGNITION_GAZEBO_SYSTEMS_LINK_PLOT3D_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_LINK_PLOT3D_PLUGIN_HH_

#include <memory>
#include <vector>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief The LinkPlot3DPlugin system use the marker to trace the
  /// of a model in the simulation
  /// TODO complete
  class IGNITION_GAZEBO_VISIBLE LinkPlot3DPlugin
      : public System,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: LinkPlot3DPlugin();

    /// \brief Destructor
    public: ~LinkPlot3DPlugin() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &,
        EntityComponentManager &_ecm, EventManager &) override;

    // Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief The publisher
    private: std::unique_ptr<transport::Node::Publisher> modelPub;

    /// \brief The joints that will be published.
    private: std::set<Entity> joints;

    /// \brief Ignition communication node. 
    private: transport::Node node;

    /// \brief Joint indexes to be used.
    private: std::vector<unsigned int> jointIndexs = [0u];
  }
}
}
}
}

#endif