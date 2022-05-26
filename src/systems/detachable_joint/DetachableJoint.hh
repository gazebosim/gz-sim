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

#ifndef GZ_SIM_SYSTEMS_DETACHABLEJOINT_HH_
#define GZ_SIM_SYSTEMS_DETACHABLEJOINT_HH_

#include <gz/msgs/empty.pb.h>

#include <memory>
#include <string>
#include <gz/transport/Node.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  /// \brief A system that initially attaches two models via a fixed joint and
  /// allows for the models to get detached during simulation via a topic.
  ///
  /// Parameters:
  ///
  /// - `<parent_link>`: Name of the link in the parent model to be used in
  /// creating a fixed joint with a link in the child model.
  ///
  /// - `<child_model>`: Name of the model to which this model will be connected
  ///
  /// - `<child_link>`: Name of the link in the child model to be used in
  /// creating a fixed joint with a link in the parent model.
  ///
  /// - `<topic>` (optional): Topic name to be used for detaching connections
  ///
  /// - `<suppress_child_warning>` (optional): If true, the system
  /// will not print a warning message if a child model does not exist yet.
  /// Otherwise, a warning message is printed. Defaults to false.

  class DetachableJoint
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// Documentation inherited
    public: DetachableJoint() = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Callback for detach request topic
    private: void OnDetachRequest(const msgs::Empty &_msg);

    /// \brief The model associated with this system.
    private: Model model;

    /// \brief Name of child model
    private: std::string childModelName;

    /// \brief Name of attachment link in the child model
    private: std::string childLinkName;

    /// \brief Topic to be used for detaching connections
    private: std::string topic;

    /// \brief Whether to suppress warning about missing child model.
    private: bool suppressChildWarning{false};

    /// \brief Entity of attachment link in the parent model
    private: Entity parentLinkEntity{kNullEntity};

    /// \brief Entity of attachment link in the child model
    private: Entity childLinkEntity{kNullEntity};

    /// \brief Entity of the detachable joint created by this system
    private: Entity detachableJointEntity{kNullEntity};

    /// \brief Whether detachment has been requested
    private: std::atomic<bool> detachRequested{false};

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Whether all parameters are valid and the system can proceed
    private: bool validConfig{false};

    /// \brief Whether the system has been initialized
    private: bool initialized{false};
  };
  }
}
}
}

#endif
