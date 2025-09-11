/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
 * Author: Adarsh Karan K P, Neobotix GmbH
 * 
 */

#ifndef GZ_SIM_SYSTEMS_DYNAMICDETACHABLEJOINT_HH_
#define GZ_SIM_SYSTEMS_DYNAMICDETACHABLEJOINT_HH_

#include <gz/msgs/dynamic_detachable_joint.pb.h>

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
/// \brief A system that dynamically creates detachable joints between models
/// via a service interface, allowing runtime attachment and detachment of any
/// model with a parent model that is defined in the SDF/URDF.
///
/// Unlike the standard DetachableJoint which requires child model specification
/// in the SDF, this system accepts child model and link names via service calls.
/// Hence, it is a more flexible extension of the existing DetachableJoint system.
///
/// ## System Parameters
///
/// - `<parent_link>`: Name of the link in the parent model to be used in
///   creating a fixed joint with a link in the child model. Required.
///
/// - `<attach_distance>` (optional): Maximum distance in meters between parent
///   and child links to allow attachment. Defaults to 0.1 meters.
///
/// - `<service_name>` (optional): Service name for attach/detach requests.
///   Defaults to `/model/{model_name}/dynamic_detachable_joint/attach_detach`.
///
/// - `<output_topic>` (optional): Topic name for publishing attachment state.
///   Defaults to `/model/{model_name}/detachable_joint/state`.
///
/// - `<suppress_child_warning>` (optional): If true, the system
/// will not print a warning message if a child model does not exist yet.
/// Otherwise, a warning message is printed. Defaults to false.

  class DynamicDetachableJoint
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// Documentation inherited
    public: DynamicDetachableJoint() = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Gazebo communication node.
    private: transport::Node node;

    /// \brief A publisher to send state of the detachment
    private: transport::Node::Publisher outputPub;

    /// \brief Callback for attach/detach service request
    /// \param[in] _req Request message containing child model/link names and command
    /// \param[out] _res Response message with success status and message
    /// \return Always returns true to indicate message was processed
    private: bool OnServiceRequest(const gz::msgs::AttachDetachRequest &_req,
                                  gz::msgs::AttachDetachResponse &_res);
    
    /// \brief Helper function to publish the state of the detachment
    private: void PublishJointState(bool attached);

    /// \brief Toggle the attach state.
    private: void OnAttachRequest(const msgs::Empty &_msg);

    /// \brief Toggle the detach state.
    private: void OnDetachRequest(const msgs::Empty &_msg);

    /// \brief The model associated with this system.
    private: Model model;

    /// \brief Name of child model
    private: std::string childModelName;

    /// \brief Name of attachment link in the child model
    private: std::string childLinkName;

    /// \brief Minimum distance to consider the child link as attached
    private: double defaultAttachDistance = 0.1;
    
    /// \brief Distance to consider the child link as attached
    private: double attachDistance{defaultAttachDistance};
    /// \brief Service name to be used for attaching or detaching connections
    private: std::string serviceName;

    /// \brief Topic to be used for publishing detached state
    private: std::string outputTopic;

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

    /// \brief Whether attachment has been requested
    private: std::atomic<bool> attachRequested{false};

    /// \brief Whether child entity is attached
    private: std::atomic<bool> isAttached{false};

    /// \brief Whether all parameters are valid and the system can proceed
    private: bool validConfig{false};

  };
  }
}
}
}

#endif
