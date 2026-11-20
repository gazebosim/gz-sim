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

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <gz/msgs/dynamic_detachable_joint.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/result.pb.h>

#include <gz/transport/Node.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"

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
/// in the SDF, this system accepts parent link, child model, and child link
/// names via service calls. Hence, it is a more flexible extension of the
/// existing DetachableJoint system and can manage multiple parent/child
/// attachment pairs simultaneously.
///
/// Both an asynchronous service (`<service_name>`) and a blocking service
/// (`<service_name>/blocking`) are advertised. The blocking service waits for
/// the next simulation `PreUpdate` step to execute the attachment or
/// detachment and returns any error encountered while creating or removing the
/// joint.
///
/// ## System Parameters
///
/// - `<attach_distance>` (optional): Maximum distance in meters between parent
///   and child links to allow attachment. If not specified, distance is not
///   checked and attachment is always allowed regardless of distance.
///
/// - `<service_name>` (optional): Service name for attach/detach requests.
///   Defaults to `/model/{model_name}/dynamic_detachable_joint/attach_detach`.
///   A blocking variant is also advertised at `<service_name>/blocking`.
///
/// - `<output_topic>` (optional): Topic name for publishing attachment state.
///   Defaults to `/model/{model_name}/dynamic_detachable_joint/state`.
  class DynamicDetachableJoint
      : public System,
        public ISystemConfigure,
        public ISystemReset,
        public ISystemPreUpdate
  {
    /// \brief Key uniquely identifying a parent/child attachment pair.
    private: struct JointKey
    {
      /// \brief Name of the link in the parent model.
      std::string parentLinkName;

      /// \brief Name of the child model.
      std::string childModelName;

      /// \brief Name of the link in the child model.
      std::string childLinkName;

      /// \brief Less-than operator for use in std::map.
      /// \param[in] _other Other key to compare against.
      /// \return True if this key is ordered before _other.
      bool operator<(const JointKey &_other) const
      {
        return std::tie(this->parentLinkName,
                        this->childModelName,
                        this->childLinkName) <
               std::tie(_other.parentLinkName,
                        _other.childModelName,
                        _other.childLinkName);
      }
    };

    /// \brief Entities associated with an active detachable joint.
    private: struct AttachedJointInfo
    {
      /// \brief Detachable joint entity.
      Entity jointEntity{kNullEntity};

      /// \brief Parent link entity.
      Entity parentLinkEntity{kNullEntity};

      /// \brief Child link entity.
      Entity childLinkEntity{kNullEntity};
    };

    /// \brief A pending attach or detach request to be processed in PreUpdate.
    private: struct PendingRequest
    {
      /// \brief Requested parent/child pair.
      JointKey key;

      /// \brief Requested command (ATTACH or DETACH).
      gz::msgs::AttachDetachRequest::Command command{
          gz::msgs::AttachDetachRequest::COMMAND_UNSPECIFIED};

      /// \brief Promise fulfilled in PreUpdate with the execution result.
      std::promise<gz::msgs::Result> promise;
    };

    /// Documentation inherited
    public: DynamicDetachableJoint() = default;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    /// Documentation inherited
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) final;

    /// \brief Asynchronous callback for attach/detach service request.
    /// \param[in] _req Request message containing parent/child link names and
    /// command.
    /// \param[out] _res Response message with error code and message.
    /// \return Always returns true to indicate message was processed.
    private: bool OnServiceRequest(const gz::msgs::AttachDetachRequest &_req,
                                   gz::msgs::Result &_res);

    /// \brief Blocking callback for attach/detach service request that waits
    /// for PreUpdate to execute the request.
    /// \param[in] _req Request message containing parent/child link names and
    /// command.
    /// \param[out] _res Response message with error code and message.
    /// \return Always returns true to indicate message was processed.
    private: bool OnBlockingServiceRequest(
                 const gz::msgs::AttachDetachRequest &_req,
                 gz::msgs::Result &_res);

    /// \brief Validate a service request and create a PendingRequest.
    /// Must be called with this->mutex held.
    /// \param[in] _req Incoming service request.
    /// \param[out] _res Populated with acceptance or rejection details.
    /// \param[out] _pendingReq Created pending request if valid.
    /// \return True if the request is valid and should be queued.
    private: bool ValidateAndCreatePendingRequest(
                 const gz::msgs::AttachDetachRequest &_req,
                 gz::msgs::Result &_res,
                 std::shared_ptr<PendingRequest> &_pendingReq);

    /// \brief Find a parent link entity by name (direct or scoped) within
    /// this->model.
    /// \param[in] _parentLinkName Link name or scoped name.
    /// \param[in] _ecm EntityComponentManager.
    /// \return Link entity, or kNullEntity if not found.
    private: Entity FindParentLinkEntity(
                 const std::string &_parentLinkName,
                 const EntityComponentManager &_ecm) const;

    /// \brief Find a child model entity by name or scoped name.
    /// \param[in] _childModelName Model name or scoped name.
    /// \param[in] _childLinkName Child link name used to disambiguate models.
    /// \param[in] _ecm EntityComponentManager.
    /// \return Model entity, or kNullEntity if not found.
    private: Entity FindChildModelEntity(
                 const std::string &_childModelName,
                 const std::string &_childLinkName,
                 const EntityComponentManager &_ecm) const;

    /// \brief Find a child link entity by name or scoped name within a model.
    /// \param[in] _modelEntity Child model entity.
    /// \param[in] _childLinkName Link name or scoped name.
    /// \param[in] _ecm EntityComponentManager.
    /// \return Link entity, or kNullEntity if not found.
    private: Entity FindChildLinkEntity(
                 Entity _modelEntity,
                 const std::string &_childLinkName,
                 const EntityComponentManager &_ecm) const;

    /// \brief Helper function to publish the state of the attachment.
    /// \param[in] _childLinkEntity Entity of the attached child link, or
    /// kNullEntity when detached.
    private: void PublishJointState(Entity _childLinkEntity);

    /// \brief Gazebo communication node.
    private: transport::Node node;

    /// \brief A publisher to send state of the detachment.
    private: transport::Node::Publisher outputPub;

    /// \brief The model associated with this system.
    private: Model model;

    /// \brief Active detachable joints keyed by parent/child pair.
    private: std::map<JointKey, AttachedJointInfo> attachedJoints;

    /// \brief Queue of pending requests to process in PreUpdate.
    private: std::vector<std::shared_ptr<PendingRequest>> pendingRequests;

    /// \brief Optional maximum distance to consider the child link as
    /// attachable. If std::nullopt, distance is not checked.
    private: std::optional<double> attachDistance;

    /// \brief Timeout for blocking service requests.
    private: std::chrono::milliseconds blockingTimeout{5000};

    /// \brief Service name to be used for attaching or detaching connections.
    private: std::string serviceName;

    /// \brief Topic to be used for publishing detached state.
    private: std::string outputTopic;

    /// \brief Whether all parameters are valid and the system can proceed.
    private: bool validConfig{false};

    /// \brief Mutex to protect access to member variables.
    private: std::mutex mutex;
  };
  }
}
}
}

#endif
