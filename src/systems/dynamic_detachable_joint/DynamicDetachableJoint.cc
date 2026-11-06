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

#include "DynamicDetachableJoint.hh"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/////////////////////////////////////////////////
void DynamicDetachableJoint::Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &/*_eventMgr*/)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "DynamicDetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("parent_link"))
  {
    this->defaultParentLinkName = _sdf->Get<std::string>("parent_link");
    Entity parentLinkEntity =
        this->FindParentLinkEntity(this->defaultParentLinkName, _ecm);
    if (kNullEntity == parentLinkEntity)
    {
      gzerr << "Link with name " << this->defaultParentLinkName
             << " not found in model " << this->model.Name(_ecm)
             << ". Make sure the parameter 'parent_link' has the "
             << "correct value. Failed to initialize.\n";
      return;
    }
  }
  else
  {
    gzdbg << "No default 'parent_link' specified in SDF for "
          << "DynamicDetachableJoint on model [" << this->model.Name(_ecm)
          << "]. 'parent_link_name' must be provided in service requests.\n";
  }

  // Setup attach distance threshold
  auto [value, found] = _sdf->Get<double>(
      "attach_distance", this->defaultAttachDistance);
  if (!found)
  {
    gzwarn << "No 'attach_distance' specified in sdf, using default value of "
          << this->defaultAttachDistance << " meters.\n";
  }
  else
  {
    gzmsg << "Found 'attach_distance' in sdf: " << value << " meters.\n";
  }

  this->attachDistance = value;
  gzmsg << "Final attachDistance set to: " << this->attachDistance
        << " meters.\n";

  // Setup service
  // Check if the SDF has a service_name element
  std::vector<std::string> serviceNames;
  if (_sdf->HasElement("service_name"))
  {
    // If it does, add it to the list of service names
    serviceNames.push_back(_sdf->Get<std::string>("service_name"));
  }
  // Add a fallback service name
  serviceNames.push_back("/model/" + this->model.Name(_ecm) +
      "/dynamic_detachable_joint/attach_detach");

  // Get the valid service name
  this->serviceName = validTopic(serviceNames);
  if (this->serviceName.empty())
  {
    gzerr << "No valid service name for DynamicDetachableJoint could be "
          << "found.\n";
    return;
  }
  gzdbg << "Using service: " << this->serviceName << std::endl;

  // Advertise the service
  if (!this->node.Advertise(
      this->serviceName, &DynamicDetachableJoint::OnServiceRequest, this))
  {
    gzerr << "Error advertising service [" << this->serviceName << "]"
          << std::endl;
    return;
  }

  // Advertise the blocking service
  const std::string blockingServiceName = this->serviceName + "/blocking";
  if (!this->node.Advertise(
      blockingServiceName,
      &DynamicDetachableJoint::OnBlockingServiceRequest, this))
  {
    gzerr << "Error advertising service [" << blockingServiceName << "]"
          << std::endl;
    return;
  }

  // Setup output topic
  std::vector<std::string> outputTopics;
  if (_sdf->HasElement("output_topic"))
  {
    outputTopics.push_back(_sdf->Get<std::string>("output_topic"));
  }

  outputTopics.push_back("/model/" + this->model.Name(_ecm) +
      "/dynamic_detachable_joint/state");

  this->outputTopic = validTopic(outputTopics);
  if (this->outputTopic.empty())
  {
    gzerr << "No valid output topics for DynamicDetachableJoint could be "
          << "found.\n";
    return;
  }
  gzdbg << "Output topic is: " << this->outputTopic << std::endl;

  // Setup publisher for output topic
  this->outputPub = this->node.Advertise<gz::msgs::Entity>(
      this->outputTopic);
  if (!this->outputPub)
  {
    gzerr << "Error advertising topic [" << this->outputTopic << "]"
              << std::endl;
    return;
  }

  this->validConfig = true;
}

//////////////////////////////////////////////////
void DynamicDetachableJoint::PreUpdate(
  const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("DynamicDetachableJoint::PreUpdate");
  if (!this->validConfig)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  // Clean up any active joints whose parent link, child link, or joint entity
  // was removed externally.
  for (auto it = this->attachedJoints.begin();
       it != this->attachedJoints.end();)
  {
    const auto &key = it->first;
    const auto &info = it->second;
    if (!_ecm.HasEntity(info.parentLinkEntity) ||
        !_ecm.HasEntity(info.childLinkEntity) ||
        !_ecm.HasEntity(info.jointEntity))
    {
      if (kNullEntity != info.jointEntity && _ecm.HasEntity(info.jointEntity))
      {
        _ecm.RequestRemoveEntity(info.jointEntity);
      }
      gzdbg << "Attached entity for parent link [" << key.parentLinkName
            << "] and child model [" << key.childModelName << "] link ["
            << key.childLinkName << "] was removed externally. "
            << "Cleaning up detachable joint." << std::endl;
      this->PublishJointState(kNullEntity);
      it = this->attachedJoints.erase(it);
    }
    else
    {
      ++it;
    }
  }

  // Return if no attach or detach requests are pending.
  if (this->pendingRequests.empty())
  {
    return;
  }

  std::vector<std::shared_ptr<PendingRequest>> requests;
  requests.swap(this->pendingRequests);

  for (auto &reqPtr : requests)
  {
    gz::msgs::Result res;
    const auto &key = reqPtr->key;

    if (gz::msgs::AttachDetachRequest::ATTACH == reqPtr->command)
    {
      // Only allow attaching if this parent/child pair is detached
      if (this->attachedJoints.find(key) != this->attachedJoints.end())
      {
        std::string msg = "Already attached parent link [" +
            key.parentLinkName + "] to child model [" + key.childModelName +
            "] at link [" + key.childLinkName + "].";
        gzdbg << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      // Look for the parent link
      Entity parentLinkEntity =
          this->FindParentLinkEntity(key.parentLinkName, _ecm);
      if (kNullEntity == parentLinkEntity)
      {
        std::string msg = "Attach Failed. Parent link [" +
            key.parentLinkName + "] could not be found in model [" +
            this->model.Name(_ecm) + "].";
        gzerr << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      // Look for the child model and link
      Entity modelEntity = this->FindChildModelEntity(
          key.childModelName, key.childLinkName, _ecm);

      // if child model is not found
      if (kNullEntity == modelEntity)
      {
        std::string msg = "Attach Failed. child model [" +
            key.childModelName + "] could not be found.";
        gzerr << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      Entity childLinkEntity = this->FindChildLinkEntity(
          modelEntity, key.childLinkName, _ecm);

      // if child link is not found
      if (kNullEntity == childLinkEntity)
      {
        std::string msg = "Attach Failed. child link [" +
            key.childLinkName + "] could not be found.";
        gzerr << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      // store the child and parent link poses in the world frame
      math::Pose3d childPose = gz::sim::worldPose(childLinkEntity, _ecm);
      math::Pose3d parentPose = gz::sim::worldPose(parentLinkEntity, _ecm);

      auto dist = childPose.Pos().Distance(parentPose.Pos());
      gzdbg << "Centre-to-centre distance: " << dist << " m" << std::endl;

      // Check if the child link is within the attach distance
      if (dist > this->attachDistance)
      {
        std::string msg = "Attach Failed. Child Link [" + key.childLinkName +
            "] is too far from parent link [" + key.parentLinkName +
            "]. Distance: " + std::to_string(dist) + "m, threshold: " +
            std::to_string(this->attachDistance) + "m";
        gzerr << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      // If the child link is within the attach distance, proceed to attach
      std::string msg = "Attach Success. Child model [" + key.childModelName +
          "] link [" + key.childLinkName + "] attached to parent link [" +
          key.parentLinkName + "]. Distance: " + std::to_string(dist) + "m";
      gzmsg << msg << std::endl;

      // Attach the models
      // We do this by creating a detachable joint entity.
      Entity jointEntity = _ecm.CreateEntity();

      // creating the joint
      _ecm.CreateComponent(
          jointEntity,
          components::DetachableJoint({parentLinkEntity,
                                        childLinkEntity, "fixed"}));

      // Keep track of the attached pair for future validation
      this->attachedJoints[key] =
          AttachedJointInfo{jointEntity, parentLinkEntity, childLinkEntity};
      this->PublishJointState(childLinkEntity);
      gzdbg << "Attaching entity: " << jointEntity << std::endl;

      res.set_error_code(0);
      res.set_message(msg);
      reqPtr->promise.set_value(res);
    }
    else if (gz::msgs::AttachDetachRequest::DETACH == reqPtr->command)
    {
      // only allow detaching if child entity is attached
      auto it = this->attachedJoints.find(key);
      if (it == this->attachedJoints.end())
      {
        std::string msg = "Detach rejected: parent link [" +
            key.parentLinkName + "] is not attached to child model [" +
            key.childModelName + "] link [" + key.childLinkName + "].";
        gzerr << msg << std::endl;
        res.set_error_code(1);
        res.set_message(msg);
        reqPtr->promise.set_value(res);
        continue;
      }

      // Detach the models
      std::string msg = "Detach Success. Child model [" + key.childModelName +
          "] link [" + key.childLinkName + "] detached from parent link [" +
          key.parentLinkName + "].";
      gzmsg << msg << std::endl;
      if (kNullEntity != it->second.jointEntity &&
          _ecm.HasEntity(it->second.jointEntity))
      {
        gzdbg << "Removing entity: " << it->second.jointEntity << std::endl;
        _ecm.RequestRemoveEntity(it->second.jointEntity);
      }

      this->attachedJoints.erase(it);
      this->PublishJointState(kNullEntity);
      res.set_error_code(0);
      res.set_message(msg);
      reqPtr->promise.set_value(res);
    }
  }
}

//////////////////////////////////////////////////
void DynamicDetachableJoint::Reset(
    const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("DynamicDetachableJoint::Reset");
  std::lock_guard<std::mutex> lock(this->mutex);

  for (const auto &[key, info] : this->attachedJoints)
  {
    if (kNullEntity != info.jointEntity && _ecm.HasEntity(info.jointEntity))
    {
      _ecm.RequestRemoveEntity(info.jointEntity);
    }
  }

  if (!this->attachedJoints.empty())
  {
    this->PublishJointState(kNullEntity);
    this->attachedJoints.clear();
  }

  for (auto &reqPtr : this->pendingRequests)
  {
    gz::msgs::Result res;
    res.set_error_code(1);
    res.set_message("Request cancelled due to simulation reset.");
    reqPtr->promise.set_value(res);
  }
  this->pendingRequests.clear();
}

//////////////////////////////////////////////////
bool DynamicDetachableJoint::OnServiceRequest(
    const gz::msgs::AttachDetachRequest &_req,
    gz::msgs::Result &_res)
{
  GZ_PROFILE("DynamicDetachableJoint::OnServiceRequest");
  std::lock_guard<std::mutex> lock(this->mutex);

  std::shared_ptr<PendingRequest> pendingReq;
  if (this->ValidateAndCreatePendingRequest(_req, _res, pendingReq))
  {
    this->pendingRequests.push_back(pendingReq);
  }
  return true;
}

//////////////////////////////////////////////////
bool DynamicDetachableJoint::OnBlockingServiceRequest(
    const gz::msgs::AttachDetachRequest &_req,
    gz::msgs::Result &_res)
{
  GZ_PROFILE("DynamicDetachableJoint::OnBlockingServiceRequest");
  std::shared_ptr<PendingRequest> pendingReq;
  std::future<gz::msgs::Result> future;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->ValidateAndCreatePendingRequest(_req, _res, pendingReq))
    {
      return true;
    }
    future = pendingReq->promise.get_future();
    this->pendingRequests.push_back(pendingReq);
  }

  if (future.wait_for(this->blockingTimeout) == std::future_status::ready)
  {
    _res = future.get();
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto it = std::find(
        this->pendingRequests.begin(),
        this->pendingRequests.end(),
        pendingReq);
    if (it != this->pendingRequests.end())
    {
      this->pendingRequests.erase(it);
      _res.set_error_code(1);
      _res.set_message(
          "Timed out waiting for simulation update to process request.");
      return true;
    }
  }

  _res = future.get();
  return true;
}

//////////////////////////////////////////////////
bool DynamicDetachableJoint::ValidateAndCreatePendingRequest(
    const gz::msgs::AttachDetachRequest &_req,
    gz::msgs::Result &_res,
    std::shared_ptr<PendingRequest> &_pendingReq)
{
  // Resolve the parent link name (request overrides SDF default)
  const std::string parentLinkName = _req.parent_link_name().empty() ?
      this->defaultParentLinkName : _req.parent_link_name();

  if (parentLinkName.empty())
  {
    _res.set_error_code(1);
    _res.set_message(
        "Invalid request: parent_link_name is not set and no default "
        "<parent_link> was configured.");
    return false;
  }

  // Check if the request is valid
  if (_req.child_model_name().empty() || _req.child_link_name().empty())
  {
    _res.set_error_code(1);
    _res.set_message(
        "Invalid request: child_model_name and child_link_name must be set.");
    return false;
  }

  JointKey key{
      parentLinkName, _req.child_model_name(), _req.child_link_name()};

  switch (_req.command())
  {
    case gz::msgs::AttachDetachRequest::ATTACH:
    {
      // If attach is requested
      if (this->attachedJoints.find(key) != this->attachedJoints.end())
      {
        _res.set_error_code(1);
        _res.set_message("Already attached parent link [" +
                         key.parentLinkName + "] to child model [" +
                         key.childModelName + "] at link [" +
                         key.childLinkName + "].");
        gzdbg << _res.message() << std::endl;
        return false;
      }

      // Queue the attach request
      _pendingReq = std::make_shared<PendingRequest>();
      _pendingReq->key = key;
      _pendingReq->command = gz::msgs::AttachDetachRequest::ATTACH;
      _res.set_error_code(0);
      _res.set_message("Attach request accepted for parent link [" +
                       key.parentLinkName + "] and child model [" +
                       key.childModelName + "] at link [" +
                       key.childLinkName + "].");
      return true;
    }

    case gz::msgs::AttachDetachRequest::DETACH:
    {
      // If detach is requested
      bool pendingAttachExists = std::any_of(
          this->pendingRequests.begin(),
          this->pendingRequests.end(),
          [&key](const std::shared_ptr<PendingRequest> &_r)
          {
            return _r->command == gz::msgs::AttachDetachRequest::ATTACH &&
                   !(_r->key < key) && !(key < _r->key);
          });

      if (this->attachedJoints.empty() && !pendingAttachExists)
      {
        _res.set_error_code(1);
        _res.set_message(
            "Detach request rejected: no child link is currently attached.");
        gzdbg << "Already detached" << std::endl;
        return false;
      }

      // Validate that the request matches what is actually attached.
      if (this->attachedJoints.find(key) == this->attachedJoints.end() &&
          !pendingAttachExists)
      {
        _res.set_error_code(1);
        _res.set_message(
            "Detach rejected: parent link [" + key.parentLinkName +
            "] is not attached to child model [" + key.childModelName +
            "] link [" + key.childLinkName + "].");
        gzerr << _res.message() << std::endl;
        return false;
      }

      _pendingReq = std::make_shared<PendingRequest>();
      _pendingReq->key = key;
      _pendingReq->command = gz::msgs::AttachDetachRequest::DETACH;
      _res.set_error_code(0);
      _res.set_message("Detach request accepted for parent link [" +
                       key.parentLinkName + "] and child model [" +
                       key.childModelName + "] at link [" +
                       key.childLinkName + "].");
      return true;
    }

    case gz::msgs::AttachDetachRequest::COMMAND_UNSPECIFIED:
    default:
    {
      _res.set_error_code(1);
      _res.set_message("Invalid request: command must be ATTACH or DETACH.");
      return false;
    }
  }
}

//////////////////////////////////////////////////
Entity DynamicDetachableJoint::FindParentLinkEntity(
    const std::string &_parentLinkName,
    const EntityComponentManager &_ecm) const
{
  Entity linkEntity = this->model.LinkByName(_ecm, _parentLinkName);
  if (kNullEntity != linkEntity)
  {
    return linkEntity;
  }

  auto entities = entitiesFromScopedName(
      _parentLinkName, _ecm, this->model.Entity());
  for (const auto &entity : entities)
  {
    if (_ecm.EntityHasComponentType(entity, components::Link::typeId))
    {
      return entity;
    }
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
Entity DynamicDetachableJoint::FindChildModelEntity(
    const std::string &_childModelName,
    const std::string &_childLinkName,
    const EntityComponentManager &_ecm) const
{
  // if child model is the parent model
  if ("__model__" == _childModelName)
  {
    return this->model.Entity();
  }

  // Querying the ECM for a top-level child model entity first
  Entity directModel = _ecm.EntityByComponents(
      components::Model(), components::Name(_childModelName));
  if (kNullEntity != directModel &&
      kNullEntity !=
          this->FindChildLinkEntity(directModel, _childLinkName, _ecm))
  {
    return directModel;
  }

  // Querying the ECM by scoped name to support nested models
  auto entitiesMatchingName = entitiesFromScopedName(_childModelName, _ecm);
  std::vector<Entity> candidateModels;
  for (const auto &entity : entitiesMatchingName)
  {
    if (_ecm.EntityHasComponentType(entity, components::Model::typeId))
    {
      candidateModels.push_back(entity);
    }
  }

  if (candidateModels.size() == 1)
  {
    return candidateModels.front();
  }

  for (const auto &entity : candidateModels)
  {
    if (kNullEntity != this->FindChildLinkEntity(entity, _childLinkName, _ecm))
    {
      return entity;
    }
  }

  if (!candidateModels.empty())
  {
    return candidateModels.front();
  }

  return directModel;
}

//////////////////////////////////////////////////
Entity DynamicDetachableJoint::FindChildLinkEntity(
    Entity _modelEntity,
    const std::string &_childLinkName,
    const EntityComponentManager &_ecm) const
{
  Entity linkEntity = _ecm.EntityByComponents(
      components::Link(),
      components::ParentEntity(_modelEntity),
      components::Name(_childLinkName));
  if (kNullEntity != linkEntity)
  {
    return linkEntity;
  }

  auto entities = entitiesFromScopedName(
      _childLinkName, _ecm, _modelEntity);
  for (const auto &entity : entities)
  {
    if (_ecm.EntityHasComponentType(entity, components::Link::typeId))
    {
      return entity;
    }
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
void DynamicDetachableJoint::PublishJointState(Entity _childLinkEntity)
{
  gz::msgs::Entity stateMsg;
  if (kNullEntity != _childLinkEntity)
  {
    stateMsg.set_id(_childLinkEntity);
    stateMsg.set_type(gz::msgs::Entity::LINK);
  }
  else
  {
    stateMsg.set_id(kNullEntity);
    stateMsg.set_type(gz::msgs::Entity::NONE);
  }
  this->outputPub.Publish(stateMsg);
}

GZ_ADD_PLUGIN(DynamicDetachableJoint,
                    System,
                    DynamicDetachableJoint::ISystemConfigure,
                    DynamicDetachableJoint::ISystemReset,
                    DynamicDetachableJoint::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DynamicDetachableJoint,
  "gz::sim::systems::DynamicDetachableJoint")
