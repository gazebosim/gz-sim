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

#include <string>
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
    auto parentLinkName = _sdf->Get<std::string>("parent_link");
    this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
    if (kNullEntity == this->parentLinkEntity)
    {
      gzerr << "Link with name " << parentLinkName
             << " not found in model " << this->model.Name(_ecm)
             << ". Make sure the parameter 'parent_link' has the "
             << "correct value. Failed to initialize.\n";
      return;
    }
  }
  else
  {
    gzerr << "'parent_link' is a required parameter for DynamicDetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  // Setup attach distance threshold
  auto [value, found] = _sdf->Get<double>("attach_distance", this->defaultAttachDistance);
  if (!found)
  {
    gzmsg << "No 'attach_distance' specified in sdf, using default value of "
          << this->defaultAttachDistance << " meters.\n";
  }
  else
  {
    gzmsg << "Found 'attach_distance' in sdf: " << value << " meters.\n";
  }

  this->attachDistance = value;
  gzmsg << "Final attachDistance set to: " << this->attachDistance << " meters.\n";

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
    gzerr << "No valid service name for DynamicDetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Using service: " << this->serviceName << std::endl;

  // Advertise the service
  if (!this->node.Advertise(this->serviceName, &DynamicDetachableJoint::OnServiceRequest, this))
  {
    gzerr << "Error advertising service [" << this->serviceName << "]" << std::endl;
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
    gzerr << "No valid output topics for DynamicDetachableJoint could be found.\n";
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

  // Suppress Child Warning
  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  this->validConfig = true;
}

//////////////////////////////////////////////////
void DynamicDetachableJoint::PreUpdate(
  const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("DynamicDetachableJoint::PreUpdate");
  // only allow attaching if child entity is detached
  if (this->validConfig && !this->isAttached)
  {
    // return if attach is not requested.
    if (!this->attachRequested)
    {
      return;
    }
    // Look for the child model and link
    Entity modelEntity{kNullEntity};

    // if child model is the parent model
    if ("__model__" == this->childModelName)
    {
      modelEntity = this->model.Entity();
    }
    else
    {
      // Querying the ECM for the child model entity
      modelEntity = _ecm.EntityByComponents(
          components::Model(), components::Name(this->childModelName));
    }

    // if child model is not found
    if (kNullEntity == modelEntity)
    {
      if (!this->suppressChildWarning)
      {
        gzwarn << "Child Model " << this->childModelName
               << " could not be found.\n";
      }
      return;
    }

    this->childLinkEntity = _ecm.EntityByComponents(
        components::Link(),
        components::ParentEntity(modelEntity),
        components::Name(this->childLinkName));
    
    // if child link is not found
    if (kNullEntity == this->childLinkEntity)
    {
      gzwarn << "Child Link " << this->childLinkName
              << " could not be found.\n";
      return;
    }

    // store the child and parent link poses in the world frame
    math::Pose3d childPose = gz::sim::worldPose(this->childLinkEntity, _ecm);
    math::Pose3d parentPose = gz::sim::worldPose(this->parentLinkEntity, _ecm);

    auto dist = childPose.Pos().Distance(parentPose.Pos());
    gzdbg << "Centre-to-centre distance: " << dist << " m" << std::endl;

    // Check if the child link is within the attach distance
    if (dist > this->attachDistance)
    {
      gzwarn << "Child Link [" << this->childLinkName 
              << "] is too far from parent. Distance: " << dist 
              << "m, threshold: " << this->attachDistance << "m" << std::endl;
      this->attachRequested = false; // reset attach request
      return;
    }
    // If the child link is within the attach distance, proceed to attach
    gzdbg << "Child Link " << this->childLinkName
          << " is within attach distance of Parent Link. Proceeding to attach." << std::endl;

    // Attach the models
    // We do this by creating a detachable joint entity.
    this->detachableJointEntity = _ecm.CreateEntity();

    // creating the joint
    _ecm.CreateComponent(
        this->detachableJointEntity,
        components::DetachableJoint({this->parentLinkEntity,
                                      this->childLinkEntity, "fixed"}));
    this->attachRequested = false;
    this->isAttached = true;
    // Keep track of the attached pair for future validation
    this->attachedChildModelName = this->childModelName;
    this->attachedChildLinkName  = this->childLinkName;
    this->PublishJointState(this->isAttached);
    gzdbg << "Attaching entity: " << this->detachableJointEntity
          << std::endl;
  }

  // only allow detaching if child entity is attached
  if (this->isAttached)
  {
    if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
    {
      // Detach the models
      gzdbg << "Removing entity: " << this->detachableJointEntity << std::endl;
      _ecm.RequestRemoveEntity(this->detachableJointEntity);
      this->detachableJointEntity = kNullEntity;
      this->detachRequested = false;
      this->isAttached = false;
      // Publish using the last known attached pair, then clear them.
      this->PublishJointState(this->isAttached);
      this->attachedChildModelName.clear();
      this->attachedChildLinkName.clear();
    }
  }
}

//////////////////////////////////////////////////
bool DynamicDetachableJoint::OnServiceRequest(const gz::msgs::AttachDetachRequest &_req,
                                              gz::msgs::AttachDetachResponse &_res)
{
  GZ_PROFILE("DynamicDetachableJoint::OnServiceRequest");

  // Check if the request is valid
  if (_req.child_model_name().empty() || _req.child_link_name().empty() )
  {
    _res.set_success(false);
    _res.set_message("Invalid request: child_model_name and child_link_name must be set.");
    return true;
  }

  if (_req.command().empty())
  {
    _res.set_success(false);
    _res.set_message("Invalid request: command must be 'attach' or 'detach'.");
    return true;
  }

   // If attach is requested
   if (_req.command() == "attach")
   {
     if (this->isAttached) 
     {
       _res.set_success(false);
       _res.set_message("Already attached to child model " + this->attachedChildModelName +
                        " at link " + this->attachedChildLinkName + ".");
       gzdbg << "Already attached" << std::endl;
       return true;
     }

    // set the child model and link names from the request
    this->childModelName = _req.child_model_name();
    this->childLinkName  = _req.child_link_name();
    this->attachRequested = true;
    _res.set_success(true);
    _res.set_message("Attached to child model " + this->childModelName +
                     " at link " + this->childLinkName + ".");
   }

   // If detach is requested
   else if (_req.command() == "detach")
   {
     if (!this->isAttached)
     {
         _res.set_success(false);
         _res.set_message(std::string("Detach request received for ")
              + this->attachedChildModelName + "/" + this->attachedChildLinkName);
         gzdbg << "Already detached" << std::endl;
         return true;
     }

    // Validate that the request matches what is actually attached.
    const auto &reqModel = _req.child_model_name();
    const auto &reqLink  = _req.child_link_name();
    if (reqModel != this->attachedChildModelName ||
        reqLink  != this->attachedChildLinkName)
    {
      _res.set_success(false);
      _res.set_message(
        "Detach rejected: requested " + reqModel + "/" + reqLink +
        " but currently attached to " + this->attachedChildModelName + "/" +
        this->attachedChildLinkName + "."
      );
      return true;
    }

     this->detachRequested = true;
     _res.set_success(true);
     _res.set_message("Detached from child model " + this->attachedChildModelName +
                      " at link " + this->attachedChildLinkName + ".");
   }

   else
   {
     _res.set_success(false);
     _res.set_message("Invalid command. Use 'attach' or 'detach'.");
     return true;
   }
   return true;
 }

//////////////////////////////////////////////////
void DynamicDetachableJoint::PublishJointState(bool attached)
{
  gz::msgs::Entity stateMsg;
  if (attached)
  {
    stateMsg.set_id(this->childLinkEntity);
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
                    DynamicDetachableJoint::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DynamicDetachableJoint,
  "gz::sim::systems::DynamicDetachableJoint")
