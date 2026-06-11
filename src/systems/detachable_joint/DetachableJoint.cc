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

#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Profiler.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "DetachableJoint.hh"

using namespace gz;
using namespace gz::sim;
using namespace systems;

/////////////////////////////////////////////////
void DetachableJoint::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "DetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("parent_link"))
  {
    auto parentLinkName = _sdf->Get<std::string>("parent_link");
    this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);
    if (kNullEntity == this->parentLinkEntity)
    {
      ignerr << "Link with name " << parentLinkName
             << " not found in model " << this->model.Name(_ecm)
             << ". Make sure the parameter 'parent_link' has the "
             << "correct value. Failed to initialize.\n";
      return;
    }
  }
  else
  {
    ignerr << "'parent_link' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    ignerr << "'child_model' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    ignerr << "'child_link' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  // Setup detach topic
  std::vector<std::string> detachTopics;
  if (_sdf->HasElement("detach_topic"))
  {
    detachTopics.push_back(_sdf->Get<std::string>("detach_topic"));
  }
  detachTopics.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/detach");

  if (_sdf->HasElement("topic"))
  {
    if (_sdf->HasElement("detach_topic"))
    {
      if (_sdf->Get<std::string>("topic") !=
          _sdf->Get<std::string>("detach_topic"))
      {
        ignerr << "<topic> and <detach_topic> tags have different contents. "
                  "Please verify the correct string and use <detach_topic>."
               << std::endl;
      }
      else
      {
        igndbg << "Ignoring <topic> tag and using <detach_topic> tag."
                << std::endl;
      }
    }
    else
    {
      detachTopics.insert(detachTopics.begin(),
                          _sdf->Get<std::string>("topic"));
    }
  }

  this->detachTopic = validTopic(detachTopics);
  if (this->detachTopic.empty())
  {
    ignerr << "No valid detach topics for DetachableJoint could be found.\n";
    return;
  }
  igndbg << "Detach topic is: " << this->detachTopic << std::endl;

  // Setup subscriber for detach topic
  this->node.Subscribe(
      this->detachTopic, &DetachableJoint::OnDetachRequest, this);

  igndbg << "DetachableJoint subscribing to messages on "
         << "[" << this->detachTopic << "]" << std::endl;

  // Setup attach topic
  std::vector<std::string> attachTopics;
  if (_sdf->HasElement("attach_topic"))
  {
    attachTopics.push_back(_sdf->Get<std::string>("attach_topic"));
  }
  attachTopics.push_back("/model/" + this->model.Name(_ecm) +
      "/detachable_joint/attach");
  this->attachTopic = validTopic(attachTopics);
  if (this->attachTopic.empty())
  {
    ignerr << "No valid attach topics for DetachableJoint could be found.\n";
    return;
  }
  igndbg << "Attach topic is: " << this->attachTopic << std::endl;

  // Setup subscriber for attach topic
  auto msgCb = std::function<void(const transport::ProtoMsg &)>(
      [this](const auto &)
      {
        if (this->isAttached){
          igndbg << "Already attached" << std::endl;
          return;
        }
        this->attachRequested = true;
      });

  if (!this->node.Subscribe(this->attachTopic, msgCb))
  {
    ignerr << "Subscriber could not be created for [attach] topic.\n";
    return;
  }

  // Setup output topic
  std::vector<std::string> outputTopics;
  if (_sdf->HasElement("output_topic"))
  {
    outputTopics.push_back(_sdf->Get<std::string>("output_topic"));
  }

  outputTopics.push_back("/model/" + this->childModelName +
      "/detachable_joint/state");

  this->outputTopic = validTopic(outputTopics);
  if (this->outputTopic.empty())
  {
    ignerr << "No valid output topics for DetachableJoint could be found.\n";
    return;
  }
  igndbg << "Output topic is: " << this->outputTopic << std::endl;

  // Setup publisher for output topic
  this->outputPub = this->node.Advertise<ignition::msgs::StringMsg>(
      this->outputTopic);
  if (!this->outputPub)
  {
    ignerr << "Error advertising topic [" << this->outputTopic << "]"
              << std::endl;
    return;
  }

  // Supress Child Warning
  this->suppressChildWarning =
      _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
          .first;

  this->validConfig = true;

  this->GetChildModelAndLinkEntities(_ecm);
}

//////////////////////////////////////////////////
void DetachableJoint::GetChildModelAndLinkEntities(
  EntityComponentManager &_ecm)
{
  this->childLinkEntity = kNullEntity;
  // Look for the child model and link
  Entity modelEntity{kNullEntity};

  if ("__model__" == this->childModelName)
  {
    modelEntity = this->model.Entity();
  }
  else
  {
    auto entitiesMatchingName = entitiesFromScopedName(
      this->childModelName, _ecm);

    // TODO(arjoc): There is probably a more efficient way of combining entitiesFromScopedName
    // With filtering.
    // Filter for entities with only models
    std::vector<Entity> candidateEntities;
    std::copy_if(entitiesMatchingName.begin(), entitiesMatchingName.end(),
                std::back_inserter(candidateEntities),
                [&_ecm](Entity e) { return _ecm.EntityHasComponentType(e, components::Model::typeId); });

    if (candidateEntities.size() == 1)
    {
      // If there is one entity select that entity itself
      modelEntity = *candidateEntities.begin();
    }
    else
    {
      std::string selectedModelName;
      auto parentEntityScopedPath = scopedName(this->model.Entity(), _ecm);
      // If there is more than one model with the given child model name, the plugin looks for a model which is
      // - a descendant of the plugin's parent model with that name, and
      // - has a child link with the given child link name
      for (auto entity : candidateEntities)
      {
        auto childEntityScope = scopedName(entity, _ecm);
        if (childEntityScope.size() < parentEntityScopedPath.size())
        {
          continue;
        }
        if (childEntityScope.rfind(parentEntityScopedPath, 0) != 0)
        {
          continue;
        }
        if (modelEntity == kNullEntity)
        {

          this->childLinkEntity = _ecm.EntityByComponents(
                    components::Link(), components::ParentEntity(entity),
                    components::Name(this->childLinkName));

          if (kNullEntity != this->childLinkEntity)
          {
                // Only select this child model entity if the entity has a link with the given child link name
                modelEntity = entity;
                selectedModelName = childEntityScope;
                igndbg << "Selecting " << childEntityScope << " as model to be detached" << std::endl;
          }
          else
          {
            ignwarn << "Found " << childEntityScope << " with no link named " << this->childLinkName << std::endl;
          }
        }
        else
        {
          ignwarn << "Found multiple models skipping " << childEntityScope
            << "Using " << selectedModelName << " instead" << std::endl;
        }
      }
    }
  }
  if (kNullEntity != modelEntity)
  {
    this->childLinkEntity = _ecm.EntityByComponents(
        components::Link(), components::ParentEntity(modelEntity),
        components::Name(this->childLinkName));
  }
  else if (!this->suppressChildWarning)
  {
    ignwarn << "Child Model " << this->childModelName
            << " could not be found.\n";
  }
}
//////////////////////////////////////////////////
void DetachableJoint::PreUpdate(
  const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  IGN_PROFILE("DetachableJoint::PreUpdate");
  // only allow attaching if child entity is detached
  if (this->validConfig && !this->isAttached)
  {
    // return if attach is not requested.
    if (!this->attachRequested){
      return;
    }

    if (this->childLinkEntity == kNullEntity || !_ecm.HasEntity(this->childLinkEntity))
      this->GetChildModelAndLinkEntities(_ecm);

    if (kNullEntity != this->childLinkEntity)
    {
      // Attach the models
      // We do this by creating a detachable joint entity.
      this->detachableJointEntity = _ecm.CreateEntity();

      _ecm.CreateComponent(
          this->detachableJointEntity,
          components::DetachableJoint({this->parentLinkEntity,
                                        this->childLinkEntity, "fixed"}));
      this->attachRequested = false;
      this->isAttached = true;
      this->PublishJointState(this->isAttached);
      igndbg << "Attaching entity: " << this->detachableJointEntity
              << std::endl;
    }
    else if (!this->suppressChildWarning)
    {
      ignwarn << "Child Link " << this->childLinkName
              << " could not be found.\n";
    }

  }

  // only allow detaching if child entity is attached
  if (this->isAttached)
  {
    if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
    {
      // Detach the models
      igndbg << "Removing entity: " << this->detachableJointEntity << std::endl;
      _ecm.RequestRemoveEntity(this->detachableJointEntity);
      this->detachableJointEntity = kNullEntity;
      this->detachRequested = false;
      this->isAttached = false;
      this->PublishJointState(this->isAttached);
    }
  }
}

//////////////////////////////////////////////////
void DetachableJoint::PublishJointState(bool attached)
{
  ignition::msgs::StringMsg detachedStateMsg;
  if (attached)
  {
    detachedStateMsg.set_data("attached");
  }
  else
  {
    detachedStateMsg.set_data("detached");
  }
  this->outputPub.Publish(detachedStateMsg);
}

//////////////////////////////////////////////////
void DetachableJoint::OnDetachRequest(const msgs::Empty &)
{
  if (!this->isAttached){
    igndbg << "Already detached" << std::endl;
    return;
  }
  this->detachRequested = true;
}

IGNITION_ADD_PLUGIN(DetachableJoint,
                    System,
                    DetachableJoint::ISystemConfigure,
                    DetachableJoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DetachableJoint,
  "gz::sim::systems::DetachableJoint")

// TODO(CH3): Deprecated, remove on version 8
IGNITION_ADD_PLUGIN_ALIAS(DetachableJoint,
  "ignition::gazebo::systems::DetachableJoint")
