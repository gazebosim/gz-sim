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

#include <optional>
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
using namespace sim;
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
    gzerr << "DetachableJoint should be attached to a model entity. "
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
    gzerr << "'parent_link' is a required parameter for DetachableJoint. "
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    gzerr << "'child_model' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  else
  {
    gzerr << "'child_link' is a required parameter for DetachableJoint."
              "Failed to initialize.\n";
    return;
  }

  // Optional physics parameters
  if (_sdf->HasElement("physics"))
  {
    // Clone as GetElement() requires non-const access.
    auto sdfClone = _sdf->Clone();
    auto physicsSdf = sdfClone->GetElement("physics");
    if (physicsSdf->HasElement("cfm"))
    {
      this->cfm = std::optional<double>(physicsSdf->Get<double>("cfm"));
    }
    if (physicsSdf->HasElement("erp"))
    {
      this->erp = std::optional<double>(physicsSdf->Get<double>("erp"));
    }
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
        gzerr << "<topic> and <detach_topic> tags have different contents. "
                 "Please verify the correct string and use <detach_topic>."
              << std::endl;
      }
      else
      {
        gzdbg << "Ignoring <topic> tag and using <detach_topic> tag."
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
    gzerr << "No valid detach topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Detach topic is: " << this->detachTopic << std::endl;

  // Setup subscriber for detach topic
  this->node.Subscribe(
      this->detachTopic, &DetachableJoint::OnDetachRequest, this);

  gzdbg << "DetachableJoint subscribing to messages on "
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
    gzerr << "No valid attach topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Attach topic is: " << this->attachTopic << std::endl;

  // Setup subscriber for attach topic
  auto msgCb = std::function<void(const transport::ProtoMsg &)>(
      [this](const auto &)
      {
        if (this->isAttached){
          gzdbg << "Already attached" << std::endl;
          return;
        }
        this->attachRequested = true;
      });

  if (!this->node.Subscribe(this->attachTopic, msgCb))
  {
    gzerr << "Subscriber could not be created for [attach] topic.\n";
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
    gzerr << "No valid output topics for DetachableJoint could be found.\n";
    return;
  }
  gzdbg << "Output topic is: " << this->outputTopic << std::endl;

  // Setup publisher for output topic
  this->outputPub = this->node.Advertise<gz::msgs::StringMsg>(
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
void DetachableJoint::PreUpdate(
  const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("DetachableJoint::PreUpdate");
  // only allow attaching if child entity is detached
  if (this->validConfig && !this->isAttached)
  {
    // return if attach is not requested.
    if (!this->attachRequested){
      return;
    }
    // Look for the child model and link
    Entity modelEntity{kNullEntity};

    if ("__model__" == this->childModelName)
    {
      modelEntity = this->model.Entity();
    }
    else
    {
      modelEntity = _ecm.EntityByComponents(
          components::Model(), components::Name(this->childModelName));
    }
    if (kNullEntity != modelEntity)
    {
      this->childLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(modelEntity),
          components::Name(this->childLinkName));

      if (kNullEntity != this->childLinkEntity)
      {
        // Attach the models
        // We do this by creating a detachable joint entity.
        this->detachableJointEntity = _ecm.CreateEntity();

        _ecm.CreateComponent(
            this->detachableJointEntity,
            components::DetachableJoint({
              this->parentLinkEntity,
              this->childLinkEntity,
              "fixed",
              this->cfm,
              this->erp
            }));
        this->attachRequested = false;
        this->isAttached = true;
        this->PublishJointState(this->isAttached);
        gzdbg << "Attaching entity: " << this->detachableJointEntity
               << std::endl;

        //! @todo  create a separate component for the dynamic joint
        ///        constraints?
      }
      else
      {
        gzwarn << "Child Link " << this->childLinkName
                << " could not be found.\n";
      }
    }
    else if (!this->suppressChildWarning)
    {
      gzwarn << "Child Model " << this->childModelName
              << " could not be found.\n";
    }
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
      this->PublishJointState(this->isAttached);
    }
  }
}

//////////////////////////////////////////////////
void DetachableJoint::PublishJointState(bool attached)
{
  msgs::StringMsg detachedStateMsg;
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
    gzdbg << "Already detached" << std::endl;
    return;
  }
  this->detachRequested = true;
}

GZ_ADD_PLUGIN(DetachableJoint,
                    System,
                    DetachableJoint::ISystemConfigure,
                    DetachableJoint::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DetachableJoint,
  "gz::sim::systems::DetachableJoint")
