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

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "DetachableJoint.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void DetachableJoint::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  // Store the pointer to the model this battery is under
  auto model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "DetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("parent_model_link"))
  {
    this->parentModelLinkName =
        _sdf->Get<std::string>("parent_model_link");
    this->parentModelLinkEntity =
        model.LinkByName(_ecm, this->parentModelLinkName);
    if (kNullEntity == this->parentModelLinkEntity)
    {
      ignerr << "Link with name " << this->parentModelLinkName
             << " not found in model " << model.Name(_ecm)
             << ". Make sure the parameter 'parent_model_link' has the "
             << "correct value." << std::endl;
      return;
    }
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  else
  {
    ignerr << "'child_model' is a required parameter for DetachableJoint."
           << std::endl;
    return;
  }

  if (_sdf->HasElement("child_model_link"))
  {
    this->childModelLinkName = _sdf->Get<std::string>("child_model_link");
  }
  else
  {
    ignerr << "'child_model_link' is a required parameter for DetachableJoint."
           << std::endl;
    return;
  }

  // Setup battery state topic
  std::string defaultTopic{"/model/" + model.Name(_ecm) +
                             "/detachable_joint/detach"};
  this->topic = _sdf->Get<std::string>("topic", defaultTopic).first;

  this->validConfig = true;
}

//////////////////////////////////////////////////
void DetachableJoint::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("DetachableJoint::PreUpdate");
  if (this->validConfig && !this->initialized)
  {
    // Look for the child model and link
    auto modelEntity = _ecm.EntityByComponents(components::Model(),
                            components::Name(this->childModelName));
    if (kNullEntity != modelEntity)
    {
      this->childModelLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(modelEntity),
          components::Name(this->childModelLinkName));

      if (kNullEntity != this->childModelLinkEntity)
      {
        // Attach the models
        // We do this by creating a detachable joint entity.
        this->detachableJointEntity = _ecm.CreateEntity();

        _ecm.CreateComponent(
            this->detachableJointEntity,
            components::DetachableJoint({this->parentModelLinkEntity,
                                         this->childModelLinkEntity, "fixed"}));

        this->node.Subscribe(
            this->topic, &DetachableJoint::OnDetachRequest, this);

        ignmsg << "DetachableJoint subscribing to messages on "
               << "[" << this->topic << "]" << std::endl;

        this->initialized = true;
      }
      else
      {
        ignwarn << "Child Link " << this->childModelLinkName
                << " could not be found.\n";
      }
    }
    else
    {
      ignwarn << "Child Model " << this->childModelName
              << " could not be found.\n";
    }
  }

  if (this->initialized)
  {
    if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
    {
      // Detach the models
      std::cout << "Removing entity: " << this->detachableJointEntity
                << std::endl;
      _ecm.RequestRemoveEntity(this->detachableJointEntity);
      this->detachableJointEntity = kNullEntity;
      this->detachRequested = false;
    }
  }
}

//////////////////////////////////////////////////
void DetachableJoint::OnDetachRequest(const msgs::Boolean &)
{
  this->detachRequested = true;
}

IGNITION_ADD_PLUGIN(DetachableJoint,
                    ignition::gazebo::System,
                    DetachableJoint::ISystemConfigure,
                    DetachableJoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(DetachableJoint,
  "ignition::gazebo::systems::DetachableJoint")
