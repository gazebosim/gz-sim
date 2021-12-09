/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "Parameters.hh"

#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/EntityComponentManager.hh"
#include <ignition/gazebo/components/ParametersRegistry.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ParametersPrivate
{
  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Pointer to entity component manager.
  public: EntityComponentManager * ecm{nullptr};

  /// \brief World entity.
  public: Entity worldEntity{kNullEntity};
};

//////////////////////////////////////////////////
Parameters::Parameters()
  : dataPtr(std::make_unique<ParametersPrivate>())
{
}

//////////////////////////////////////////////////
void Parameters::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->worldEntity = _entity;
  this->dataPtr->ecm = &_ecm;
  // this->dataPtr->node.Advertise(createService,
  //     &UserCommandsPrivate::CreateService, this->dataPtr.get());
  this->dataPtr->ecm->CreateComponent(
      _entity, components::ParametersRegistry(msgs::ParameterDeclarations()));
}

//////////////////////////////////////////////////
void Parameters::PreUpdate(const UpdateInfo &_info, EntityComponentManager &)
{
  IGN_PROFILE("Parameters::PreUpdate");
}

IGNITION_ADD_PLUGIN(Parameters,
                    ignition::gazebo::System,
                    Parameters::ISystemConfigure,
                    Parameters::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Parameters,
                          "ignition::gazebo::systems::Parameters")
