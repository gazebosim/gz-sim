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

#include <sstream>
#include <unordered_map>

#include "Parameters.hh"

#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/EntityComponentManager.hh"
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParameterDeclarationCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/msgs/parameter_declarations.pb.h>
#include <ignition/msgs/parameter_name.pb.h>
#include <ignition/msgs/parameter_value.pb.h>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Data we need to store for each parameter.
struct ParameterData
{
  /// \brief Associated component key.
  ComponentKey componentKey;
};

/// \brief Registry type to store all parameters.
using ParameterMap = std::unordered_map<std::string, ParameterData>;

class ignition::gazebo::systems::ParametersPrivate
{
  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Pointer to entity component manager.
  public: EntityComponentManager * ecm{nullptr};

  /// \brief World entity.
  public: Entity worldEntity{kNullEntity};

  /// \brief Parameter registry.
  public: ParameterMap registry;

  /// \brief Callback for create service
  /// \param[in] _req Request containing entity description.
  /// \param[out] _res True if message successfully received and queued.
  /// It does not mean that the entity will be successfully spawned.
  /// \return True if successful.
  public: bool GetParameter(const msgs::ParameterName &_req,
      msgs::ParameterValue &_res);
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

  // this->dataPtr->ecm->CreateComponent(
  //     _entity, components::ParametersRegistry(msgs::ParameterDeclarations()));

  const components::Name *constCmp = _ecm.Component<components::Name>(_entity);
  const std::string &worldName = constCmp->Data();
  auto validWorldName = transport::TopicUtils::AsValidTopic(worldName);
  if (validWorldName.empty())
  {
    ignerr << "World name [" << worldName
           << "] doesn't work well with transport, parameter services not advertised."
           << std::endl;
    return;
  }

  std::string srvNamePrefix = "/world/" + validWorldName;
  std::string getParameterSrvName{srvNamePrefix + "/get_parameter"};
  this->dataPtr->node.Advertise(getParameterSrvName,
    &ParametersPrivate::GetParameter, this->dataPtr.get());
}

//////////////////////////////////////////////////
void Parameters::PreUpdate(const UpdateInfo &, EntityComponentManager &)
{
  IGN_PROFILE("Parameters::PreUpdate");
  auto * declaration_cmd = this->dataPtr->ecm->Component<components::ParameterDeclarationCmd>(this->dataPtr->worldEntity);
  if (!declaration_cmd) {
    return;
  }
  auto & cmdData = declaration_cmd->Data();
  for (const auto & decl : cmdData.parameter_declarations())
  {
    ComponentKey key{decl.component_type_id(), decl.component_id()};
    this->dataPtr->registry.emplace(decl.name(), ParameterData{key});
  }
  cmdData.clear_parameter_declarations();
}

bool ParametersPrivate::GetParameter(const msgs::ParameterName &_req,
  msgs::ParameterValue &_res)
{
  const auto & param_name = _req.name();
  auto it = this->registry.find(param_name);
  if (it == this->registry.end()) {
    return false;
  }
  ComponentKey key{it->second.componentKey};
  auto component = this->ecm->Component<components::BaseComponent>(key);
  if (!component) {
    return false;
  }
  std::ostringstream oss;
  component->Serialize(oss);
  _res.set_value(oss.str());
  _res.set_type(components::Factory::Instance()->Name(key.first));
  return true;
}

IGNITION_ADD_PLUGIN(Parameters,
                    ignition::gazebo::System,
                    Parameters::ISystemConfigure,
                    Parameters::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Parameters,
                          "ignition::gazebo::systems::Parameters")
