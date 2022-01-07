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

#include <mutex>
#include <unordered_map>
#include <sstream>

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

  /// \brief Mutex protecting `this->registry`.
  public: std::mutex registryMutex;

  /// \brief Get parameter service callback.
  /// \param[in] _req Request specifying the parameter name.
  /// \param[out] _res The value of the parameter.
  /// \return True if successful.
  public: bool GetParameter(const msgs::ParameterName &_req,
      msgs::ParameterValue &_res);

  /// \brief List parameter service callback.
  /// \param[in] _req unused.
  /// \param[out] _res List of available parameters.
  /// \return True if successful.
  public: bool ListParameters(const msgs::Empty &_req,
    msgs::ParameterDeclarations &_res);

  /// \brief Set parameter service callback.
  /// \param[in] _req Request specifying which parameter to set and its value.
  /// \param[out] _res Unused.
  /// \return True if successful.
  public: bool SetParameter(const msgs::Parameter &_req,
    msgs::Empty &_res);
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

  std::string listParametersSrvName{srvNamePrefix + "/list_parameters"};
  this->dataPtr->node.Advertise(listParametersSrvName,
    &ParametersPrivate::ListParameters, this->dataPtr.get());

  std::string setParameterSrvName{srvNamePrefix + "/set_parameter"};
  this->dataPtr->node.Advertise(setParameterSrvName,
    &ParametersPrivate::SetParameter, this->dataPtr.get());
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
  {
    std::lock_guard guard{this->dataPtr->registryMutex};
    for (const auto & decl : cmdData.parameter_declarations())
    {
      ComponentKey key{decl.component_type_id(), decl.component_id()};
      this->dataPtr->registry.emplace(decl.name(), ParameterData{key});
    }
  }
  cmdData.clear_parameter_declarations();
}

bool ParametersPrivate::GetParameter(const msgs::ParameterName &_req,
  msgs::ParameterValue &_res)
{
  const auto & param_name = _req.name();
  ComponentKey key;
  {
    std::lock_guard guard{this->registryMutex};
    auto it = this->registry.find(param_name);
    if (it == this->registry.end()) {
      return false;
    }
    key = it->second.componentKey;
  }
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

bool ParametersPrivate::ListParameters(const msgs::Empty &,
  msgs::ParameterDeclarations &_res)
{
  // TODO(ivanpauno): Maybe the response should only include parameter names (?)
  // Maybe only names and types (?)
  // Including the component key doesn't seem to matter much, though it's also not wrong.
  {
    std::lock_guard guard{this->registryMutex};
    for (const auto & paramPair: this->registry) {
      auto * decl = _res.add_parameter_declarations();
      decl->set_name(paramPair.first);
      const auto & cmpKey = paramPair.second.componentKey;
      decl->set_type(components::Factory::Instance()->Name(cmpKey.first));
      decl->set_component_type_id(cmpKey.first);
      decl->set_component_id(cmpKey.second);
    }
  }
  return true;
}

bool ParametersPrivate::SetParameter(const msgs::Parameter &_req,
  msgs::Empty &)
{
  const auto & param_name = _req.name();
  ComponentKey key;
  {
    std::lock_guard guard{this->registryMutex};
    auto it = this->registry.find(param_name);
    if (it == this->registry.end()) {
      return false;
    }
    key = it->second.componentKey;
  }
  if (components::Factory::Instance()->Name(key.first) != _req.type()) {
    // parameter type doesn't match
    return false;
  }
  auto * component = this->ecm->Component<components::BaseComponent>(key);
  if (!component) {
    // component was removed
    // TODO(ivanpauno): Add a way to underclare a parameter
    return false;
  }
  std::istringstream iss{_req.value()};
  component->Deserialize(iss);
  return false;
}

IGNITION_ADD_PLUGIN(Parameters,
                    ignition::gazebo::System,
                    Parameters::ISystemConfigure,
                    Parameters::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Parameters,
                          "ignition::gazebo::systems::Parameters")
