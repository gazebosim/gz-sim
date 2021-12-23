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

#include "ShaderParam.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>


using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ShaderParamPrivate
{
};

/////////////////////////////////////////////////
ShaderParam::ShaderParam()
    : System(), dataPtr(std::make_unique<ShaderParamPrivate>())
{
}

/////////////////////////////////////////////////
ShaderParam::~ShaderParam()
{
}

/////////////////////////////////////////////////
void ShaderParam::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  std::cerr << "=== shader param configure" << std::endl;
}

//////////////////////////////////////////////////
void ShaderParam::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ShaderParam::PreUpdate");
  std::cerr << "shader param preupdate " << std::endl;
}

//////////////////////////////////////////////////
void ShaderParam::Update(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm)
{
  IGN_PROFILE("ShaderParam::Update");
  std::cerr << "shader param update" << std::endl;
}

//////////////////////////////////////////////////
void ShaderParam::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("ShaderParam::PostUpdate");
  std::cerr << "shader param post" << std::endl;
}

IGNITION_ADD_PLUGIN(ShaderParam,
                    ignition::gazebo::System,
                    ShaderParam::ISystemConfigure,
                    ShaderParam::ISystemPreUpdate,
                    ShaderParam::ISystemUpdate,
                    ShaderParam::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ShaderParam,
  "ignition::gazebo::systems::ShaderParam")
