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

#include <mutex>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>

#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>


using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ShaderParamPrivate
{
  /// \brief Path to vertex shader
  public: std::string vertexShaderUri;

  /// \brief Path to fragment shader
  public: std::string fragmentShaderUri;

  /// \brief A list of params that requires setting sim time.
  /// Each element is a pair of <paramName, shaderType>
  public: std::vector<std::pair<std::string, std::string>> simTimeParams;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: ignition::common::ConnectionPtr connection{nullptr};

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();
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
  IGN_PROFILE("ShaderParam::Configure");
  std::cerr << "shader param configure" << std::endl;

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());


  if (sdf->HasElement("param"))
  {
    // loop and set all shader params
    sdf::ElementPtr paramElem = sdf->GetElement("param");
    while (paramElem)
    {
      if (!paramElem->HasElement("type") ||
          !paramElem->HasElement("name") ||
          !paramElem->HasElement("value"))
      {
        ignerr << "<param> must have <type>, <name> and <value> sdf elements"
               << std::endl;
        paramElem = paramElem->GetNextElement("param");
        continue;
      }
      std::string shaderType = paramElem->Get<std::string>("type");
      std::string paramName = paramElem->Get<std::string>("name");
      std::string value = paramElem->Get<std::string>("value");

      // TIME is reserved keyword for sim time
      if (value == "TIME")
      {
        this->dataPtr->simTimeParams.push_back(
            std::make_pair(paramName, shaderType));
      }
      else
      {
        // todo store value and set in rendering thread?
//        this->dataPtr->visual->SetMaterialShaderParam(
//            paramName, shaderType, value);
      }
      paramElem = paramElem->GetNextElement("param");
    }
  }

  if (sdf->HasElement("shader"))
  {
    sdf::ElementPtr shaderElem = sdf->GetElement("shader");
    if (!shaderElem->HasElement("vertex") ||
        !shaderElem->HasElement("fragment"))
    {
      ignerr << "<shader> must have <vertex> and <fragment> sdf elements"
             << std::endl;
    }
    else
    {
      sdf::ElementPtr vertexElem = shaderElem->GetElement("vertex");
      this->dataPtr->fragmentShaderUri = vertexElem->Get<std::string>();
      sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
      this->dataPtr->vertexShaderUri = fragmentElem->Get<std::string>();
    }
  }

  auto &eventMgr = RenderUtil::RenderEventManager();
  this->dataPtr->connection =
      eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
      std::bind(&ShaderParamPrivate::OnUpdate, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void ShaderParam::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ShaderParam::PreUpdate");
//  std::cerr << "shader param preupdate " << std::endl;
}

//////////////////////////////////////////////////
void ShaderParam::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ShaderParam::Update");
//  std::cerr << "shader param update" << std::endl;

//  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
}

//////////////////////////////////////////////////
void ShaderParam::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &/*_ecm*/)
{
  IGN_PROFILE("ShaderParam::PostUpdate");
//  std::cerr << "shader param post" << std::endl;
}

//////////////////////////////////////////////////
void ShaderParamPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::cerr << " on update ======== " << this << std::endl;
}

IGNITION_ADD_PLUGIN(ShaderParam,
                    ignition::gazebo::System,
                    ShaderParam::ISystemConfigure,
                    ShaderParam::ISystemPreUpdate,
                    ShaderParam::ISystemUpdate,
                    ShaderParam::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ShaderParam,
  "ignition::gazebo::systems::ShaderParam")
