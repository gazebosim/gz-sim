/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ShaderParams.hh>
#include <ignition/rendering/Visual.hh>

#include <sdf/Element.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/gazebo/rendering/RenderUtil.hh>


using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ShaderParamPrivate
{
  public: class ShaderParamValue
  {
    public: std::string type;
    public: std::string name;
    public: std::string value;
  };

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

  public: std::string visualName;

  public: rendering::VisualPtr visual;
  public: rendering::MaterialPtr material;
  public: rendering::ScenePtr scene;
  public: Entity entity = kNullEntity;
  public: std::vector<ShaderParamValue> shaderParams;

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
        ShaderParamPrivate::ShaderParamValue spv;
        spv.type = shaderType;
        spv.name = paramName;
        spv.value = value;
        this->dataPtr->shaderParams.push_back(spv);
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
      this->dataPtr->vertexShaderUri = vertexElem->Get<std::string>();
      sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
      this->dataPtr->fragmentShaderUri = fragmentElem->Get<std::string>();
    }
  }

  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

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
  if (this->visualName.empty())
    return;

  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  if (!this->visual)
  {
    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUti stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H?
        auto variant = n->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);
        if (value && *value == static_cast<int>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  if (!this->material)
  {
    auto mat = scene->CreateMaterial();
    mat->SetVertexShader(this->vertexShaderUri);
    mat->SetFragmentShader(this->fragmentShaderUri);
    this->visual->SetMaterial(mat);
    scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
  }

  if (!this->material)
    return;

  for (const auto & spv : this->shaderParams)
  {
    std::vector<std::string> values = common::split(spv.value, " ");
    if (values.empty())
      continue;

    int intValue = 0;
    float floatValue = 0;
    std::vector<float> floatArrayValue;

    rendering::ShaderParam::ParamType paramType;

    if (values.size() == 1u)
    {
      std::string str = values[0];
      std::string::size_type sz;
      int n = std::stoi(str, &sz);
      if (sz == str.size())
      {
        intValue = n;
        paramType = rendering::ShaderParam::PARAM_INT;
      }
      else
      {
        floatValue = std::stof(str);
        paramType = rendering::ShaderParam::PARAM_FLOAT;
      }
    }
    else
    {
      for (const auto &v : values)
        floatArrayValue.push_back(std::stof(v));
      paramType = rendering::ShaderParam::PARAM_FLOAT_BUFFER;
    }

    rendering::ShaderParamsPtr params;
    if (spv.type == "fragment")
    {
      params = this->material->FragmentShaderParams();
    }
    else if (spv.type == "vertex")
    {
      params = this->material->VertexShaderParams();
    }

    if (paramType == rendering::ShaderParam::PARAM_INT)
    {
//      (*params)[spv.name].InitializeBuffer(1);
      (*params)[spv.name] = intValue;
    }
    else if (paramType == rendering::ShaderParam::PARAM_FLOAT)
    {
//      (*params)[spv.name].InitializeBuffer(1);
      (*params)[spv.name] = floatValue;
    }
    else if (paramType == rendering::ShaderParam::PARAM_FLOAT_BUFFER)
    {
      (*params)[spv.name].InitializeBuffer(floatArrayValue.size());
      float *fv = &floatArrayValue[0];
      (*params)[spv.name].UpdateBuffer(fv);
    }
  }

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
