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

#include <chrono>
#include <list>
#include <map>
#include <mutex>
#include <vector>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/rendering/Visual.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ShaderParamPrivate
{
  /// \brief Data structure for storing shader param info
  public: class ShaderParamValue
  {
    /// \brief shader type: vertex or fragment
    public: std::string shader;

    /// \brief variable type: int, float, float_array, int_array,
    /// texture, texture_cube
    public: std::string type;

    /// \brief variable name of param
    public: std::string name;

    /// \brief param value
    public: std::string value;

    /// \brief Any additional arguments
    public: std::vector<std::string> args;
  };

  /// \brief Data structure for storing shader files uri
  public: class ShaderUri
  {
    /// \brief Shader language: glsl or metal
    public: std::string language;

    /// \brief Path to vertex shader
    public: std::string vertexShaderUri;

    /// \brief Path to fragment shader
    public: std::string fragmentShaderUri;
  };

  /// \brief A map of shader language to shader program files
  public: std::map<std::string, ShaderUri> shaders;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: gz::common::ConnectionPtr connection{nullptr};

  /// \brief Connection to render tear down event callback
  public: gz::common::ConnectionPtr teardownConnection{nullptr};

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Material used by this visual
  public: rendering::MaterialPtr material;

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Entity id of the visual
  public: Entity entity = kNullEntity;

  /// \brief A list of shader params
  public: std::vector<ShaderParamValue> shaderParams;

  /// \brief Time params that will be updated every iteration
  public: std::vector<ShaderParamValue> timeParams;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /// \brief Path to model
  public: std::string modelPath;

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /// \brief Callback to rendering engine tear down
  public: void OnRenderTeardown();
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
               EventManager &_eventMgr)
{
  GZ_PROFILE("ShaderParam::Configure");

  if (_sdf->HasElement("param"))
  {
    // loop and parse all shader params
    auto paramElem = _sdf->FindElement("param");
    while (paramElem)
    {
      if (!paramElem->HasElement("shader") ||
          !paramElem->HasElement("name"))
      {
        gzerr << "<param> must have <shader> and <name> sdf elements"
               << std::endl;
        paramElem = paramElem->GetNextElement("param");
        continue;
      }
      std::string shaderType = paramElem->Get<std::string>("shader");
      std::string paramName = paramElem->Get<std::string>("name");

      std::string type = paramElem->Get<std::string>("type", "float").first;
      std::string value = paramElem->Get<std::string>("value", "").first;

      ShaderParamPrivate::ShaderParamValue spv;
      spv.shader = shaderType;
      spv.name = paramName;
      spv.value = value;
      spv.type = type;

      if (paramElem->HasElement("arg"))
      {
        auto argElem = paramElem->FindElement("arg");
        while (argElem)
        {
          spv.args.push_back(argElem->Get<std::string>());
          argElem = argElem->GetNextElement("arg");
        }
      }

      this->dataPtr->shaderParams.push_back(spv);
      paramElem = paramElem->GetNextElement("param");
    }
  }

  if (this->dataPtr->modelPath.empty())
  {
    auto modelEntity = topLevelModel(_entity, _ecm);
    this->dataPtr->modelPath =
        _ecm.ComponentData<components::SourceFilePath>(modelEntity).value();
  }

  // parse path to shaders
  if (!_sdf->HasElement("shader"))
  {
    gzerr << "Unable to load shader param system. "
           << "Missing <shader> SDF element." << std::endl;
    return;
  }
  // allow mulitple shader SDF element for different shader languages
  auto shaderElem = _sdf->FindElement("shader");
  while (shaderElem)
  {
    if (!shaderElem->HasElement("vertex") ||
        !shaderElem->HasElement("fragment"))
    {
      gzerr << "<shader> must have <vertex> and <fragment> sdf elements"
             << std::endl;
    }
    else
    {
      // default to glsl
      std::string api = "glsl";
      if (shaderElem->HasAttribute("language"))
        api = shaderElem->GetAttribute("language")->GetAsString();

      ShaderParamPrivate::ShaderUri shader;
      shader.language = api;

      auto vertexElem = shaderElem->FindElement("vertex");
      shader.vertexShaderUri = common::findFile(
          asFullPath(vertexElem->Get<std::string>(),
          this->dataPtr->modelPath));
      auto fragmentElem = shaderElem->FindElement("fragment");
      shader.fragmentShaderUri = common::findFile(
          asFullPath(fragmentElem->Get<std::string>(),
          this->dataPtr->modelPath));
      this->dataPtr->shaders[api] = shader;
      shaderElem = shaderElem->GetNextElement("shader");
    }
  }
  if (this->dataPtr->shaders.empty())
  {
    gzerr << "Unable to load shader param system. "
           << "No valid shaders." << std::endl;
    return;
  }

  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<gz::sim::events::SceneUpdate>(
      std::bind(&ShaderParamPrivate::OnUpdate, this->dataPtr.get()));

  this->dataPtr->teardownConnection =
      _eventMgr.Connect<gz::sim::events::RenderTeardown>(
      std::bind(&ShaderParamPrivate::OnRenderTeardown, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void ShaderParam::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  GZ_PROFILE("ShaderParam::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
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
        auto variant = n->UserData("gazebo-entity");
        const uint64_t *value = std::get_if<uint64_t>(&variant);
        if (value && *value == this->entity)
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

  // get the material and set shaders
  if (!this->material)
  {
    auto mat = scene->CreateMaterial();

    if (scene->Engine() && scene->Engine()->GraphicsAPI() ==
        rendering::GraphicsAPI::METAL)
    {
      // metal
      auto metalIt = this->shaders.find("metal");
      if (metalIt != this->shaders.end())
      {
        mat->SetVertexShader(metalIt->second.vertexShaderUri);
        mat->SetFragmentShader(metalIt->second.fragmentShaderUri);
        gzmsg << "Using metal shaders. " << std::endl;
      }
    }
    else
    {
      //  try glsl for all others
      auto it = this->shaders.find("glsl");
      if (it != this->shaders.end())
      {
        mat->SetVertexShader(it->second.vertexShaderUri);
        mat->SetFragmentShader(it->second.fragmentShaderUri);
        gzmsg << "Using glsl shaders. " << std::endl;
      }
    }

    // inherit cast shadows property from existing material
    rendering::MaterialPtr oldMat;
    if (visual->GeometryCount() >  0u)
      oldMat = this->visual->GeometryByIndex(0u)->Material();
    else
      oldMat = this->visual->Material();
    if (oldMat)
      mat->SetCastShadows(oldMat->CastShadows());

    this->visual->SetMaterial(mat);
    scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
  }

  if (!this->material)
    return;

  // set the shader params read from SDF
  // this is only done once
  for (const auto & spv : this->shaderParams)
  {
    // TIME is reserved keyword for sim time
    if (spv.value == "TIME")
    {
      this->timeParams.push_back(spv);
      continue;
    }

    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
    {
      params = this->material->FragmentShaderParams();
    }
    else if (spv.shader == "vertex")
    {
      params = this->material->VertexShaderParams();
    }

    // if no <value> is specified, this could be a constant
    if (spv.value.empty())
    {
      // \todo handle args for constants in gz-rendering
      (*params)[spv.name] = 1;
      continue;
    }

    // handle texture params
    if (spv.type == "texture")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE, uvSetIndex);
    }
    else if (spv.type == "texture_cube")
    {
      unsigned int uvSetIndex = spv.args.empty() ? 0u :
          static_cast<unsigned int>(std::stoul(spv.args[0]));
      std::string texPath = common::findFile(
          asFullPath(spv.value, this->modelPath));
      (*params)[spv.name].SetTexture(texPath,
          rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, uvSetIndex);
    }
    // handle int, float, int_array, and float_array params
    else
    {
      std::vector<std::string> values = common::split(spv.value, " ");

      int intValue = 0;
      float floatValue = 0;
      std::vector<float> floatArrayValue;

      rendering::ShaderParam::ParamType paramType =
          rendering::ShaderParam::PARAM_NONE;

      // float / int
      if (values.size() == 1u)
      {
        std::string str = values[0];

        // if <type> is not empty, respect the specified type
        if (!spv.type.empty())
        {
          if (spv.type == "int")
          {
            intValue = std::stoi(str);
            paramType = rendering::ShaderParam::PARAM_INT;
          }
          else if (spv.type == "float")
          {
            floatValue = std::stof(str);
            paramType = rendering::ShaderParam::PARAM_FLOAT;
          }
        }
        // else do our best guess at what the type is
        else
        {
          std::string::size_type sz;
          int n = std::stoi(str, &sz);
          if ( sz == str.size())
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
      }
      // arrays
      else
      {
        // int array
        if (!spv.type.empty() && spv.type == "int_array")
        {
          for (const auto &v : values)
            floatArrayValue.push_back(std::stof(v));
          paramType = rendering::ShaderParam::PARAM_INT_BUFFER;
        }
        // treat everything else as float_array
        else
        {
          for (const auto &v : values)
            floatArrayValue.push_back(std::stof(v));
          paramType = rendering::ShaderParam::PARAM_FLOAT_BUFFER;
        }
      }

      // set the params
      if (paramType == rendering::ShaderParam::PARAM_INT)
      {
        (*params)[spv.name] = intValue;
      }
      else if (paramType == rendering::ShaderParam::PARAM_FLOAT)
      {
        (*params)[spv.name] = floatValue;
      }
      else if (paramType == rendering::ShaderParam::PARAM_INT_BUFFER ||
          paramType == rendering::ShaderParam::PARAM_FLOAT_BUFFER)
      {
        (*params)[spv.name].InitializeBuffer(floatArrayValue.size());
        float *fv = &floatArrayValue[0];
        (*params)[spv.name].UpdateBuffer(fv);
      }
    }
  }
  this->shaderParams.clear();

  // time variables need to be updated every iteration
  for (const auto & spv : this->timeParams)
  {
    float floatValue = (std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->currentSimTime).count()) * 1e-9;
    rendering::ShaderParamsPtr params;
    if (spv.shader == "fragment")
      params = this->material->FragmentShaderParams();
    else if (spv.shader == "vertex")
      params = this->material->VertexShaderParams();
    (*params)[spv.name] = floatValue;
  }
}

//////////////////////////////////////////////////
void ShaderParamPrivate::OnRenderTeardown()
{
  this->visual.reset();
  this->scene.reset();
  this->material.reset();
}

GZ_ADD_PLUGIN(ShaderParam,
                    gz::sim::System,
                    ShaderParam::ISystemConfigure,
                    ShaderParam::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ShaderParam,
  "gz::sim::systems::ShaderParam")
