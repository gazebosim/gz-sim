/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "SceneManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data class.
class ignition::gazebo::systems::SceneManagerPrivate
{
  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Map of visual id to visual pointers.
  public: std::map<unsigned int, rendering::VisualPtr> visuals;

  /// \brief Map of light id to light pointers.
  public: std::map<unsigned int, rendering::LightPtr> lights;

  /// \brief Map of sensor id to sensors
  public: std::map<unsigned int, rendering::SensorPtr> sensors;
};


/////////////////////////////////////////////////
SceneManager::SceneManager()
  : dataPtr(std::make_unique<SceneManagerPrivate>())
{
}

/////////////////////////////////////////////////
SceneManager::~SceneManager()
{
}

/////////////////////////////////////////////////
void SceneManager::SetScene(rendering::ScenePtr _scene)
{
  this->dataPtr->scene = _scene;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateModel(int _id,
    const sdf::Model &_model, int _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId > 0)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding entity: [" << _id << "]" << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  std::string name = _model.Name().empty() ? std::to_string(_id) :
      _model.Name();
  if (parent)
    name = parent->Name() +  "::" + name;
  rendering::VisualPtr modelVis = this->dataPtr->scene->CreateVisual(name);
  modelVis->SetLocalPose(_model.Pose());
  this->dataPtr->visuals[_id] = modelVis;

  if (parent)
    parent->AddChild(modelVis);
  else
    this->dataPtr->scene->RootVisual()->AddChild(modelVis);

  return modelVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateLink(int _id, const sdf::Link &_link,
    int _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId > 0)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding entity: [" << _id << "]" << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  std::string name = _link.Name().empty() ? std::to_string(_id) :
      _link.Name();
  if (parent)
    name = parent->Name() + "::" + name;
  rendering::VisualPtr linkVis = this->dataPtr->scene->CreateVisual(name);
  linkVis->SetLocalPose(_link.Pose());
  this->dataPtr->visuals[_id] = linkVis;

  if (parent)
    parent->AddChild(linkVis);

  return linkVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateVisual(int _id,
    const sdf::Visual &_visual, int _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId > 0)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding entity: [" << _id << "]" << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  if (!_visual.Geom())
    return rendering::VisualPtr();

  std::string name = _visual.Name().empty() ? std::to_string(_id) :
      _visual.Name();
  if (parent)
    name = parent->Name() + "::" + name;
  rendering::VisualPtr visualVis = this->dataPtr->scene->CreateVisual(name);

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose;
  rendering::GeometryPtr geom =
      this->LoadGeometry(*_visual.Geom(), scale, localPose);

  if (geom)
  {
    /// localPose is currently used to handle the normal vector in plane visuals
    /// In general, this can be used to store any local transforms between the
    /// parent Visual and geometry.
    rendering::VisualPtr geomVis;
    if (localPose != math::Pose3d::Zero)
    {
      geomVis = this->dataPtr->scene->CreateVisual(name + "_geom");
      geomVis->SetLocalPose(_visual.Pose() * localPose);
      visualVis = geomVis;
    }

    visualVis->AddGeometry(geom);
    visualVis->SetLocalScale(scale);

    // set material
    rendering::MaterialPtr material{nullptr};
    if (_visual.Material())
    {
      material = this->LoadMaterial(*_visual.Material());
    }
    // Don't set a default material for meshes because they
    // may have their own
    // TODO(anyone) support overriding mesh material
    else if (_visual.Geom()->Type() == sdf::GeometryType::MESH)
    {
      material = geom->Material();
    }
    else
    {
      // create default material
      material = this->dataPtr->scene->Material("ign-grey");
      if (!material)
      {
        material = this->dataPtr->scene->CreateMaterial("ign-grey");
        material->SetAmbient(0.3, 0.3, 0.3);
        material->SetDiffuse(0.7, 0.7, 0.7);
        material->SetSpecular(1.0, 1.0, 1.0);
        material->SetRoughness(0.2);
        material->SetMetalness(1.0);
      }
    }

    // TODO(anyone) set transparency)
    // material->SetTransparency(_visual.Transparency());

    // TODO(anyone) Get roughness and metalness from message instead
    // of giving a default value.
    material->SetRoughness(0.3);
    material->SetMetalness(0.3);

    geom->SetMaterial(material);
  }
  else
  {
    ignerr << "Failed to load geometry for visual: " << _visual.Name()
           << std::endl;
  }

  this->dataPtr->visuals[_id] = visualVis;
  if (parent)
    parent->AddChild(visualVis);

  return visualVis;
}

/////////////////////////////////////////////////
rendering::GeometryPtr SceneManager::LoadGeometry(const sdf::Geometry &_geom,
    math::Vector3d &_scale, math::Pose3d &_localPose)
{
  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose = math::Pose3d::Zero;
  rendering::GeometryPtr geom{nullptr};
  if (_geom.Type() == sdf::GeometryType::BOX)
  {
    geom = this->dataPtr->scene->CreateBox();
    scale = _geom.BoxShape()->Size();
  }
  else if (_geom.Type() == sdf::GeometryType::CYLINDER)
  {
    geom = this->dataPtr->scene->CreateCylinder();
    scale.X() = _geom.CylinderShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.CylinderShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::PLANE)
  {
    geom = this->dataPtr->scene->CreatePlane();
    scale.X() = _geom.PlaneShape()->Size().X();
    scale.Y() = _geom.PlaneShape()->Size().Y();

    // Create a rotation for the plane mesh to account for the normal vector.
    // The rotation is the angle between the +z(0,0,1) vector and the
    // normal, which are both expressed in the local (Visual) frame.
    math::Vector3d normal = _geom.PlaneShape()->Normal();
    localPose.Rot().From2Axes(math::Vector3d::UnitZ, normal.Normalized());
  }
  else if (_geom.Type() == sdf::GeometryType::SPHERE)
  {
    geom = this->dataPtr->scene->CreateSphere();
    scale.X() = _geom.SphereShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = scale.X();
  }
  else if (_geom.Type() == sdf::GeometryType::MESH)
  {
    if (_geom.MeshShape()->Uri().empty())
    {
      ignerr << "Mesh geometry missing uri" << std::endl;
      return geom;
    }
    rendering::MeshDescriptor descriptor;

    // Assume absolute path to mesh file
    descriptor.meshName = _geom.MeshShape()->Uri();

    ignition::common::MeshManager* meshManager =
        ignition::common::MeshManager::Instance();
    descriptor.mesh = meshManager->Load(descriptor.meshName);
    geom = this->dataPtr->scene->CreateMesh(descriptor);
    _scale = _geom.MeshShape()->Scale();
  }
  else
  {
    ignerr << "Unsupported geometry type" << std::endl;
  }
  _scale = scale;
  _localPose = localPose;
  return geom;
}

/////////////////////////////////////////////////
rendering::MaterialPtr SceneManager::LoadMaterial(
    const sdf::Material &_material)
{
  rendering::MaterialPtr material = this->dataPtr->scene->CreateMaterial();
  material->SetAmbient(_material.Ambient());
  material->SetDiffuse(_material.Diffuse());
  material->SetSpecular(_material.Specular());
  material->SetEmissive(_material.Emissive());
  return material;
}

/////////////////////////////////////////////////
rendering::LightPtr SceneManager::CreateLight(int _id, const sdf::Light &_light,
    int _parentId)
{
  if (this->dataPtr->lights.find(_id) != this->dataPtr->lights.end())
  {
    ignerr << "Light with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::LightPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId > 0)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding light: [" << _id << "]" << std::endl;
      return rendering::LightPtr();
    }
    parent = it->second;
  }

  rendering::LightPtr light;
  switch (_light.Type())
  {
    case sdf::LightType::POINT:
      light = this->dataPtr->scene->CreatePointLight();
      break;
    case sdf::LightType::SPOT:
    {
      light = this->dataPtr->scene->CreateSpotLight();
      rendering::SpotLightPtr spotLight =
          std::dynamic_pointer_cast<rendering::SpotLight>(light);
      spotLight->SetInnerAngle(_light.SpotInnerAngle());
      spotLight->SetOuterAngle(_light.SpotOuterAngle());
      spotLight->SetFalloff(_light.SpotFalloff());
      break;
    }
    case sdf::LightType::DIRECTIONAL:
    {
      light = this->dataPtr->scene->CreateDirectionalLight();
      rendering::DirectionalLightPtr dirLight =
          std::dynamic_pointer_cast<rendering::DirectionalLight>(light);

      dirLight->SetDirection(_light.Direction());
      break;
    }
    default:
      ignerr << "Light type not supported" << std::endl;
      return light;
  }

  light->SetLocalPose(_light.Pose());
  light->SetDiffuseColor(_light.Diffuse());
  light->SetSpecularColor(_light.Specular());

  light->SetAttenuationConstant(_light.ConstantAttenuationFactor());
  light->SetAttenuationLinear(_light.LinearAttenuationFactor());
  light->SetAttenuationQuadratic(_light.QuadraticAttenuationFactor());
  light->SetAttenuationRange(_light.AttenuationRange());

  light->SetCastShadows(_light.CastShadows());

  this->dataPtr->lights[_id] = light;

  if (parent)
    parent->AddChild(light);

  return light;
}

/////////////////////////////////////////////////
bool SceneManager::AddSensor(int _id, const std::string &_name,
    int _parentId)
{
  if (this->dataPtr->sensors.find(_id) != this->dataPtr->sensors.end())
  {
    ignerr << "Sensor with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return false;
  }

  rendering::VisualPtr parent;
  if (_parentId > 0)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding sensor: [" << _id << "]" << std::endl;
      return false;
    }
    parent = it->second;
  }

  rendering::SensorPtr sensor = this->dataPtr->scene->SensorByName(_name);
  if (!sensor)
  {
    ignerr << "Unable to find sensor: [" << _name << "]" << std::endl;
    return false;
  }

  if (parent)
  {
    sensor->RemoveParent();
    parent->AddChild(sensor);
  }


  this->dataPtr->sensors[_id] = sensor;
  return true;
}

/////////////////////////////////////////////////
bool SceneManager::HasEntity(int _id) const
{
  return this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end() ||
      this->dataPtr->lights.find(_id) != this->dataPtr->lights.end() ||
      this->dataPtr->sensors.find(_id) != this->dataPtr->sensors.end();
}

/////////////////////////////////////////////////
rendering::NodePtr SceneManager::EntityById(int _id) const
{
  auto vIt = this->dataPtr->visuals.find(_id);
  if (vIt != this->dataPtr->visuals.end())
  {
    return vIt->second;
  }
  else
  {
    auto lIt = this->dataPtr->lights.find(_id);
    if (lIt != this->dataPtr->lights.end())
    {
      return lIt->second;
    }
    else
    {
      auto sIt = this->dataPtr->sensors.find(_id);
      if (sIt != this->dataPtr->sensors.end())
      {
        return sIt->second;
      }
    }
  }
  return rendering::NodePtr();
}
