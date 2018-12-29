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

/////////////////////////////////////////////////
SceneManager::SceneManager()
{
}

/////////////////////////////////////////////////
void SceneManager::SetScene(rendering::ScenePtr _scene)
{
  this->scene = _scene;
}

/////////////////////////////////////////////////
/*void SceneManager::OnPoseVMsg(const msgs::Pose_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    math::Pose3d pose = msgs::Convert(_msg.pose(i));

    // apply additional local poses if available
    const auto it = this->localPoses.find(_msg.pose(i).id());
    if (it != this->localPoses.end())
    {
      pose = pose * it->second;
    }

    this->poses[_msg.pose(i).id()] = pose;
  }
}
*/

/////////////////////////////////////////////////
void SceneManager::Update()
{
  // process msgs
  std::lock_guard<std::mutex> lock(this->mutex);

  for (auto pIt = this->poses.begin(); pIt != this->poses.end();)
  {
    auto vIt = this->visuals.find(pIt->first);
    if (vIt != this->visuals.end())
    {
      vIt->second->SetLocalPose(pIt->second);
      this->poses.erase(pIt++);
    }
    else
    {
      auto lIt = this->lights.find(pIt->first);
      if (lIt != this->lights.end())
      {
        lIt->second->SetLocalPose(pIt->second);
        this->poses.erase(pIt++);
      }
      else
      {
        ++pIt;
      }
    }
  }

  // Note we are clearing the pose msgs here but later on we may need to
  // consider the case where pose msgs arrive before scene/visual msgs
  this->poses.clear();
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadModel(int _id, const sdf::Model &_model,
    int _parentId)
{
  if (this->visuals.find(_id) != this->visuals.end())
    return rendering::VisualPtr();

  std::string name = _model.Name().empty() ? std::to_string(_id) :
      _model.Name();
  rendering::VisualPtr modelVis = this->scene->CreateVisual(name);
  modelVis->SetLocalPose(_model.Pose());
  this->visuals[_id] = modelVis;

  if (_parentId > 0)
  {
    auto it = this->visuals.find(_parentId);
    if (it != this->visuals.end())
      it->second->AddChild(modelVis);
  }
  std::cerr << "adding model " << _model.Name() << std::endl;

  return modelVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadLink(int _id, const sdf::Link &_link,
    int _parentId)
{
  if (this->visuals.find(_id) != this->visuals.end())
    return rendering::VisualPtr();

  std::string name = _link.Name().empty() ? std::to_string(_id) :
      _link.Name();
  rendering::VisualPtr linkVis = this->scene->CreateVisual(name);
  linkVis->SetLocalPose(_link.Pose());
  this->visuals[_id] = linkVis;

  if (_parentId > 0)
  {
    auto it = this->visuals.find(_parentId);
    if (it != this->visuals.end())
      it->second->AddChild(linkVis);
  }

  std::cerr << "adding link " << _link.Name() << std::endl;

  return linkVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::LoadVisual(int _id, const sdf::Visual &_visual,
    int _parentId)
{
  if (this->visuals.find(_id) != this->visuals.end())
    return rendering::VisualPtr();

  if (!_visual.Geom())
    return rendering::VisualPtr();

  std::string name = _visual.Name().empty() ? std::to_string(_id) :
      _visual.Name();
  rendering::VisualPtr visualVis = this->scene->CreateVisual(name);
  this->visuals[_id] = visualVis;

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose;
  rendering::GeometryPtr geom =
      this->LoadGeometry(*_visual.Geom(), scale, localPose);

  visualVis->SetLocalPose(_visual.Pose() * localPose);

  if (geom)
  {
    // store the local pose
    this->localPoses[_id] = localPose;

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
      material = this->scene->Material("ign-grey");
      if (!material)
      {
        material = this->scene->CreateMaterial("ign-grey");
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

  if (_parentId > 0)
  {
    auto it = this->visuals.find(_parentId);
    if (it != this->visuals.end())
      it->second->AddChild(visualVis);
  }

  std::cerr << "adding visual" << _visual.Name() << std::endl;

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
    geom = this->scene->CreateBox();
    scale = _geom.BoxShape()->Size();
  }
  else if (_geom.Type() == sdf::GeometryType::CYLINDER)
  {
    geom = this->scene->CreateCylinder();
    scale.X() = _geom.CylinderShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.CylinderShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::PLANE)
  {
    geom = this->scene->CreatePlane();
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
    geom = this->scene->CreateSphere();
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
    // TODO(anyone) resolve filename path?
    // currently assumes absolute path to mesh file
    descriptor.meshName = _geom.MeshShape()->Uri();

    ignition::common::MeshManager* meshManager =
        ignition::common::MeshManager::Instance();
    descriptor.mesh = meshManager->Load(descriptor.meshName);
    geom = this->scene->CreateMesh(descriptor);
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
rendering::MaterialPtr SceneManager::LoadMaterial(const sdf::Material &_material)
{
  rendering::MaterialPtr material = this->scene->CreateMaterial();
  material->SetAmbient(_material.Ambient());
  material->SetDiffuse(_material.Diffuse());
  material->SetSpecular(_material.Specular());
  material->SetEmissive(_material.Emissive());
  return material;
}

/////////////////////////////////////////////////
rendering::LightPtr SceneManager::LoadLight(int _id, const sdf::Light &_light,
    int _parentId)
{
  rendering::LightPtr light;

  switch (_light.Type())
  {
    case sdf::LightType::POINT:
      light = this->scene->CreatePointLight();
      break;
    case sdf::LightType::SPOT:
    {
      light = this->scene->CreateSpotLight();
      rendering::SpotLightPtr spotLight =
          std::dynamic_pointer_cast<rendering::SpotLight>(light);
      spotLight->SetInnerAngle(_light.SpotInnerAngle());
      spotLight->SetOuterAngle(_light.SpotOuterAngle());
      spotLight->SetFalloff(_light.SpotFalloff());
      break;
    }
    case sdf::LightType::DIRECTIONAL:
    {
      light = this->scene->CreateDirectionalLight();
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

  this->lights[_id] = light;

  if (_parentId > 0)
  {
    auto it = this->lights.find(_id);
    if (it != this->lights.end())
      it->second->AddChild(light);
  }

  return light;
}

/////////////////////////////////////////////////
bool SceneManager::HasEntity(int _id) const
{
  return this->visuals.find(_id) != this->visuals.end() &&
      this->lights.find(_id) != this->lights.end();
}
