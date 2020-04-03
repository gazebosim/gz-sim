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


#include <map>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/Skeleton.hh>
#include <ignition/common/SkeletonAnimation.hh>
#include <ignition/common/MeshManager.hh>

#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Light.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Private data class.
class ignition::gazebo::SceneManagerPrivate
{
  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Ignition Gazebo.
  public: Entity worldId{0};

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Map of visual entity in Gazebo to visual pointers.
  public: std::map<Entity, rendering::VisualPtr> visuals;

  /// \brief Map of actor entity in Gazebo to actor pointers.
  public: std::map<Entity, rendering::MeshPtr> actors;

  /// \brief Map of actor entity in Gazebo to actor animations.
  public: std::map<Entity, common::SkeletonPtr> actorSkeletons;

  /// \brief Map of actor entity to the associated trajectories.
  public: std::map<Entity, std::vector<common::TrajectoryInfo>>
                    actorTrajectories;

  /// \brief Map of light entity in Gazebo to light pointers.
  public: std::map<Entity, rendering::LightPtr> lights;

  /// \brief Map of sensor entity in Gazebo to sensor pointers.
  public: std::map<Entity, rendering::SensorPtr> sensors;
};


/////////////////////////////////////////////////
SceneManager::SceneManager()
  : dataPtr(std::make_unique<SceneManagerPrivate>())
{
}

/////////////////////////////////////////////////
SceneManager::~SceneManager() = default;

/////////////////////////////////////////////////
void SceneManager::SetScene(rendering::ScenePtr _scene)
{
  this->dataPtr->scene = std::move(_scene);
}

/////////////////////////////////////////////////
rendering::ScenePtr SceneManager::Scene() const
{
  return this->dataPtr->scene;
}

/////////////////////////////////////////////////
void SceneManager::SetWorldId(Entity _id)
{
  this->dataPtr->worldId = _id;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateModel(Entity _id,
    const sdf::Model &_model, Entity _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  std::string name = _model.Name().empty() ? std::to_string(_id) :
      _model.Name();

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding model visual with ID[" << _id
             << "]  and name [" << name << "] to the rendering scene."
             << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  if (parent)
    name = parent->Name() +  "::" + name;

  if (this->dataPtr->scene->HasVisualName(name))
  {
    ignerr << "Visual: [" << name << "] already exists" << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr modelVis = this->dataPtr->scene->CreateVisual(name);
  modelVis->SetUserData("gazebo-entity", static_cast<int>(_id));
  modelVis->SetUserData("pause-update", static_cast<int>(0));
  modelVis->SetLocalPose(_model.RawPose());
  this->dataPtr->visuals[_id] = modelVis;

  if (parent)
    parent->AddChild(modelVis);
  else
    this->dataPtr->scene->RootVisual()->AddChild(modelVis);

  return modelVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateLink(Entity _id,
    const sdf::Link &_link, Entity _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  std::string name = _link.Name().empty() ? std::to_string(_id) :
      _link.Name();
  if (parent)
    name = parent->Name() + "::" + name;
  rendering::VisualPtr linkVis = this->dataPtr->scene->CreateVisual(name);
  linkVis->SetUserData("gazebo-entity", static_cast<int>(_id));
  linkVis->SetUserData("pause-update", static_cast<int>(0));
  linkVis->SetLocalPose(_link.RawPose());
  this->dataPtr->visuals[_id] = linkVis;

  if (parent)
    parent->AddChild(linkVis);

  return linkVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateVisual(Entity _id,
    const sdf::Visual &_visual, Entity _parentId)
{
  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
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
  visualVis->SetUserData("gazebo-entity", static_cast<int>(_id));
  visualVis->SetUserData("pause-update", static_cast<int>(0));
  visualVis->SetLocalPose(_visual.RawPose());

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
      geomVis->SetUserData("gazebo-entity", static_cast<int>(_id));
      geomVis->SetUserData("pause-update", static_cast<int>(0));
      geomVis->SetLocalPose(_visual.RawPose() * localPose);
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
    else if (_visual.Geom()->Type() != sdf::GeometryType::MESH)
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
    else
    {
      // meshes created by mesh loader may have their own materials
      // update/override their properties based on input sdf element values
      auto mesh = std::dynamic_pointer_cast<rendering::Mesh>(geom);
      for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i)
      {
        auto submesh = mesh->SubMeshByIndex(i);
        auto submeshMat = submesh->Material();
        if (submeshMat)
        {
          double productAlpha = (1.0-_visual.Transparency()) *
              (1.0 - submeshMat->Transparency());
          submeshMat->SetTransparency(1 - productAlpha);
          submeshMat->SetCastShadows(_visual.CastShadows());
        }
      }
    }

    if (material)
    {
      // set transparency
      material->SetTransparency(_visual.Transparency());

      // cast shadows
      material->SetCastShadows(_visual.CastShadows());

      geom->SetMaterial(material);
      // todo(anyone) SetMaterial function clones the input material.
      // but does not take ownership of it so we need to destroy it here.
      // This is not ideal. We should let ign-rendering handle the lifetime
      // of this material
      this->dataPtr->scene->DestroyMaterial(material);
    }
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
    auto fullPath = asFullPath(_geom.MeshShape()->Uri(),
        _geom.MeshShape()->FilePath());
    if (fullPath.empty())
    {
      ignerr << "Mesh geometry missing uri" << std::endl;
      return geom;
    }
    rendering::MeshDescriptor descriptor;

    // Assume absolute path to mesh file
    descriptor.meshName = fullPath;
    descriptor.subMeshName = _geom.MeshShape()->Submesh();
    descriptor.centerSubMesh = _geom.MeshShape()->CenterSubmesh();

    ignition::common::MeshManager *meshManager =
        ignition::common::MeshManager::Instance();
    descriptor.mesh = meshManager->Load(descriptor.meshName);
    geom = this->dataPtr->scene->CreateMesh(descriptor);
    scale = _geom.MeshShape()->Scale();
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

  // parse PBR params
  const sdf::Pbr *pbr = _material.PbrMaterial();
  if (pbr)
  {
    sdf::PbrWorkflow *workflow = nullptr;
    const sdf::PbrWorkflow *metal =
        pbr->Workflow(sdf::PbrWorkflowType::METAL);
    if (metal)
    {
      double roughness = metal->Roughness();
      double metalness = metal->Metalness();
      material->SetRoughness(roughness);
      material->SetMetalness(metalness);

      // roughness map
      std::string roughnessMap = metal->RoughnessMap();
      if (!roughnessMap.empty())
      {
        std::string fullPath = common::findFile(roughnessMap);
        if (!fullPath.empty())
          material->SetRoughnessMap(fullPath);
        else
          ignerr << "Unable to find file [" << roughnessMap << "]\n";
      }

      // metalness map
      std::string metalnessMap = metal->MetalnessMap();
      if (!metalnessMap.empty())
      {
        std::string fullPath = common::findFile(metalnessMap);
        if (!fullPath.empty())
          material->SetMetalnessMap(fullPath);
        else
          ignerr << "Unable to find file [" << metalnessMap << "]\n";
      }
      workflow = const_cast<sdf::PbrWorkflow *>(metal);
    }
    else
    {
      ignerr << "PBR material: currently only metal workflow is supported"
             << std::endl;
    }

    // albedo map
    std::string albedoMap = workflow->AlbedoMap();
    if (!albedoMap.empty())
    {
      std::string fullPath = common::findFile(albedoMap);
      if (!fullPath.empty())
      {
        material->SetTexture(fullPath);
      }
      else
        ignerr << "Unable to find file [" << albedoMap << "]\n";
    }

    // normal map
    std::string normalMap = workflow->NormalMap();
    if (!normalMap.empty())
    {
      std::string fullPath = common::findFile(normalMap);
      if (!fullPath.empty())
        material->SetNormalMap(fullPath);
      else
        ignerr << "Unable to find file [" << normalMap << "]\n";
    }


    // environment map
    std::string environmentMap = workflow->EnvironmentMap();
    if (!environmentMap.empty())
    {
      std::string fullPath = common::findFile(environmentMap);
      if (!fullPath.empty())
        material->SetEnvironmentMap(fullPath);
      else
        ignerr << "Unable to find file [" << environmentMap << "]\n";
    }

    // emissive map
    std::string emissiveMap = workflow->EmissiveMap();
    if (!emissiveMap.empty())
    {
      std::string fullPath = common::findFile(emissiveMap);
      if (!fullPath.empty())
        material->SetEmissiveMap(fullPath);
      else
        ignerr << "Unable to find file [" << emissiveMap << "]\n";
    }
  }
  return material;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateActor(Entity _id,
    const sdf::Actor &_actor, Entity _parentId)
{
  // creating an actor needs to create the visual and the mesh
  // the visual is stored in: this->dataPtr->visuals
  // the mesh is stored in: this->dataPtr->actors

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    ignerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  std::string name = _actor.Name().empty() ? std::to_string(_id) :
      _actor.Name();

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding actor with ID[" << _id
             << "]  and name [" << name << "] to the rendering scene."
             << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  if (parent)
    name = parent->Name() +  "::" + name;

  rendering::MeshDescriptor descriptor;
  descriptor.meshName = asFullPath(_actor.SkinFilename(), _actor.FilePath());
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  if (nullptr == descriptor.mesh)
  {
    ignerr << "Actor skin mesh [" << descriptor.meshName << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  common::SkeletonPtr meshSkel = descriptor.mesh->MeshSkeleton();
  if (nullptr == meshSkel)
  {
    ignerr << "Mesh skeleton in [" << descriptor.meshName << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::MeshPtr actorMesh = this->dataPtr->scene->CreateMesh(descriptor);
  if (nullptr == actorMesh)
  {
    ignerr << "Actor skin file [" << descriptor.meshName << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  unsigned int numAnims = 0;
  std::map<std::string, unsigned int> mapAnimNameId;
  mapAnimNameId[descriptor.meshName] = numAnims++;

  rendering::VisualPtr actorVisual = this->dataPtr->scene->CreateVisual(name);
  actorVisual->SetLocalPose(_actor.RawPose());
  actorVisual->SetUserData("gazebo-entity", static_cast<int>(_id));
  actorVisual->SetUserData("pause-update", static_cast<int>(0));
  actorVisual->AddGeometry(actorMesh);

  this->dataPtr->visuals[_id] = actorVisual;
  this->dataPtr->actors[_id] = actorMesh;

  // Load all animations
  for (unsigned i = 0; i < _actor.AnimationCount(); ++i)
  {
    std::string animName = _actor.AnimationByIndex(i)->Name();
    std::string animFilename = asFullPath(
        _actor.AnimationByIndex(i)->Filename(),
        _actor.AnimationByIndex(i)->FilePath());
    double animScale = _actor.AnimationByIndex(i)->Scale();

    std::string extension = animFilename.substr(animFilename.rfind('.') + 1,
                                  animFilename.size());
    std::transform(extension.begin(), extension.end(),
                    extension.begin(), ::tolower);

    if (extension == "bvh")
    {
      meshSkel->AddBvhAnimation(animFilename, animScale);
      mapAnimNameId[animName] = numAnims++;
    }
    else if (extension == "dae")
    {
      common::MeshManager::Instance()->Load(animFilename);
      auto animMesh = common::MeshManager::Instance()->MeshByName(animFilename);

      // add the first animation
      if (animMesh->MeshSkeleton()->AnimationCount() > 1)
      {
        ignwarn << "File [" << animFilename
            << "] has more than one animation, but only the 1st one is used."
            << std::endl;
      }
      auto firstAnim = animMesh->MeshSkeleton()->Animation(0);
      if (nullptr == firstAnim)
      {
        ignerr << "Failed to get animations from [" << animFilename
                << "]" << std::endl;
        return nullptr;
      }
      meshSkel->AddAnimation(firstAnim);
      mapAnimNameId[animName] = numAnims++;
    }
  }
  this->dataPtr->actorSkeletons[_id] = meshSkel;

  using TP = std::chrono::steady_clock::time_point;

  std::vector<common::TrajectoryInfo> trajectories;
  if (_actor.TrajectoryCount() != 0)
  {
    // Load all trajectories specified in sdf
    for (unsigned i = 0; i < _actor.TrajectoryCount(); i++)
    {
      const sdf::Trajectory *trajSdf = _actor.TrajectoryByIndex(i);
      if (nullptr == trajSdf)
      {
        ignerr << "Null trajectory SDF for [" << _actor.Name() << "]"
               << std::endl;
        continue;
      }

      common::TrajectoryInfo trajInfo;
      trajInfo.SetId(trajSdf->Id());
      trajInfo.SetAnimIndex(mapAnimNameId[trajSdf->Type()]);

      if (trajSdf->WaypointCount() != 0)
      {
        std::map<TP, math::Pose3d> waypoints;
        for (unsigned j = 0; j < trajSdf->WaypointCount(); j++)
        {
          auto point = trajSdf->WaypointByIndex(j);
          TP pointTp(std::chrono::milliseconds(
                    static_cast<int>(point->Time()*1000)));
          waypoints[pointTp] = point->Pose();
        }
        trajInfo.SetWaypoints(waypoints);
      }
      else
      {
        trajInfo.SetTranslated(false);
      }
      trajectories.push_back(trajInfo);
    }
  }
  // if there are no trajectories, but there are animations, add a trajectory
  else
  {
    auto skel = this->dataPtr->actorSkeletons[_id];
    common::TrajectoryInfo trajInfo;
    trajInfo.SetId(0);
    trajInfo.SetAnimIndex(0);
    trajInfo.SetStartTime(TP(0ms));
    auto timepoint = std::chrono::milliseconds(
                  static_cast<int>(skel->Animation(0)->Length() * 1000));
    trajInfo.SetEndTime(TP(timepoint));
    trajInfo.SetTranslated(false);
    trajectories.push_back(trajInfo);
  }

  // sequencing all trajectories
  auto delayStartTime = std::chrono::milliseconds(
              static_cast<int>(_actor.ScriptDelayStart() * 1000));
  TP time(delayStartTime);
  for (auto &trajectory : trajectories)
  {
    auto dura = trajectory.Duration();
    trajectory.SetStartTime(time);
    time += dura;
    trajectory.SetEndTime(time);
  }

  // loop flag: add a placeholder trajectory to indicate no loop
  if (!_actor.ScriptLoop())
  {
    common::TrajectoryInfo noLoopTrajInfo;
    noLoopTrajInfo.SetId(0);
    noLoopTrajInfo.SetAnimIndex(0);
    noLoopTrajInfo.SetStartTime(time);
    noLoopTrajInfo.SetEndTime(time);
    noLoopTrajInfo.SetTranslated(false);
    trajectories.push_back(noLoopTrajInfo);
  }

  this->dataPtr->actorTrajectories[_id] = trajectories;

  if (parent)
    parent->AddChild(actorVisual);
  else
    this->dataPtr->scene->RootVisual()->AddChild(actorVisual);

  return actorVisual;
}

/////////////////////////////////////////////////
rendering::LightPtr SceneManager::CreateLight(Entity _id,
    const sdf::Light &_light, Entity _parentId)
{
  if (this->dataPtr->lights.find(_id) != this->dataPtr->lights.end())
  {
    ignerr << "Light with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::LightPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return rendering::LightPtr();
    }
    parent = it->second;
  }

  std::string name = _light.Name().empty() ? std::to_string(_id) :
      _light.Name();
  if (parent)
    name = parent->Name() +  "::" + name;

  rendering::LightPtr light;
  switch (_light.Type())
  {
    case sdf::LightType::POINT:
      light = this->dataPtr->scene->CreatePointLight(name);
      break;
    case sdf::LightType::SPOT:
    {
      light = this->dataPtr->scene->CreateSpotLight(name);
      rendering::SpotLightPtr spotLight =
          std::dynamic_pointer_cast<rendering::SpotLight>(light);
      spotLight->SetInnerAngle(_light.SpotInnerAngle());
      spotLight->SetOuterAngle(_light.SpotOuterAngle());
      spotLight->SetFalloff(_light.SpotFalloff());
      break;
    }
    case sdf::LightType::DIRECTIONAL:
    {
      light = this->dataPtr->scene->CreateDirectionalLight(name);
      rendering::DirectionalLightPtr dirLight =
          std::dynamic_pointer_cast<rendering::DirectionalLight>(light);

      dirLight->SetDirection(_light.Direction());
      break;
    }
    default:
      ignerr << "Light type not supported" << std::endl;
      return light;
  }

  // \todo(anyone) Set entity user data once rendering Node supports it
  light->SetLocalPose(_light.RawPose());
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
bool SceneManager::AddSensor(Entity _gazeboId, const std::string &_sensorName,
    Entity _parentGazeboId)
{
  if (this->dataPtr->sensors.find(_gazeboId) != this->dataPtr->sensors.end())
  {
    ignerr << "Sensor for entity [" << _gazeboId
           << "] already exists in the scene" << std::endl;
    return false;
  }

  rendering::VisualPtr parent;
  if (_parentGazeboId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentGazeboId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return false;
    }
    parent = it->second;
  }

  rendering::SensorPtr sensor = this->dataPtr->scene->SensorByName(_sensorName);
  if (!sensor)
  {
    ignerr << "Unable to find sensor [" << _sensorName << "]" << std::endl;
    return false;
  }

  if (parent)
  {
    sensor->RemoveParent();
    parent->AddChild(sensor);
  }

  this->dataPtr->sensors[_gazeboId] = sensor;
  return true;
}

/////////////////////////////////////////////////
bool SceneManager::HasEntity(Entity _id) const
{
  return this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end() ||
      this->dataPtr->actors.find(_id) != this->dataPtr->actors.end() ||
      this->dataPtr->lights.find(_id) != this->dataPtr->lights.end() ||
      this->dataPtr->sensors.find(_id) != this->dataPtr->sensors.end();
}

/////////////////////////////////////////////////
rendering::NodePtr SceneManager::NodeById(Entity _id) const
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

/////////////////////////////////////////////////
rendering::MeshPtr SceneManager::ActorMeshById(Entity _id) const
{
  auto vIt = this->dataPtr->actors.find(_id);
  if (vIt != this->dataPtr->actors.end())
  {
    return vIt->second;
  }
  return rendering::MeshPtr();
}

/////////////////////////////////////////////////
std::map<std::string, math::Matrix4d> SceneManager::ActorMeshAnimationAt(
    Entity _id, std::chrono::steady_clock::duration _time) const
{
  std::map<std::string, math::Matrix4d> allFrames;

  if (this->dataPtr->actorTrajectories.find(_id)
      == this->dataPtr->actorTrajectories.end())
  {
    return allFrames;
  }

  auto trajs = this->dataPtr->actorTrajectories[_id];
  bool followTraj = true;
  if (1 == trajs.size() && nullptr == trajs[0].Waypoints())
  {
    followTraj = false;
  }

  auto firstTraj = trajs.begin();
  auto poseFrame = common::PoseKeyFrame(0.0);

  common::TrajectoryInfo traj = trajs[0];

  using TP = std::chrono::steady_clock::time_point;
  auto totalTime = trajs.rbegin()->EndTime() - trajs.begin()->StartTime();

  // delay start
  if (_time < (trajs.begin()->StartTime() - TP(0ms)))
  {
    _time = std::chrono::steady_clock::duration(0);
  }
  else
  {
    _time -= trajs.begin()->StartTime() - TP(0ms);
  }

  bool noLoop = trajs.rbegin()->StartTime() != TP(0ms)
            && trajs.rbegin()->StartTime() == trajs.rbegin()->EndTime();

  if (_time >= totalTime && noLoop)
  {
    _time = totalTime;
  }

  if (!noLoop || _time <= totalTime)
  {
    while (_time > totalTime)
    {
      _time = _time - totalTime;
    }
    if (followTraj)
    {
      for (auto &trajectory : trajs)
      {
        if (trajectory.StartTime() - firstTraj->StartTime() <= _time
            && trajectory.EndTime() - firstTraj->StartTime() >= _time)
        {
          traj = trajectory;
          _time -= traj.StartTime() - firstTraj->StartTime();

          traj.Waypoints()->Time(std::chrono::duration<double>(_time).count());
          traj.Waypoints()->InterpolatedKeyFrame(poseFrame);
          break;
        }
      }
    }
  }

  math::Matrix4d rootTf(poseFrame.Rotation());
  rootTf.SetTranslation(poseFrame.Translation());

  auto vIt = this->dataPtr->actorSkeletons.find(_id);
  if (vIt != this->dataPtr->actorSkeletons.end())
  {
    auto skel = vIt->second;
    unsigned int animIndex = traj.AnimIndex();
    std::map<std::string, math::Matrix4d> rawFrames;

    double timeSeconds = std::chrono::duration<double>(_time).count();

    if (followTraj)
    {
      double distance = traj.DistanceSoFar(_time);
      if (distance < 0.1)
      {
        rawFrames = skel->Animation(animIndex)->PoseAt(timeSeconds, !noLoop);
      }
      else
      {
        rawFrames = skel->Animation(animIndex)->PoseAtX(distance,
                                        skel->RootNode()->Name());
      }
    }
    else
    {
      rawFrames = skel->Animation(animIndex)->PoseAt(timeSeconds, !noLoop);
    }

    for (auto pair : rawFrames)
    {
      std::string nodeName = pair.first;
      auto nodeTf = pair.second;

      std::string skinName = skel->NodeNameAnimToSkin(animIndex, nodeName);
      math::Matrix4d skinTf = skel->AlignTranslation(animIndex, nodeName)
              * nodeTf * skel->AlignRotation(animIndex, nodeName);

      allFrames[skinName] = skinTf;
    }
  }

  // correct animation root pose
  auto skel = this->dataPtr->actorSkeletons[_id];

  if (followTraj)
  {
    allFrames["actorPose"] = rootTf;
  }
  else
  {
    allFrames["actorPose"] = math::Matrix4d(math::Quaterniond::Identity);
    allFrames["actorPose"].SetTranslation(
            allFrames[skel->RootNode()->Name()].Translation());
  }

  allFrames[skel->RootNode()->Name()].SetTranslation(math::Vector3d::Zero);
  return allFrames;
}

/////////////////////////////////////////////////
void SceneManager::RemoveEntity(Entity _id)
{
  {
    auto it = this->dataPtr->visuals.find(_id);
    if (it != this->dataPtr->visuals.end())
    {
      this->dataPtr->scene->DestroyVisual(it->second);
      this->dataPtr->visuals.erase(it);
      return;
    }
  }

  {
    auto it = this->dataPtr->lights.find(_id);
    if (it != this->dataPtr->lights.end())
    {
      this->dataPtr->scene->DestroyLight(it->second);
      this->dataPtr->lights.erase(it);
      return;
    }
  }

  {
    auto it = this->dataPtr->sensors.find(_id);
    if (it != this->dataPtr->sensors.end())
    {
      // Stop keeping track of it but don't destroy it;
      // ign-sensors is the one responsible for that.
      this->dataPtr->sensors.erase(it);
      return;
    }
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::TopLevelVisual(
// NOLINTNEXTLINE
    rendering::VisualPtr _visual) const
{
  auto node = this->TopLevelNode(_visual);
  return std::dynamic_pointer_cast<rendering::Visual>(node);
}

/////////////////////////////////////////////////
rendering::NodePtr SceneManager::TopLevelNode(
    const rendering::NodePtr &_node) const
{
  rendering::NodePtr rootNode =
      this->dataPtr->scene->RootVisual();

  rendering::NodePtr node = _node;
  while (node && node->Parent() != rootNode)
  {
    node =
      std::dynamic_pointer_cast<rendering::Node>(node->Parent());
  }

  return node;
}

/////////////////////////////////////////////////
Entity SceneManager::EntityFromNode(const rendering::NodePtr &_node) const
{
  // TODO(anyone) On Dome, set entity ID into node with SetUserData
  auto visual = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (visual)
  {
    auto found = std::find_if(std::begin(this->dataPtr->visuals),
        std::end(this->dataPtr->visuals),
        [&](const std::pair<Entity, rendering::VisualPtr> &_item)
    {
      return _item.second == visual;
    });

    if (found != this->dataPtr->visuals.end())
    {
      return found->first;
    }
  }

  auto light = std::dynamic_pointer_cast<rendering::Light>(_node);
  if (light)
  {
    auto found = std::find_if(std::begin(this->dataPtr->lights),
        std::end(this->dataPtr->lights),
        [&](const std::pair<Entity, rendering::LightPtr> &_item)
    {
      return _item.second == light;
    });

    if (found != this->dataPtr->lights.end())
    {
      return found->first;
    }
  }

  return kNullEntity;
}
