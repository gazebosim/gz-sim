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

#include "ignition/gazebo/rendering/SceneManager.hh"

using namespace ignition;
using namespace gazebo;

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
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding link: [" << _id << "]" << std::endl;
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
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding visual: [" << _id << "]" << std::endl;
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
  visualVis->SetLocalPose(_visual.Pose());

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
    // TODO(anyone) set transparency)
    // material->SetTransparency(_visual.Transparency());
    if (material)
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
    descriptor.subMeshName = _geom.MeshShape()->Submesh();
    descriptor.centerSubMesh = _geom.MeshShape()->CenterSubmesh();

    common::MeshManager *meshManager = common::MeshManager::Instance();
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
  descriptor.meshName = _actor.SkinFilename();
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  if (nullptr == descriptor.mesh)
  {
    ignerr << "Actor skin mesh [" << _actor.SkinFilename() << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  common::SkeletonPtr meshSkel = descriptor.mesh->MeshSkeleton();
  if (nullptr == meshSkel)
  {
    ignerr << "Mesh skeleton in [" << _actor.SkinFilename() << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::MeshPtr actorMesh = this->dataPtr->scene->CreateMesh(descriptor);
  if (nullptr == actorMesh)
  {
    ignerr << "Actor skin file [" << _actor.SkinFilename() << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  unsigned int numAnims = 0;
  std::map<std::string, unsigned int> mapAnimNameId;
  mapAnimNameId[descriptor.meshName] = numAnims++;

  rendering::VisualPtr actorVisual = this->dataPtr->scene->CreateVisual(name);
  actorVisual->SetLocalPose(_actor.Pose());
  actorVisual->AddGeometry(actorMesh);

  this->dataPtr->visuals[_id] = actorVisual;
  this->dataPtr->actors[_id] = actorMesh;

  // Load all animations
  for (unsigned i = 0; i < _actor.AnimationCount(); ++i)
  {
    std::string animName = _actor.AnimationByIndex(i)->Name();
    std::string animFilename = _actor.AnimationByIndex(i)->Filename();
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
      meshSkel->AddAnimation(animMesh->MeshSkeleton()->Animation(0));
      mapAnimNameId[animName] = numAnims++;
    }
  }
  this->dataPtr->actorSkeletons[_id] = meshSkel;

  std::vector<common::TrajectoryInfo> trajectories;
  if (_actor.TrajectoryCount() != 0)
  {
    // Load all trajectories specified in sdf
    for (unsigned i = 0; i < _actor.TrajectoryCount(); i++)
    {
      common::TrajectoryInfo trajInfo;
      const sdf::Trajectory *trajSdf = _actor.TrajectoryByIndex(i);
      trajInfo.SetId(trajSdf->Id());
      trajInfo.SetAnimIndex(mapAnimNameId[trajSdf->Type()]);

      if (trajSdf->WaypointCount() != 0)
      {
        std::map<double, math::Pose3d> waypoints;
        for (unsigned j = 0; j < trajSdf->WaypointCount(); j++)
        {
          auto point = trajSdf->WaypointByIndex(j);
          waypoints[point->Time()] = point->Pose();
        }
        trajInfo.AddWaypoints(waypoints);
      }
      else
      {
        auto skel = this->dataPtr->actorSkeletons[_id];
        trajInfo.SetDuration(skel->Animation(trajInfo.AnimIndex())->Length());
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
    trajInfo.SetStartTime(0.0);
    trajInfo.SetDuration(skel->Animation(0)->Length());
    trajInfo.SetEndTime(trajInfo.Duration());
    trajInfo.SetTranslated(false);
    trajectories.push_back(trajInfo);
  }

  // sequencing all trajectories
  double time = 0.0;
  for (auto &trajectory : trajectories)
  {
    trajectory.SetStartTime(time);
    time += trajectory.Duration();
    trajectory.SetEndTime(time);
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
      ignerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding light: [" << _id << "]" << std::endl;
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
      ignerr << "Parent entity with Id [" << _parentGazeboId << "] not found. "
             << "Not adding sensor entity [" << _gazeboId << "]" << std::endl;
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
    Entity _id, double _time, bool _loop) const
{
  auto trajs = this->dataPtr->actorTrajectories[_id];
  bool followTraj = true;
  if ( 1 == trajs.size() && nullptr == trajs[0].Waypoints())
  {
    followTraj = false;
  }

  common::TrajectoryInfo traj = trajs[0];
  auto poseFrame = common::PoseKeyFrame(0.0);

  double totalTime = trajs.rbegin()->EndTime();

  if (_loop || _time <= totalTime)
  {
    while (_time > totalTime)
    {
      _time -= totalTime;
    }
    if (followTraj)
    {
      for (auto &trajectory : trajs)
      {
        if (trajectory.StartTime() <= _time && trajectory.EndTime() >= _time)
        {
          traj = trajectory;
          _time -= traj.StartTime();
          traj.Waypoints()->Time(_time);
          traj.Waypoints()->InterpolatedKeyFrame(poseFrame);
          break;
        }
      }
    }
  }

  math::Matrix4d rootTf(poseFrame.Rotation());
  rootTf.SetTranslation(poseFrame.Translation());

  std::map<std::string, math::Matrix4d> allFrames;

  auto vIt = this->dataPtr->actorSkeletons.find(_id);
  if (vIt != this->dataPtr->actorSkeletons.end())
  {
    auto skel = vIt->second;
    unsigned int animIndex = traj.AnimIndex();
    std::map<std::string, math::Matrix4d> rawFrames;

    if (followTraj)
    {
      double distance = traj.DistanceSoFar(_time);
      if (distance < 0.1)
      {
        rawFrames = skel->Animation(animIndex)->PoseAt(_time, _loop);
      }
      else
      {
        rawFrames = skel->Animation(animIndex)->PoseAtX(distance,
                                        skel->RootNode()->Name());
      }
    }
    else
    {
      rawFrames = skel->Animation(animIndex)->PoseAt(_time, _loop);
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
    rendering::VisualPtr _visual) const
{
  rendering::VisualPtr rootVisual =
      this->dataPtr->scene->RootVisual();

  rendering::VisualPtr visual = _visual;
  while (visual && visual->Parent() != rootVisual)
  {
    visual =
      std::dynamic_pointer_cast<rendering::Visual>(visual->Parent());
  }

  return visual;
}
