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
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <ignition/common/Animation.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/HeightmapData.hh>
#include <ignition/common/ImageHeightmap.hh>
#include <ignition/common/KeyFrame.hh>
#include <ignition/common/Skeleton.hh>
#include <ignition/common/SkeletonAnimation.hh>
#include <ignition/common/MeshManager.hh>

#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Heightmap.hh>
#include <ignition/rendering/HeightmapDescriptor.hh>
#include <ignition/rendering/Light.hh>
#include <ignition/rendering/Material.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

using TP = std::chrono::steady_clock::time_point;

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

  /// \brief Helper function to compute actor trajectory at specified tiime
  /// \param[in] _id Actor entity's unique id
  /// \param[in] _time Simulation time
  /// \return AnimationUpdateData with trajectory related fields filled. It
  /// also sets the time point in which the animation should be played
  public: AnimationUpdateData ActorTrajectoryAt(
      Entity _id, const std::chrono::steady_clock::duration &_time) const;
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
Entity SceneManager::WorldId() const
{
  return this->dataPtr->worldId;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateModel(Entity _id,
    const sdf::Model &_model, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

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
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

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
  linkVis->SetUserData("gazebo-entity", static_cast<int>(_id));
  this->dataPtr->visuals[_id] = linkVis;

  if (parent)
    parent->AddChild(linkVis);

  return linkVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateVisual(Entity _id,
    const sdf::Visual &_visual, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

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
    if (_visual.Geom()->Type() == sdf::GeometryType::HEIGHTMAP)
    {
      // Heightmap's material is loaded together with it.
    }
    else if (_visual.Material())
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

  // visibility flags
  visualVis->SetVisibilityFlags(_visual.VisibilityFlags());

  this->dataPtr->visuals[_id] = visualVis;
  if (parent)
    parent->AddChild(visualVis);

  return visualVis;
}

/////////////////////////////////////////////////
rendering::GeometryPtr SceneManager::LoadGeometry(const sdf::Geometry &_geom,
    math::Vector3d &_scale, math::Pose3d &_localPose)
{
  if (!this->dataPtr->scene)
    return rendering::GeometryPtr();

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
  else if (_geom.Type() == sdf::GeometryType::HEIGHTMAP)
  {
    auto fullPath = asFullPath(_geom.HeightmapShape()->Uri(),
        _geom.HeightmapShape()->FilePath());
    if (fullPath.empty())
    {
      ignerr << "Heightmap geometry missing URI" << std::endl;
      return geom;
    }

    auto data = std::make_shared<common::ImageHeightmap>();
    if (data->Load(fullPath) < 0)
    {
      ignerr << "Failed to load heightmap image data from [" << fullPath << "]"
             << std::endl;
      return geom;
    }

    rendering::HeightmapDescriptor descriptor;
    descriptor.SetData(data);
    descriptor.SetSize(_geom.HeightmapShape()->Size());
    descriptor.SetSampling(_geom.HeightmapShape()->Sampling());

    for (uint64_t i = 0; i < _geom.HeightmapShape()->TextureCount(); ++i)
    {
      auto textureSdf = _geom.HeightmapShape()->TextureByIndex(i);
      rendering::HeightmapTexture textureDesc;
      textureDesc.SetSize(textureSdf->Size());
      textureDesc.SetDiffuse(asFullPath(textureSdf->Diffuse(),
          _geom.HeightmapShape()->FilePath()));
      textureDesc.SetNormal(asFullPath(textureSdf->Normal(),
          _geom.HeightmapShape()->FilePath()));
      descriptor.AddTexture(textureDesc);
    }

    for (uint64_t i = 0; i < _geom.HeightmapShape()->BlendCount(); ++i)
    {
      auto blendSdf = _geom.HeightmapShape()->BlendByIndex(i);
      rendering::HeightmapBlend blendDesc;
      blendDesc.SetMinHeight(blendSdf->MinHeight());
      blendDesc.SetFadeDistance(blendSdf->FadeDistance());
      descriptor.AddBlend(blendDesc);
    }

    geom = this->dataPtr->scene->CreateHeightmap(descriptor);
    if (nullptr == geom)
    {
      ignerr << "Failed to create heightmap [" << fullPath << "]" << std::endl;
    }
    scale = _geom.HeightmapShape()->Size();
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
  if (!this->dataPtr->scene)
    return rendering::MaterialPtr();

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
        // Use alpha channel for transparency
        material->SetAlphaFromTexture(true, 0.5, _material.DoubleSided());
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
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

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

  // todo(anyone) create a copy of meshSkel so we don't modify the original
  // when adding animations!
  common::SkeletonPtr meshSkel = descriptor.mesh->MeshSkeleton();
  if (nullptr == meshSkel)
  {
    ignerr << "Mesh skeleton in [" << descriptor.meshName << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  unsigned int numAnims = 0;
  std::map<std::string, unsigned int> mapAnimNameId;
  mapAnimNameId[descriptor.meshName] = numAnims++;

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
      // do not add duplicate animation
      // start checking from index 1 since index 0 is reserved by skin mesh
      bool addAnim = true;
      for (unsigned int a = 1; a < meshSkel->AnimationCount(); ++a)
      {
        if (meshSkel->Animation(a)->Name() == animFilename)
        {
          addAnim = false;
          break;
        }
      }
      if (addAnim)
        meshSkel->AddBvhAnimation(animFilename, animScale);
      mapAnimNameId[animName] = numAnims++;
    }
    else if (extension == "dae")
    {
      // Load the mesh if it has not been loaded before
      const common::Mesh *animMesh = nullptr;
      if (!meshManager->HasMesh(animFilename))
      {
        animMesh = meshManager->Load(animFilename);
        if (animMesh->MeshSkeleton()->AnimationCount() > 1)
        {
          ignwarn << "File [" << animFilename
              << "] has more than one animation, but only the 1st one is used."
              << std::endl;
        }
      }
      animMesh = meshManager->MeshByName(animFilename);

      // add the first animation
      auto firstAnim = animMesh->MeshSkeleton()->Animation(0);
      if (nullptr == firstAnim)
      {
        ignerr << "Failed to get animations from [" << animFilename
                << "]" << std::endl;
        return nullptr;
      }
      // do not add duplicate animation
      // start checking from index 1 since index 0 is reserved by skin mesh
      bool addAnim = true;
      for (unsigned int a = 1; a < meshSkel->AnimationCount(); ++a)
      {
        if (meshSkel->Animation(a)->Name() == animName)
        {
          addAnim = false;
          break;
        }
      }
      if (addAnim)
      {
        // collada loader loads animations with name that defaults to
        // "animation0", "animation"1", etc causing conflicts in names
        // when multiple animations are added to meshSkel.
        // We have to clone the skeleton animation before giving it a unique
        // name otherwise if mulitple instances of the same animation were added
        // to meshSkel, changing the name that would also change the name of
        // other instances of the animation
        // todo(anyone) cloning is inefficient and error-prone. We should
        // add a copy constructor to animation classes in ign-common.
        // The proper fix is probably to update ign-rendering to allow it to
        // load multiple animations of the same name
        common::SkeletonAnimation *skelAnim =
            new common::SkeletonAnimation(animName);
        for (unsigned int j = 0; j < meshSkel->NodeCount(); ++j)
        {
          common::SkeletonNode *node = meshSkel->NodeByHandle(j);
          common::NodeAnimation *nodeAnim = firstAnim->NodeAnimationByName(
              node->Name());
          if (!nodeAnim)
            continue;
          for (unsigned int k = 0; k < nodeAnim->FrameCount(); ++k)
          {
            std::pair<double, math::Matrix4d> keyFrame = nodeAnim->KeyFrame(k);
            skelAnim->AddKeyFrame(
                node->Name(), keyFrame.first, keyFrame.second);
          }
        }

        meshSkel->AddAnimation(skelAnim);
      }
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
        // Animations are offset by 1 because index 0 is taken by the mesh name
        auto animation = _actor.AnimationByIndex(trajInfo.AnimIndex()-1);

        if (animation && animation->InterpolateX())
        {
          // warn if no x displacement can be interpolated
          // warn only once per mesh
          static std::unordered_set<std::string> animInterpolateCheck;
          if (animInterpolateCheck.count(animation->Filename()) == 0)
          {
            std::string rootNodeName = meshSkel->RootNode()->Name();
            common::SkeletonAnimation *skelAnim =
                meshSkel->Animation(trajInfo.AnimIndex());
            common::NodeAnimation *rootNode = skelAnim->NodeAnimationByName(
                rootNodeName);
            math::Matrix4d lastPos = rootNode->KeyFrame(
                rootNode->FrameCount() - 1).second;
            math::Matrix4d firstPos = rootNode->KeyFrame(0).second;
            if (!math::equal(firstPos.Translation().X(),
                lastPos.Translation().X()))
            {
              trajInfo.Waypoints()->SetInterpolateX(animation->InterpolateX());
            }
            else
            {
              ignwarn << "Animation has no x displacement. "
                      << "Ignoring <interpolate_x> for the animation in '"
                      << animation->Filename() << "'." << std::endl;
            }
            animInterpolateCheck.insert(animation->Filename());
          }
        }
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

  // create mesh with animations
  rendering::MeshPtr actorMesh = this->dataPtr->scene->CreateMesh(
      descriptor);
  if (nullptr == actorMesh)
  {
    ignerr << "Actor skin file [" << descriptor.meshName << "] not found."
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr actorVisual = this->dataPtr->scene->CreateVisual(name);
  actorVisual->SetLocalPose(_actor.RawPose());
  actorVisual->AddGeometry(actorMesh);
  actorVisual->SetUserData("gazebo-entity", static_cast<int>(_id));
  actorVisual->SetUserData("pause-update", static_cast<int>(0));

  this->dataPtr->visuals[_id] = actorVisual;
  this->dataPtr->actors[_id] = actorMesh;


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
  if (!this->dataPtr->scene)
    return rendering::LightPtr();

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
  if (!this->dataPtr->scene)
    return false;

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
common::SkeletonPtr SceneManager::ActorSkeletonById(Entity _id) const
{
  auto it = this->dataPtr->actorSkeletons.find(_id);
  if (it != this->dataPtr->actorSkeletons.end())
    return it->second;

  return common::SkeletonPtr();
}

/////////////////////////////////////////////////
AnimationUpdateData SceneManager::ActorAnimationAt(
    Entity _id, std::chrono::steady_clock::duration _time) const
{
  AnimationUpdateData animData;
  auto trajIt = this->dataPtr->actorTrajectories.find(_id);
  if (trajIt == this->dataPtr->actorTrajectories.end())
    return animData;

  auto skelIt = this->dataPtr->actorSkeletons.find(_id);
  if (skelIt == this->dataPtr->actorSkeletons.end())
    return animData;

  animData = this->dataPtr->ActorTrajectoryAt(_id, _time);

  auto skel = skelIt->second;
  unsigned int animIndex = animData.trajectory.AnimIndex();
  animData.animationName = skel->Animation(animIndex)->Name();

  std::string rootNodeName = skel->RootNode()->Name();
  double distance = animData.trajectory.DistanceSoFar(animData.time);
  if (animData.followTrajectory)
  {
    math::Matrix4d rawFrame;
    if (animData.trajectory.Waypoints()->InterpolateX() &&
        !math::equal(distance, 0.0))
    {
      // logic here is mostly taken from
      // common::SkeletonAnimation::PoseAtX
      // We should consider refactoring part of that function to return
      // PoseAtX for only one skeleton node in addition to the current
      // function that computes PoseAtX for all skeleton nodes
      common::NodeAnimation *rootNode =
          skel->Animation(animIndex)->NodeAnimationByName(rootNodeName);
      math::Matrix4d lastPos = rootNode->KeyFrame(
          rootNode->FrameCount() - 1).second;
      math::Matrix4d firstPos = rootNode->KeyFrame(0).second;
      double x = distance;
      if (x < firstPos.Translation().X())
        x = firstPos.Translation().X();
      double lastX = lastPos.Translation().X();
      if (x > lastX && !animData.loop)
        x = lastX;
      while (x > lastX)
        x -= lastX;

      // update animation timepoint for root node
      // this should be the time that is used in the
      // SkeletonAnimationEnabled call
      double time = rootNode->TimeAtX(x);

      // get raw skeleton transform for root node. Needed to keep skeleton
      // animation in sync with trajectory animation
      rawFrame = rootNode->FrameAt(time, animData.loop);
      animData.time = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(time));
    }
    else
    {
      double timeSeconds = std::chrono::duration<double>(animData.time).count();
      rawFrame = skel->Animation(animIndex)->NodePoseAt(
          rootNodeName, timeSeconds, animData.loop);
    }

    math::Matrix4d skinTf = skel->AlignTranslation(animIndex, rootNodeName)
            * rawFrame * skel->AlignRotation(animIndex, rootNodeName);

    // zero out translation since we only need rotation to sync with actor
    // trajectory animation
    skinTf.SetTranslation(math::Vector3d::Zero);
    animData.rootTransform = skinTf;
  }

  return animData;
}

/////////////////////////////////////////////////
std::map<std::string, math::Matrix4d> SceneManager::ActorMeshAnimationAt(
    Entity _id, std::chrono::steady_clock::duration _time) const
{
  return this->ActorSkeletonTransformsAt(_id, _time);
}

/////////////////////////////////////////////////
std::map<std::string, math::Matrix4d> SceneManager::ActorSkeletonTransformsAt(
    Entity _id, std::chrono::steady_clock::duration _time) const
{
  std::map<std::string, math::Matrix4d> allFrames;

  auto trajIt = this->dataPtr->actorTrajectories.find(_id);
  if (trajIt == this->dataPtr->actorTrajectories.end())
  {
    return allFrames;
  }

  // get the trajectory at input time
  AnimationUpdateData animData = this->dataPtr->ActorTrajectoryAt(_id, _time);
  bool followTraj = animData.followTrajectory;
  bool noLoop = animData.loop;
  common::TrajectoryInfo traj = animData.trajectory;
  auto time = animData.time;

  common::PoseKeyFrame poseFrame(0.0);
  if (followTraj)
    traj.Waypoints()->InterpolatedKeyFrame(poseFrame);
  math::Matrix4d rootTf(poseFrame.Rotation());
  rootTf.SetTranslation(poseFrame.Translation());

  auto vIt = this->dataPtr->actorSkeletons.find(_id);
  if (vIt != this->dataPtr->actorSkeletons.end())
  {
    auto skel = vIt->second;
    unsigned int animIndex = traj.AnimIndex();
    std::map<std::string, math::Matrix4d> rawFrames;

    double timeSeconds = std::chrono::duration<double>(time).count();

    if (followTraj)
    {
      double distance = traj.DistanceSoFar(time);
      // check interpolate x.
      // todo(anyone) there is a problem with PoseAtX that causes
      // it to go into an infinite loop if the animation has no x displacement
      // e.g. a person standing that does not move in x direction
      if (traj.Waypoints()->InterpolateX() && !math::equal(distance, 0.0))
      {
        rawFrames = skel->Animation(animIndex)->PoseAtX(distance,
                                        skel->RootNode()->Name());
      }
      else
      {
        rawFrames = skel->Animation(animIndex)->PoseAt(timeSeconds, !noLoop);
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
  if (!this->dataPtr->scene)
    return;

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
    const rendering::VisualPtr &_visual) const
{
  auto node = this->TopLevelNode(_visual);
  return std::dynamic_pointer_cast<rendering::Visual>(node);
}

/////////////////////////////////////////////////
rendering::NodePtr SceneManager::TopLevelNode(
    const rendering::NodePtr &_node) const
{
  if (!this->dataPtr->scene)
    return rendering::NodePtr();

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

/////////////////////////////////////////////////
AnimationUpdateData SceneManagerPrivate::ActorTrajectoryAt(
    Entity _id,
    const std::chrono::steady_clock::duration &_time) const
{
  AnimationUpdateData animData;
  auto trajIt = this->actorTrajectories.find(_id);
  if (trajIt == this->actorTrajectories.end())
    return animData;

  auto trajs = trajIt->second;
  bool followTraj = true;
  if (1 == trajs.size() && nullptr == trajs[0].Waypoints())
    followTraj = false;

  auto firstTraj = trajs.begin();
  common::TrajectoryInfo traj = trajs[0];
  auto totalTime = trajs.rbegin()->EndTime() - trajs.begin()->StartTime();

  // delay start
  auto time = _time;
  if (time < (trajs.begin()->StartTime() - TP(0ms)))
  {
    time = std::chrono::steady_clock::duration(0);
  }
  else
  {
    time -= trajs.begin()->StartTime() - TP(0ms);
  }

  bool noLoop = trajs.rbegin()->StartTime() != TP(0ms)
            && trajs.rbegin()->StartTime() == trajs.rbegin()->EndTime();

  if (time >= totalTime && noLoop)
  {
    time = totalTime;
  }

  if (!noLoop || time <= totalTime)
  {
    while (time > totalTime)
    {
      time = time - totalTime;
    }
    if (followTraj)
    {
      for (auto &trajectory : trajs)
      {
        if (trajectory.StartTime() - firstTraj->StartTime() <= time
            && trajectory.EndTime() - firstTraj->StartTime() >= time)
        {
          traj = trajectory;
          time -= traj.StartTime() - firstTraj->StartTime();

          traj.Waypoints()->Time(std::chrono::duration<double>(time).count());
          break;
        }
      }
    }
  }

  // return time;
  animData.time = time;
  animData.loop = !noLoop;
  animData.followTrajectory = followTraj;
  animData.trajectory = traj;
  animData.valid = true;
  return animData;
}
