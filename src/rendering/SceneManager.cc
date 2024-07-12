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
#include <memory>
#include <queue>
#include <unordered_map>

#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Polyline.hh>
#include <sdf/Sphere.hh>

#include <gz/common/Animation.hh>
#include <gz/common/Console.hh>
#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/common/KeyFrame.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Skeleton.hh>
#include <gz/common/SkeletonAnimation.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/Uuid.hh>

#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Inertial.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/Utility.hh>

#include "gz/rendering/Capsule.hh"
#include <gz/rendering/COMVisual.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Heightmap.hh>
#include <gz/rendering/HeightmapDescriptor.hh>
#include <gz/rendering/InertiaVisual.hh>
#include <gz/rendering/JointVisual.hh>
#include <gz/rendering/Light.hh>
#include <gz/rendering/LightVisual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/ParticleEmitter.hh>
#include <gz/rendering/Projector.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/WireBox.hh>

#include "gz/sim/Conversions.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/rendering/SceneManager.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

using TP = std::chrono::steady_clock::time_point;

/// \brief Private data class.
class gz::sim::SceneManagerPrivate
{
  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldId{0};

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Map of visual entity in Gazebo to visual pointers.
  public: std::unordered_map<Entity, rendering::VisualPtr> visuals;

  /// \brief Map of actor entity in Gazebo to actor pointers.
  public: std::unordered_map<Entity, rendering::MeshPtr> actors;

  /// \brief Map of actor entity in Gazebo to actor animations.
  public: std::unordered_map<Entity, common::SkeletonPtr> actorSkeletons;

  /// \brief Map of actor entity to the associated trajectories.
  public: std::unordered_map<Entity, std::vector<common::TrajectoryInfo>>
                    actorTrajectories;

  /// \brief Map of light entity in Gazebo to light pointers.
  public: std::unordered_map<Entity, rendering::LightPtr> lights;

  /// \brief Map of particle emitter entity in Gazebo to particle emitter
  /// rendering pointers.
  public: std::unordered_map<Entity, rendering::ParticleEmitterPtr>
      particleEmitters;

  /// \brief Map of projector entity in Gazebo to projector
  /// rendering pointers.
  public: std::unordered_map<Entity, rendering::ProjectorPtr> projectors;

  /// \brief Map of sensor entity in Gazebo to sensor pointers.
  public: std::unordered_map<Entity, rendering::SensorPtr> sensors;

  /// \brief The map of the original transparency values for the nodes.
  public: std::map<std::string, double> originalTransparency;

  /// \brief The map of the original depth write values for the nodes.
  public: std::map<std::string, bool> originalDepthWrite;

  /// \brief Helper function to compute actor trajectory at specified tiime
  /// \param[in] _id Actor entity's unique id
  /// \param[in] _time Simulation time
  /// \return AnimationUpdateData with trajectory related fields filled. It
  /// also sets the time point in which the animation should be played
  public: AnimationUpdateData ActorTrajectoryAt(
      Entity _id, const std::chrono::steady_clock::duration &_time) const;

  /// \brief Load Actor trajectories
  /// \param[in] _actor Actor
  /// \param[in] _mapAnimNameId  Animation name to id map
  /// \param[in] _skel Mesh skeleton
  /// \return Trajectory vector
  public: std::vector<common::TrajectoryInfo>
      LoadTrajectories(const sdf::Actor &_actor,
      std::unordered_map<std::string, unsigned int> &_mapAnimNameId,
      common::SkeletonPtr _skel);

  /// \brief Holds the spherical coordinates from the world.
  public: math::SphericalCoordinates sphericalCoordinates;
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
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
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
      gzerr << "Parent entity with Id: [" << _parentId << "] not found. "
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
    gzerr << "Visual: [" << name << "] already exists" << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr modelVis = this->dataPtr->scene->CreateVisual(name);

  modelVis->SetUserData("gazebo-entity", _id);
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
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
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
  linkVis->SetUserData("gazebo-entity", _id);
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
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
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
  visualVis->SetUserData("gazebo-entity", _id);
  visualVis->SetUserData("pause-update", static_cast<int>(0));
  visualVis->SetLocalPose(_visual.RawPose());

  if (_visual.HasLaserRetro())
  {
    visualVis->SetUserData("laser_retro", _visual.LaserRetro());
  }

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose;
  rendering::GeometryPtr geom =
      this->LoadGeometry(*_visual.Geom(), scale, localPose);

  if (geom)
  {
    /// localPose is currently used to handle the normal vector in plane visuals
    /// In general, this can be used to store any local transforms between the
    /// parent Visual and geometry.
    if (localPose != math::Pose3d::Zero)
    {
      rendering::VisualPtr geomVis =
          this->dataPtr->scene->CreateVisual(name + "_geom");
      geomVis->AddGeometry(geom);
      geomVis->SetLocalPose(localPose);
      visualVis->AddChild(geomVis);
    }
    else
    {
      visualVis->AddGeometry(geom);
    }

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
      material = this->dataPtr->scene->Material("gz-grey");
      if (!material)
      {
        material = this->dataPtr->scene->CreateMaterial("gz-grey");
        material->SetAmbient(0.3, 0.3, 0.3);
        material->SetDiffuse(0.7, 0.7, 0.7);
        material->SetSpecular(1.0, 1.0, 1.0);
        material->SetRoughness(0.2f);
        material->SetMetalness(1.0f);
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

          // unlike setting transparency above, the parent submesh are not
          // notified about the the cast shadows changes. So we need to set
          // the material back to the submesh.
          // \todo(anyone) find way to propate cast shadows changes tos submesh
          // in gz-rendering
          submeshMat->SetCastShadows(_visual.CastShadows());
          submesh->SetMaterial(submeshMat);
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
      // This is not ideal. We should let gz-rendering handle the lifetime
      // of this material
      this->dataPtr->scene->DestroyMaterial(material);
    }
  }
  else
  {
    gzerr << "Failed to load geometry for visual: " << _visual.Name()
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
std::vector<rendering::NodePtr> SceneManager::Filter(const std::string &_node,
    std::function<bool(const rendering::NodePtr _nodeToFilter)> _filter) const
{
  std::vector<rendering::NodePtr> filteredNodes;

  // make sure there is a rendering node named _node
  auto rootNode = this->dataPtr->scene->NodeByName(_node);
  if (!rootNode)
  {
    gzerr << "Could not find a node with the name [" << _node
           << "] in the scene." << std::endl;
    return filteredNodes;
  }

  // go through _node and its children in top level order, applying _filter to
  // each node
  std::queue<rendering::NodePtr> remainingNodes;
  remainingNodes.push(rootNode);
  while (!remainingNodes.empty())
  {
    auto currentNode = remainingNodes.front();
    remainingNodes.pop();
    if (_filter(currentNode))
      filteredNodes.push_back(currentNode);

    for (auto i = 0u; i < currentNode->ChildCount(); ++i)
      remainingNodes.push(currentNode->ChildByIndex(i));
  }

  return filteredNodes;
}

/////////////////////////////////////////////////
std::pair<rendering::VisualPtr, std::vector<Entity>> SceneManager::CopyVisual(
    Entity _id, const std::string &_visual, Entity _parentId)
{
  std::pair<rendering::VisualPtr, std::vector<Entity>> result;
  if (!this->dataPtr->scene)
    return result;

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return result;
  }

  rendering::VisualPtr originalVisual =
    std::dynamic_pointer_cast<rendering::Visual>(
        this->dataPtr->scene->NodeByName(_visual));
  if (!originalVisual)
  {
    gzerr << "Could not find a node with the name [" << _visual
           << "] in the scene." << std::endl;
    return result;
  }

  auto name = originalVisual->Name() + "::" + std::to_string(_id);

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      gzerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding visual with ID [" << _id
             << "] and name [" << name << "] to the rendering scene."
             << std::endl;
      return result;
    }
    parent = it->second;
  }

  if (parent)
    name = parent->Name() + "::" + name;

  if (this->dataPtr->scene->HasVisualName(name))
  {
    gzerr << "Visual: [" << name << "] already exists" << std::endl;
    return result;
  }

  // filter visuals that were created by the gui (these shouldn't be cloned)
  auto filteredVisuals = this->Filter(_visual,
      [](const rendering::NodePtr _node)
      {
        return _node->HasUserData("gui-only");
      });

  // temporarily detach filtered visuals, but keep track of the original parent
  // so that the visuals can be re-attached later
  std::unordered_map<rendering::NodePtr, rendering::NodePtr>
    removedVisualToParent;
  for (auto filteredVis : filteredVisuals)
  {
    removedVisualToParent[filteredVis] = filteredVis->Parent();
    filteredVis->RemoveParent();
  }

  // clone the visual
  auto clonedVisual = originalVisual->Clone(name, parent);
  this->dataPtr->visuals[_id] = clonedVisual;

  // re-attach filtered visuals now that cloning is complete
  for (auto &[removedVisual, originalParent] : removedVisualToParent)
    originalParent->AddChild(removedVisual);

  // The Clone call above also clones any child visuals that exist, so we need
  // to keep track of these new child visuals as well. We get a level order
  // listing of all visuals associated with the newly copied visual
  bool childrenTracked = true;
  std::queue<Entity> remainingVisuals;
  remainingVisuals.push(_id);
  std::vector<Entity> childVisualIds;
  while (!remainingVisuals.empty())
  {
    const auto topLevelId = remainingVisuals.front();
    remainingVisuals.pop();
    const auto visual = this->dataPtr->visuals[topLevelId];
    for (auto i = 0u; i < visual->ChildCount(); ++i)
    {
      auto childId = this->UniqueId();
      if (!childId)
      {
        gzerr << "Unable to create an entity ID for the copied visual's "
               << "child, so the copied visual will be deleted.\n";
        childrenTracked = false;
        break;
      }
      auto childVisual = std::dynamic_pointer_cast<rendering::Visual>(
          visual->ChildByIndex(i));
      if (!childVisual)
      {
        gzerr << "Unable to retrieve a child visual of the copied visual, "
               << "so the copied visual will be deleted.\n";
        childrenTracked = false;
        break;
      }

      this->dataPtr->visuals[childId] = childVisual;
      childVisual->SetUserData("gazebo-entity", childId);
      childVisual->SetUserData("pause-update", static_cast<int>(0));
      childVisualIds.push_back(childId);

      remainingVisuals.push(childId);
    }
  }

  if (!childrenTracked)
  {
    this->dataPtr->scene->DestroyVisual(clonedVisual, true);
    for (const auto id : childVisualIds)
      this->dataPtr->visuals.erase(id);
  }
  else
  {
    clonedVisual->SetUserData("gazebo-entity", _id);
    clonedVisual->SetUserData("pause-update", static_cast<int>(0));

    result = {clonedVisual, std::move(childVisualIds)};

    if (!parent)
      this->dataPtr->scene->RootVisual()->AddChild(clonedVisual);
  }

  return result;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::VisualById(Entity _id)
{
  if (this->dataPtr->visuals.find(_id) == this->dataPtr->visuals.end())
  {
    gzerr << "Could not find visual for entity: " << _id << std::endl;
    return nullptr;
  }
  return this->dataPtr->visuals[_id];
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateCollision(Entity _id,
    const sdf::Collision &_collision, Entity _parentId)
{
  sdf::Material material;
  material.SetAmbient(math::Color(1.0f, 0.5088f, 0.0468f, 0.7f));
  material.SetDiffuse(math::Color(1.0f, 0.5088f, 0.0468f, 0.7f));

  sdf::Visual visual;
  visual.SetGeom(*_collision.Geom());
  visual.SetMaterial(material);
  visual.SetCastShadows(false);

  visual.SetRawPose(_collision.RawPose());
  visual.SetName(_collision.Name());

  rendering::VisualPtr collisionVis = CreateVisual(_id, visual, _parentId);
  collisionVis->SetUserData("gui-only", static_cast<bool>(true));
  return collisionVis;
}

/////////////////////////////////////////////////
void SceneManager::SetSphericalCoordinates(
    const math::SphericalCoordinates &_sphericalCoordinates)
{
  this->dataPtr->sphericalCoordinates = _sphericalCoordinates;
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
  else if (_geom.Type() == sdf::GeometryType::CAPSULE)
  {
    auto capsule = this->dataPtr->scene->CreateCapsule();
    capsule->SetRadius(_geom.CapsuleShape()->Radius());
    capsule->SetLength(_geom.CapsuleShape()->Length());
    geom = capsule;
  }
  else if (_geom.Type() == sdf::GeometryType::CONE)
  {
    geom = this->dataPtr->scene->CreateCone();
    scale.X() = _geom.ConeShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.ConeShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::CYLINDER)
  {
    geom = this->dataPtr->scene->CreateCylinder();
    scale.X() = _geom.CylinderShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.CylinderShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::ELLIPSOID)
  {
    geom = this->dataPtr->scene->CreateSphere();
    scale.X() = _geom.EllipsoidShape()->Radii().X() * 2;
    scale.Y() = _geom.EllipsoidShape()->Radii().Y() * 2;
    scale.Z() = _geom.EllipsoidShape()->Radii().Z() * 2;
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
    localPose.Rot().SetFrom2Axes(math::Vector3d::UnitZ, normal.Normalized());
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
    rendering::MeshDescriptor descriptor;
    descriptor.mesh = loadMesh(*_geom.MeshShape());
    if (!descriptor.mesh)
      return geom;
    std::string meshUri =
        (common::URI(_geom.MeshShape()->Uri()).Scheme() == "name") ?
         common::basename(_geom.MeshShape()->Uri()) :
         asFullPath(_geom.MeshShape()->Uri(),
                    _geom.MeshShape()->FilePath());
    auto a =     asFullPath(_geom.MeshShape()->Uri(),
                    _geom.MeshShape()->FilePath());

    descriptor.meshName = meshUri;
    descriptor.subMeshName = _geom.MeshShape()->Submesh();
    descriptor.centerSubMesh = _geom.MeshShape()->CenterSubmesh();

    geom = this->dataPtr->scene->CreateMesh(descriptor);
    scale = _geom.MeshShape()->Scale();
  }
  else if (_geom.Type() == sdf::GeometryType::HEIGHTMAP)
  {
    auto fullPath = common::findFile(asFullPath(_geom.HeightmapShape()->Uri(),
        _geom.HeightmapShape()->FilePath()));
    if (fullPath.empty())
    {
      gzerr << "Heightmap geometry missing URI" << std::endl;
      return geom;
    }


    std::shared_ptr<common::HeightmapData> data;
    std::string lowerFullPath = common::lowercase(fullPath);
    // check if heightmap is an image
    if (common::EndsWith(lowerFullPath, ".png")
        || common::EndsWith(lowerFullPath, ".jpg")
        || common::EndsWith(lowerFullPath, ".jpeg"))
    {
      auto img = std::make_shared<common::ImageHeightmap>();
      if (img->Load(fullPath) < 0)
      {
        gzerr << "Failed to load heightmap image data from ["
               << fullPath << "]" << std::endl;
        return geom;
      }
      data = img;
    }
    // DEM
    else
    {
      auto dem = std::make_shared<common::Dem>();
      dem->SetSphericalCoordinates(this->dataPtr->sphericalCoordinates);
      if (dem->Load(fullPath) < 0)
      {
        gzerr << "Failed to load heightmap dem data from ["
               << fullPath << "]" << std::endl;
        return geom;
      }
      data = dem;
    }

    rendering::HeightmapDescriptor descriptor;
    descriptor.SetData(data);
    descriptor.SetSize(_geom.HeightmapShape()->Size());
    descriptor.SetPosition(_geom.HeightmapShape()->Position());
    descriptor.SetSampling(_geom.HeightmapShape()->Sampling());

    for (uint64_t i = 0; i < _geom.HeightmapShape()->TextureCount(); ++i)
    {
      auto textureSdf = _geom.HeightmapShape()->TextureByIndex(i);
      rendering::HeightmapTexture textureDesc;
      textureDesc.SetSize(textureSdf->Size());
      textureDesc.SetDiffuse(common::findFile(asFullPath(textureSdf->Diffuse(),
          _geom.HeightmapShape()->FilePath())));
      textureDesc.SetNormal(common::findFile(asFullPath(textureSdf->Normal(),
          _geom.HeightmapShape()->FilePath())));
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
      gzerr << "Failed to create heightmap [" << fullPath << "]" << std::endl;
    }
    scale = _geom.HeightmapShape()->Size();
  }
  else if (_geom.Type() == sdf::GeometryType::POLYLINE)
  {
    std::vector<std::vector<math::Vector2d>> vertices;
    for (const auto &polyline : _geom.PolylineShape())
    {
      vertices.push_back(polyline.Points());
    }

    std::string name("POLYLINE_" + common::Uuid().String());

    auto meshManager = common::MeshManager::Instance();
    meshManager->CreateExtrudedPolyline(name, vertices,
        _geom.PolylineShape()[0].Height());

    rendering::MeshDescriptor descriptor;
    descriptor.meshName = name;
    descriptor.mesh = meshManager->MeshByName(name);
    if (descriptor.mesh)
    {
      geom = this->dataPtr->scene->CreateMesh(descriptor);
    }
    else
    {
      gzerr << "Unable to find the polyline mesh: " << name << std::endl;
    }
  }
  else
  {
    gzerr << "Unsupported geometry type" << std::endl;
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
  material->SetShininess(_material.Shininess());
  material->SetEmissive(_material.Emissive());
  material->SetRenderOrder(_material.RenderOrder());

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
        std::string fullPath = common::findFile(
            asFullPath(roughnessMap, _material.FilePath()));
        if (!fullPath.empty())
          material->SetRoughnessMap(fullPath);
        else
          gzerr << "Unable to find file [" << roughnessMap << "]\n";
      }

      // metalness map
      std::string metalnessMap = metal->MetalnessMap();
      if (!metalnessMap.empty())
      {
        std::string fullPath = common::findFile(
            asFullPath(metalnessMap, _material.FilePath()));
        if (!fullPath.empty())
          material->SetMetalnessMap(fullPath);
        else
          gzerr << "Unable to find file [" << metalnessMap << "]\n";
      }
      workflow = const_cast<sdf::PbrWorkflow *>(metal);
    }
    else
    {
      auto specular = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
      if (specular)
      {
        gzerr << "PBR material: currently only metal workflow is supported. "
               << "Gazebo will try to render the material using "
               << "metal workflow but without Roughness / Metalness settings."
               << std::endl;
      }
      workflow = const_cast<sdf::PbrWorkflow *>(specular);
    }

    if (!workflow)
    {
      gzerr << "No valid PBR workflow found. " << std::endl;
      return rendering::MaterialPtr();
    }

    // albedo map
    std::string albedoMap = workflow->AlbedoMap();
    if (!albedoMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(albedoMap, _material.FilePath()));
      if (!fullPath.empty())
      {
        material->SetTexture(fullPath);
        // Use alpha channel for transparency
        material->SetAlphaFromTexture(true, 0.5, _material.DoubleSided());
      }
      else
        gzerr << "Unable to find file [" << albedoMap << "]\n";
    }

    // normal map
    std::string normalMap = workflow->NormalMap();
    if (!normalMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(normalMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetNormalMap(fullPath);
      else
        gzerr << "Unable to find file [" << normalMap << "]\n";
    }


    // environment map
    std::string environmentMap = workflow->EnvironmentMap();
    if (!environmentMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(environmentMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetEnvironmentMap(fullPath);
      else
        gzerr << "Unable to find file [" << environmentMap << "]\n";
    }

    // emissive map
    std::string emissiveMap = workflow->EmissiveMap();
    if (!emissiveMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(emissiveMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetEmissiveMap(fullPath);
      else
        gzerr << "Unable to find file [" << emissiveMap << "]\n";
    }

    // light map
    std::string lightMap = workflow->LightMap();
    if (!lightMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(lightMap, _material.FilePath()));
      if (!fullPath.empty())
      {
        unsigned int uvSet = workflow->LightMapTexCoordSet();
        material->SetLightMap(fullPath, uvSet);
      }
      else
      {
        gzerr << "Unable to find file [" << lightMap << "]\n";
      }
    }
  }
  return material;
}

/////////////////////////////////////////////////
void SceneManager::SequenceTrajectories(
    std::vector<common::TrajectoryInfo>& _trajectories,
    TP _time)
{
  // sequencing all trajectories
  for (auto &trajectory : _trajectories)
  {
    auto duration = trajectory.Duration();
    trajectory.SetStartTime(_time);
    _time += duration;
    trajectory.SetEndTime(_time);
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateActor(Entity _id,
    const sdf::Actor &_actor, const std::string &_name, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

  // creating an actor needs to create the visual and the mesh
  // the visual is stored in: this->dataPtr->visuals
  // the mesh is stored in: this->dataPtr->actors

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      gzerr << "Parent entity with Id: [" << _parentId << "] not found. "
             << "Not adding actor with ID [" << _id
             << "] and name [" << _name << "] to the rendering scene."
             << std::endl;
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  std::string name = _name;
  if (parent)
    name = parent->Name() +  "::" + name;

  rendering::MeshDescriptor descriptor;
  descriptor.meshName = asFullPath(_actor.SkinFilename(), _actor.FilePath());
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  std::unordered_map<std::string, unsigned int> mapAnimNameId;
  common::SkeletonPtr meshSkel;
  if (nullptr == descriptor.mesh)
  {
    gzwarn << "Actor skin mesh [" << descriptor.meshName << "] not found."
           << std::endl;
  }
  else
  {
    // todo(anyone) create a copy of meshSkel so we don't modify the original
    // when adding animations!
    meshSkel = descriptor.mesh->MeshSkeleton();
    if (nullptr == meshSkel)
    {
      gzwarn << "Mesh skeleton in [" << descriptor.meshName << "] not found."
             << std::endl;
    }
    else
    {
      this->dataPtr->actorSkeletons[_id] = meshSkel;
      // Load all animations
      mapAnimNameId = this->LoadAnimations(_actor);
      if (mapAnimNameId.size() == 0)
        return nullptr;
    }
  }

  // load trajectories regardless of whether the mesh skeleton exists
  // or not. If there is no mesh skeleon, we can still do trajectory
  // animation
  auto trajectories = this->dataPtr->LoadTrajectories(_actor, mapAnimNameId,
                      meshSkel);

  // sequencing all trajectories
  auto delayStartTime = std::chrono::milliseconds(
              static_cast<int>(_actor.ScriptDelayStart() * 1000));
  TP time(delayStartTime);
  this->SequenceTrajectories(trajectories, time);


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

  rendering::VisualPtr actorVisual = this->dataPtr->scene->CreateVisual(name);
  rendering::MeshPtr actorMesh;
  // create mesh with animations
  if (descriptor.mesh)
  {
    actorMesh = this->dataPtr->scene->CreateMesh(descriptor);
    if (nullptr == actorMesh)
    {
      gzerr << "Actor skin file [" << descriptor.meshName << "] not found."
             << std::endl;
      return rendering::VisualPtr();
    }
    actorVisual->AddGeometry(actorMesh);
  }

  actorVisual->SetLocalPose(_actor.RawPose());
  actorVisual->SetUserData("gazebo-entity", _id);
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
rendering::VisualPtr SceneManager::CreateLightVisual(Entity _id,
    const sdf::Light &_light, const std::string &_name, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::LightPtr lightParent;
  auto it = this->dataPtr->lights.find(_parentId);
  if (it != this->dataPtr->lights.end())
  {
    lightParent = it->second;
  }
  else
  {
    gzerr << "Parent entity with Id: [" << _parentId << "] not found. "
           << "Not adding light visual with ID [" << _id
           << "] and name [" << _name << "] to the rendering scene."
           << std::endl;
    return rendering::VisualPtr();
  }

  std::string name = lightParent->Name() +  "::" + _name + "Visual";

  if (this->dataPtr->scene->HasVisualName(name))
  {
    gzerr << "Visual: [" << name << "] already exists" << std::endl;
    return rendering::VisualPtr();
  }

  rendering::LightVisualPtr lightVisual =
    this->dataPtr->scene->CreateLightVisual(name);
  if (_light.Type() == sdf::LightType::POINT)
  {
    lightVisual->SetType(rendering::LightVisualType::LVT_POINT);
  }
  else if (_light.Type() == sdf::LightType::DIRECTIONAL)
  {
    lightVisual->SetType(rendering::LightVisualType::LVT_DIRECTIONAL);
  }
  else if (_light.Type() == sdf::LightType::SPOT)
  {
    lightVisual->SetType(rendering::LightVisualType::LVT_SPOT);
    lightVisual->SetInnerAngle(_light.SpotInnerAngle().Radian());
    lightVisual->SetOuterAngle(_light.SpotOuterAngle().Radian());
  }

  lightVisual->SetVisible(_light.Visualize());

  rendering::VisualPtr lightVis = std::dynamic_pointer_cast<rendering::Visual>(
    lightVisual);
  lightVis->SetUserData("gazebo-entity", _id);
  lightVis->SetUserData("pause-update", static_cast<int>(0));
  this->dataPtr->visuals[_id] = lightVis;

  if (lightParent)
  {
    lightVis->RemoveParent();
    lightParent->AddChild(lightVis);
  }
  return lightVis;
}

/////////////////////////////////////////////////
rendering::LightPtr SceneManager::CreateLight(Entity _id,
    const sdf::Light &_light, const std::string &_name, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::LightPtr();

  if (this->HasEntity(_id))
  {
    gzerr << "Light with Id: [" << _id << "] can not be create there is "
              "another entity with the same entity number" << std::endl;
    return nullptr;
  }

  if (this->dataPtr->lights.find(_id) != this->dataPtr->lights.end())
  {
    gzerr << "Light with Id: [" << _id << "] already exists in the scene"
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

  std::string name = _name;
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
      spotLight->SetDirection(_light.Direction());
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
      gzerr << "Light type not supported" << std::endl;
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
  light->SetIntensity(_light.Intensity());

  this->dataPtr->lights[_id] = light;

  if (parent)
    parent->AddChild(light);
  else
    this->dataPtr->scene->RootVisual()->AddChild(light);

  return light;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateInertiaVisual(Entity _id,
    const math::Inertiald &_inertia, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
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

  std::string name = std::to_string(_id);
  if (parent)
    name = parent->Name() + "::" + name;

  rendering::InertiaVisualPtr inertiaVisual =
    this->dataPtr->scene->CreateInertiaVisual(name);
  inertiaVisual->SetInertial(_inertia);

  rendering::VisualPtr inertiaVis =
    std::dynamic_pointer_cast<rendering::Visual>(inertiaVisual);
  inertiaVis->SetUserData("gazebo-entity", _id);
  inertiaVis->SetUserData("pause-update", static_cast<int>(0));
  inertiaVis->SetUserData("gui-only", static_cast<bool>(true));
  this->dataPtr->visuals[_id] = inertiaVis;

  if (parent)
  {
    inertiaVis->RemoveParent();
    parent->AddChild(inertiaVis);
  }
  return inertiaVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateJointVisual(
    Entity _id, const sdf::Joint &_joint,
    Entity _childId, Entity _parentId)
{
  if (!this->dataPtr->scene)
  {
    return rendering::VisualPtr();
  }

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
           << std::endl;
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_childId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_childId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return rendering::VisualPtr();
    }
    parent = it->second;
  }

  // Name.
  std::string name = _joint.Name().empty() ? std::to_string(_id) :
    _joint.Name();
  if (parent)
  {
    name = parent->Name() +  "::" + name;
  }

  rendering::JointVisualPtr jointVisual =
    this->dataPtr->scene->CreateJointVisual(name);

  switch (_joint.Type())
  {
    case sdf::JointType::REVOLUTE:
      jointVisual->SetType(rendering::JointVisualType::JVT_REVOLUTE);
      break;
    case sdf::JointType::REVOLUTE2:
      jointVisual->SetType(rendering::JointVisualType::JVT_REVOLUTE2);
      break;
    case sdf::JointType::PRISMATIC:
      jointVisual->SetType(rendering::JointVisualType::JVT_PRISMATIC);
      break;
    case sdf::JointType::UNIVERSAL:
      jointVisual->SetType(rendering::JointVisualType::JVT_UNIVERSAL);
      break;
    case sdf::JointType::BALL:
      jointVisual->SetType(rendering::JointVisualType::JVT_BALL);
      break;
    case sdf::JointType::SCREW:
      jointVisual->SetType(rendering::JointVisualType::JVT_SCREW);
      break;
    case sdf::JointType::GEARBOX:
      jointVisual->SetType(rendering::JointVisualType::JVT_GEARBOX);
      break;
    case sdf::JointType::FIXED:
      jointVisual->SetType(rendering::JointVisualType::JVT_FIXED);
      break;
    default:
      jointVisual->SetType(rendering::JointVisualType::JVT_NONE);
      break;
  }

  if (parent)
  {
    jointVisual->RemoveParent();
    parent->AddChild(jointVisual);
  }

  if (_joint.Axis(1) &&
      (_joint.Type() == sdf::JointType::REVOLUTE2 ||
       _joint.Type() == sdf::JointType::UNIVERSAL
      ))
  {
    auto axis1 = _joint.Axis(0)->Xyz();
    auto axis2 = _joint.Axis(1)->Xyz();
    auto axis1UseParentFrame = _joint.Axis(0)->XyzExpressedIn() == "__model__";
    auto axis2UseParentFrame = _joint.Axis(1)->XyzExpressedIn() == "__model__";

    jointVisual->SetAxis(axis2, axis2UseParentFrame);

    auto it = this->dataPtr->visuals.find(_parentId);
    if (it != this->dataPtr->visuals.end())
    {
      auto parentName = it->second->Name();
      jointVisual->SetParentAxis(
          axis1, parentName, axis1UseParentFrame);
    }
  }
  else if (_joint.Axis(0) &&
      (_joint.Type() == sdf::JointType::REVOLUTE ||
       _joint.Type() == sdf::JointType::PRISMATIC
      ))
  {
    auto axis1 = _joint.Axis(0)->Xyz();
    auto axis1UseParentFrame = _joint.Axis(0)->XyzExpressedIn() == "__model__";

    jointVisual->SetAxis(axis1, axis1UseParentFrame);
  }
  else
  {
    // For fixed joint type, scale joint visual to the joint child link
    double childSize =
        std::max(0.1, parent->BoundingBox().Size().Length());
    auto scale = gz::math::Vector3d(childSize * 0.2,
        childSize * 0.2, childSize * 0.2);
    jointVisual->SetLocalScale(scale);
  }

  rendering::VisualPtr jointVis =
    std::dynamic_pointer_cast<rendering::Visual>(jointVisual);
  jointVis->SetUserData("gazebo-entity", _id);
  jointVis->SetUserData("pause-update", static_cast<int>(0));
  jointVis->SetUserData("gui-only", static_cast<bool>(true));
  jointVis->SetLocalPose(_joint.RawPose());
  this->dataPtr->visuals[_id] = jointVis;
  return jointVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr SceneManager::CreateCOMVisual(Entity _id,
    const math::Inertiald &_inertia, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::VisualPtr();

  if (this->dataPtr->visuals.find(_id) != this->dataPtr->visuals.end())
  {
    gzerr << "Entity with Id: [" << _id << "] already exists in the scene"
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

  if (!parent)
    return rendering::VisualPtr();

  std::string name = std::to_string(_id);
  name = parent->Name() + "::" + name;

  rendering::COMVisualPtr comVisual =
      this->dataPtr->scene->CreateCOMVisual(name);
  comVisual->RemoveParent();
  parent->AddChild(comVisual);
  comVisual->SetInertial(_inertia);

  rendering::VisualPtr comVis =
    std::dynamic_pointer_cast<rendering::Visual>(comVisual);
  comVis->SetUserData("gazebo-entity", _id);
  comVis->SetUserData("pause-update", static_cast<int>(0));
  comVis->SetUserData("gui-only", static_cast<bool>(true));
  this->dataPtr->visuals[_id] = comVis;

  return comVis;
}

/////////////////////////////////////////////////
rendering::ParticleEmitterPtr SceneManager::CreateParticleEmitter(
    Entity _id, const msgs::ParticleEmitter &_emitter, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::ParticleEmitterPtr();

  if (this->dataPtr->particleEmitters.find(_id) !=
     this->dataPtr->particleEmitters.end())
  {
    gzerr << "Particle emitter with Id: [" << _id << "] already exists in the "
           <<" scene" << std::endl;
    return rendering::ParticleEmitterPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return rendering::ParticleEmitterPtr();
    }
    parent = it->second;
  }

  // Name.
  std::string name = _emitter.name().empty() ? std::to_string(_id) :
    _emitter.name();
  if (parent)
    name = parent->Name() +  "::" + name;

  rendering::ParticleEmitterPtr emitter;
  emitter = this->dataPtr->scene->CreateParticleEmitter(name);

  this->dataPtr->particleEmitters[_id] = emitter;

  if (parent)
    parent->AddChild(emitter);

  this->UpdateParticleEmitter(_id, _emitter);

  return emitter;
}

/////////////////////////////////////////////////
rendering::ParticleEmitterPtr SceneManager::UpdateParticleEmitter(Entity _id,
    const msgs::ParticleEmitter &_emitter)
{
  if (!this->dataPtr->scene)
    return rendering::ParticleEmitterPtr();

  // Sanity check: Make sure that the id exists.
  auto emitterIt = this->dataPtr->particleEmitters.find(_id);
  if (emitterIt == this->dataPtr->particleEmitters.end())
  {
    gzerr << "Particle emitter with Id: [" << _id << "] not found in the "
           << "scene" << std::endl;
    return rendering::ParticleEmitterPtr();
  }
  auto emitter = emitterIt->second;

  // Type.
  switch (_emitter.type())
  {
    case gz::msgs::ParticleEmitter_EmitterType_BOX:
    {
      emitter->SetType(gz::rendering::EmitterType::EM_BOX);
      break;
    }
    case gz::msgs::ParticleEmitter_EmitterType_CYLINDER:
    {
      emitter->SetType(gz::rendering::EmitterType::EM_CYLINDER);
      break;
    }
    case gz::msgs::ParticleEmitter_EmitterType_ELLIPSOID:
    {
      emitter->SetType(gz::rendering::EmitterType::EM_ELLIPSOID);
      break;
    }
    default:
    {
      // Do nothing if type is not set.
    }
  }

  // Emitter size.
  if (_emitter.has_size())
    emitter->SetEmitterSize(gz::msgs::Convert(_emitter.size()));

  // Rate.
  if (_emitter.has_rate())
    emitter->SetRate(_emitter.rate().data());

  // Duration.
  if (_emitter.has_duration())
    emitter->SetDuration(_emitter.duration().data());

  // Emitting.
  if (_emitter.has_emitting()) {
    emitter->SetEmitting(_emitter.emitting().data());
  }

  // Particle size.
  if (_emitter.has_particle_size())
  {
    emitter->SetParticleSize(
        gz::msgs::Convert(_emitter.particle_size()));
  }

  // Lifetime.
  if (_emitter.has_lifetime())
    emitter->SetLifetime(_emitter.lifetime().data());

  // Material.
  if (_emitter.has_material())
  {
    gz::rendering::MaterialPtr material =
      this->LoadMaterial(convert<sdf::Material>(_emitter.material()));
    emitter->SetMaterial(material);
  }

  // Velocity range.
  if (_emitter.has_min_velocity() && _emitter.has_max_velocity())
  {
    emitter->SetVelocityRange(_emitter.min_velocity().data(),
        _emitter.max_velocity().data());
  }

  // Color range image.
  if (_emitter.has_color_range_image() &&
      !_emitter.color_range_image().data().empty())
  {
    emitter->SetColorRangeImage(_emitter.color_range_image().data());
  }
  // Color range.
  else if (_emitter.has_color_start() && _emitter.has_color_end())
  {
    emitter->SetColorRange(
      gz::msgs::Convert(_emitter.color_start()),
      gz::msgs::Convert(_emitter.color_end()));
  }

  // Scale rate.
  if (_emitter.has_scale_rate())
    emitter->SetScaleRate(_emitter.scale_rate().data());

  // pose
  if (_emitter.has_pose())
    emitter->SetLocalPose(msgs::Convert(_emitter.pose()));

  // particle scatter ratio
  if (_emitter.has_header())
  {
    for (int i = 0; i < _emitter.header().data_size(); ++i)
    {
      const auto &data = _emitter.header().data(i);
      const std::string key = "particle_scatter_ratio";
      if (data.key() == "particle_scatter_ratio" && data.value_size() > 0)
      {
        emitter->SetParticleScatterRatio(math::parseFloat(data.value(0)));
        break;
      }
    }
  }

  return emitter;
}

/////////////////////////////////////////////////
rendering::ProjectorPtr SceneManager::CreateProjector(
    Entity _id, const sdf::Projector &_projector, Entity _parentId)
{
  if (!this->dataPtr->scene)
    return rendering::ProjectorPtr();

  if (this->dataPtr->projectors.find(_id) !=
     this->dataPtr->projectors.end())
  {
    gzerr << "Projector with Id: [" << _id << "] already exists in the "
          << "scene" << std::endl;
    return rendering::ProjectorPtr();
  }

  rendering::VisualPtr parent;
  if (_parentId != this->dataPtr->worldId)
  {
    auto it = this->dataPtr->visuals.find(_parentId);
    if (it == this->dataPtr->visuals.end())
    {
      // It is possible to get here if the model entity is created then
      // removed in between render updates.
      return rendering::ProjectorPtr();
    }
    parent = it->second;
  }

  // Name.
  std::string name = _projector.Name().empty() ? std::to_string(_id) :
    _projector.Name();
  if (parent)
    name = parent->Name() +  "::" + name;

  rendering::ProjectorPtr projector;
  projector = this->dataPtr->scene->CreateProjector(name);

  this->dataPtr->projectors[_id] = projector;

  if (parent)
    parent->AddChild(projector);

  projector->SetLocalPose(_projector.RawPose());
  projector->SetNearClipPlane(_projector.NearClip());
  projector->SetFarClipPlane(_projector.FarClip());
  std::string texture = _projector.Texture();
  std::string fullPath = common::findFile(
      asFullPath(texture, _projector.FilePath()));

  projector->SetTexture(fullPath);
  projector->SetHFOV(_projector.HorizontalFov());
  projector->SetVisibilityFlags(_projector.VisibilityFlags());

  return projector;
}

/////////////////////////////////////////////////
bool SceneManager::AddSensor(Entity _gazeboId, const std::string &_sensorName,
    Entity _parentGazeboId)
{
  if (!this->dataPtr->scene)
    return false;

  if (this->dataPtr->sensors.find(_gazeboId) != this->dataPtr->sensors.end())
  {
    gzerr << "Sensor for entity [" << _gazeboId
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
    gzerr << "Unable to find sensor [" << _sensorName << "]" << std::endl;
    return false;
  }

  sensor->SetUserData("gazebo-entity", _gazeboId);

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
      this->dataPtr->particleEmitters.find(_id) !=
      this->dataPtr->particleEmitters.end() ||
      this->dataPtr->projectors.find(_id) !=
      this->dataPtr->projectors.end() ||
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
  auto lIt = this->dataPtr->lights.find(_id);
  if (lIt != this->dataPtr->lights.end())
  {
    return lIt->second;
  }
  auto sIt = this->dataPtr->sensors.find(_id);
  if (sIt != this->dataPtr->sensors.end())
  {
    return sIt->second;
  }
  auto pIt = this->dataPtr->particleEmitters.find(_id);
  if (pIt != this->dataPtr->particleEmitters.end())
  {
    return pIt->second;
  }
  auto prIt = this->dataPtr->projectors.find(_id);
  if (prIt != this->dataPtr->projectors.end())
  {
    return prIt->second;
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

  animData = this->dataPtr->ActorTrajectoryAt(_id, _time);

  auto skelIt = this->dataPtr->actorSkeletons.find(_id);
  if (skelIt == this->dataPtr->actorSkeletons.end())
    return animData;

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
    auto it = this->dataPtr->actors.find(_id);
    if (it != this->dataPtr->actors.end())
    {
      this->dataPtr->actors.erase(it);
    }
  }
  {
    auto it = this->dataPtr->actorTrajectories.find(_id);
    if (it != this->dataPtr->actorTrajectories.end())
    {
      this->dataPtr->actorTrajectories.erase(it);
    }
  }
  {
    auto it = this->dataPtr->actorSkeletons.find(_id);
    if (it != this->dataPtr->actorSkeletons.end())
    {
      this->dataPtr->actorSkeletons.erase(it);
    }
  }

  {
    auto it = this->dataPtr->visuals.find(_id);
    if (it != this->dataPtr->visuals.end())
    {
      // Remove visual's original transparency from map
      rendering::VisualPtr vis = it->second;
      this->dataPtr->originalTransparency.erase(vis->Name());
      // Remove visual's original depth write value from map
      this->dataPtr->originalDepthWrite.erase(vis->Name());

      for (auto g = 0u; g < vis->GeometryCount(); ++g)
      {
        auto geom = vis->GeometryByIndex(g);
        this->dataPtr->originalTransparency.erase(geom->Name());
        this->dataPtr->originalDepthWrite.erase(geom->Name());
      }

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
    auto it = this->dataPtr->particleEmitters.find(_id);
    if (it != this->dataPtr->particleEmitters.end())
    {
      this->dataPtr->scene->DestroyVisual(it->second);
      this->dataPtr->particleEmitters.erase(it);
      return;
    }
  }

  {
    auto it = this->dataPtr->projectors.find(_id);
    if (it != this->dataPtr->projectors.end())
    {
      this->dataPtr->scene->DestroyVisual(it->second);
      this->dataPtr->projectors.erase(it);
      return;
    }
  }

  {
    auto it = this->dataPtr->sensors.find(_id);
    if (it != this->dataPtr->sensors.end())
    {
      // Stop keeping track of it but don't destroy it;
      // gz-sensors is the one responsible for that.
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

////////////////////////////////////////////////
void SceneManager::UpdateTransparency(const rendering::NodePtr &_node,
    bool _makeTransparent)
{
  if (!_node)
    return;

  for (auto n = 0u; n < _node->ChildCount(); ++n)
  {
    auto child = _node->ChildByIndex(n);
    this->UpdateTransparency(child, _makeTransparent);
  }

  auto vis = std::dynamic_pointer_cast<rendering::Visual>(_node);
  if (nullptr == vis)
    return;

  // Visual material
  auto visMat = vis->Material();
  if (nullptr != visMat)
  {
    auto visTransparency =
        this->dataPtr->originalTransparency.find(vis->Name());
    auto visDepthWrite =
        this->dataPtr->originalDepthWrite.find(vis->Name());
    if (_makeTransparent)
    {
      if (visTransparency == this->dataPtr->originalTransparency.end())
      {
        this->dataPtr->originalTransparency[vis->Name()] =
          visMat->Transparency();
      }
      visMat->SetTransparency(1.0 - ((1.0 - visMat->Transparency()) * 0.5));

      if (visDepthWrite == this->dataPtr->originalDepthWrite.end())
      {
        this->dataPtr->originalDepthWrite[vis->Name()] =
          visMat->DepthWriteEnabled();
      }
      visMat->SetDepthWriteEnabled(false);
    }
    else
    {
      if (visTransparency != this->dataPtr->originalTransparency.end())
      {
        visMat->SetTransparency(visTransparency->second);
      }
      if (visDepthWrite != this->dataPtr->originalDepthWrite.end())
      {
        visMat->SetDepthWriteEnabled(visDepthWrite->second);
      }
    }
  }

  for (auto g = 0u; g < vis->GeometryCount(); ++g)
  {
    auto geom = vis->GeometryByIndex(g);

    // Geometry material
    auto geomMat = geom->Material();
    if (nullptr == geomMat || visMat == geomMat)
      continue;
    auto geomTransparency =
        this->dataPtr->originalTransparency.find(geom->Name());
    auto geomDepthWrite =
        this->dataPtr->originalDepthWrite.find(geom->Name());

    if (_makeTransparent)
    {
      if (geomTransparency == this->dataPtr->originalTransparency.end())
      {
        this->dataPtr->originalTransparency[geom->Name()] =
            geomMat->Transparency();
      }
      geomMat->SetTransparency(1.0 - ((1.0 - geomMat->Transparency()) * 0.5));

      if (geomDepthWrite == this->dataPtr->originalDepthWrite.end())
      {
        this->dataPtr->originalDepthWrite[geom->Name()] =
            geomMat->DepthWriteEnabled();
      }
      geomMat->SetDepthWriteEnabled(false);
    }
    else
    {
      if (geomTransparency != this->dataPtr->originalTransparency.end())
      {
        geomMat->SetTransparency(geomTransparency->second);
      }
      if (geomDepthWrite != this->dataPtr->originalDepthWrite.end())
      {
        geomMat->SetDepthWriteEnabled(geomDepthWrite->second);
      }
    }
  }
}

////////////////////////////////////////////////
void SceneManager::UpdateJointParentPose(Entity _jointId)
{
  auto visual =
      this->VisualById(_jointId);

  rendering::JointVisualPtr jointVisual =
      std::dynamic_pointer_cast<rendering::JointVisual>(visual);

  auto childPose = jointVisual->WorldPose();

  if (jointVisual->ParentAxisVisual())
  {
    jointVisual->ParentAxisVisual()->SetWorldPose(childPose);

    // scale parent axis visual to the child
    auto childScale = jointVisual->LocalScale();
    jointVisual->ParentAxisVisual()->SetLocalScale(childScale);
  }
}

/////////////////////////////////////////////////
Entity SceneManager::UniqueId() const
{
  auto id = std::numeric_limits<uint64_t>::max();
  while (true)
  {
    if (!this->HasEntity(id))
      return id;
    else if (id == 0u)
      return kNullEntity;
    --id;
  }
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

/////////////////////////////////////////////////
std::unordered_map<std::string, unsigned int>
SceneManager::LoadAnimations(const sdf::Actor &_actor)
{
  rendering::MeshDescriptor descriptor;
  descriptor.meshName = asFullPath(_actor.SkinFilename(), _actor.FilePath());
  common::MeshManager *meshManager = common::MeshManager::Instance();
  descriptor.mesh = meshManager->Load(descriptor.meshName);
  common::SkeletonPtr meshSkel = descriptor.mesh->MeshSkeleton();

  unsigned int numAnims = 0;
  std::unordered_map<std::string, unsigned int> mapAnimNameId;
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
      {
        if (!meshSkel->AddBvhAnimation(animFilename, animScale))
        {
          gzerr << "Bvh animation in file " << animFilename
                << " failed to load during actor creation" << std::endl;
          continue;
        }
      }
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
          gzwarn << "File [" << animFilename
                  << "] has more than one animation, "
                  << "but only the 1st one is used."
                  << std::endl;
        }
      }
      animMesh = meshManager->MeshByName(animFilename);

      // add the first animation
      auto firstAnim = animMesh->MeshSkeleton()->Animation(0);
      if (nullptr == firstAnim)
      {
        gzerr << "Failed to get animations from [" << animFilename
               << "]" << std::endl;
        mapAnimNameId.clear();
        break;
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
        // add a copy constructor to animation classes in gz-common.
        // The proper fix is probably to update gz-rendering to allow it to
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
  return mapAnimNameId;
}

/////////////////////////////////////////////////
std::vector<common::TrajectoryInfo>
SceneManagerPrivate::LoadTrajectories(const sdf::Actor &_actor,
  std::unordered_map<std::string, unsigned int> &_mapAnimNameId,
  common::SkeletonPtr _meshSkel)
{
  std::vector<common::TrajectoryInfo> trajectories;
  if (_actor.TrajectoryCount() != 0)
  {
    // Load all trajectories specified in sdf
    for (unsigned i = 0; i < _actor.TrajectoryCount(); ++i)
    {
      const sdf::Trajectory *trajSdf = _actor.TrajectoryByIndex(i);
      if (nullptr == trajSdf)
      {
        gzerr << "Null trajectory SDF for [" << _actor.Name() << "]"
              << std::endl;
        continue;
      }
      common::TrajectoryInfo trajInfo;
      trajInfo.SetId(trajSdf->Id());
      trajInfo.SetAnimIndex(_mapAnimNameId[trajSdf->Type()]);

      if (trajSdf->WaypointCount() != 0)
      {
        std::map<TP, math::Pose3d> waypoints;
        for (unsigned j = 0; j < trajSdf->WaypointCount(); ++j)
        {
          auto point = trajSdf->WaypointByIndex(j);
          TP pointTp(std::chrono::milliseconds(
                    static_cast<int>(point->Time() * 1000)));
          waypoints[pointTp] = point->Pose();
        }
        trajInfo.SetWaypoints(waypoints, trajSdf->Tension());
        // Animations are offset by 1 because index 0 is taken by the mesh name
        auto animation = _actor.AnimationByIndex(trajInfo.AnimIndex() - 1);

        if (animation && animation->InterpolateX())
        {
          // warn if no x displacement can be interpolated
          // warn only once per mesh
          static std::unordered_set<std::string> animInterpolateCheck;
          if (animInterpolateCheck.count(animation->Filename()) == 0)
          {
            std::string rootNodeName = _meshSkel->RootNode()->Name();
            common::SkeletonAnimation *skelAnim =
                _meshSkel->Animation(trajInfo.AnimIndex());
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
              gzwarn << "Animation has no x displacement. "
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
    common::TrajectoryInfo trajInfo;
    trajInfo.SetId(0);
    trajInfo.SetAnimIndex(0);
    trajInfo.SetStartTime(TP(0ms));

    double animLength = (_meshSkel) ? _meshSkel->Animation(0)->Length() : 0.0;
    auto timepoint = std::chrono::milliseconds(
                  static_cast<int>(animLength * 1000));
    trajInfo.SetEndTime(TP(timepoint));
    trajInfo.SetTranslated(false);
    trajectories.push_back(trajInfo);
  }
  return trajectories;
}

/////////////////////////////////////////////////
void SceneManager::Clear()
{
  this->dataPtr->visuals.clear();
  this->dataPtr->actors.clear();
  this->dataPtr->actorSkeletons.clear();
  this->dataPtr->actorTrajectories.clear();
  this->dataPtr->lights.clear();
  this->dataPtr->particleEmitters.clear();
  this->dataPtr->projectors.clear();
  this->dataPtr->sensors.clear();
  this->dataPtr->scene.reset();
  this->dataPtr->originalTransparency.clear();
  this->dataPtr->originalDepthWrite.clear();
}
