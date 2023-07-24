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

#include <gz/msgs/entity.pb.h>

#include <gz/common/Filesystem.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/URI.hh>
#include <gz/common/Util.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/TopicUtils.hh>
#include <sdf/Types.hh>

#include <gz/fuel_tools/Interface.hh>
#include <gz/fuel_tools/ClientConfig.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Environment.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParticleEmitter.hh"
#include "gz/sim/components/Projector.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/LinearVelocity.hh"

#include "gz/sim/InstallationDirectories.hh"
#include "gz/sim/Util.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
//////////////////////////////////////////////////
math::Pose3d worldPose(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (nullptr == poseComp)
  {
    gzwarn << "Trying to get world pose from entity [" << _entity
            << "], which doesn't have a pose component" << std::endl;
    return math::Pose3d();
  }

  // work out pose in world frame
  math::Pose3d pose = poseComp->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p)
  {
    // get pose of parent entity
    auto parentPose = _ecm.Component<components::Pose>(p->Data());
    if (!parentPose)
      break;
    // transform pose
    pose = parentPose->Data() * pose;
    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }
  return pose;
}

//////////////////////////////////////////////////
math::Vector3d relativeVel(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (nullptr == poseComp)
  {
    gzwarn << "Trying to get world pose from entity [" << _entity
            << "], which doesn't have a pose component" << std::endl;
    return math::Vector3d();
  }

  // work out pose in world frame
  math::Pose3d pose = poseComp->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p)
  {
    // get pose of parent entity
    auto parentPose = _ecm.Component<components::Pose>(p->Data());
    if (!parentPose)
      break;
    // transform pose
    pose = parentPose->Data() * pose;
    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }

  auto worldLinVel = _ecm.Component<components::WorldLinearVelocity>(_entity);
  if (nullptr == worldLinVel)
  {
    gzwarn << "Trying to get world velocity from entity [" << _entity
            << "], which doesn't have a velocity component" << std::endl;
    return math::Vector3d();
  }

  math::Vector3d vel = worldLinVel->Data();
  return pose.Rot().RotateVectorReverse(vel);
}

//////////////////////////////////////////////////
std::string scopedName(const Entity &_entity,
    const EntityComponentManager &_ecm, const std::string &_delim,
    bool _includePrefix)
{
  std::string result;

  auto entity = _entity;

  while (true)
  {
    // Get entity name
    auto nameComp = _ecm.Component<components::Name>(entity);
    if (nullptr == nameComp)
      break;
    auto name = nameComp->Data();

    // Get entity type
    std::string prefix = entityTypeStr(entity, _ecm);
    if (prefix.empty())
    {
      gzwarn << "Skipping entity [" << name
              << "] when generating scoped name, entity type not known."
              << std::endl;
    }

    auto parentComp = _ecm.Component<components::ParentEntity>(entity);
    if (!prefix.empty())
    {
      result.insert(0, name);
      if (_includePrefix)
      {
        result.insert(0, _delim);
        result.insert(0, prefix);
      }
    }

    if (nullptr == parentComp)
      break;

    if (!prefix.empty())
      result.insert(0, _delim);

    entity = parentComp->Data();
  }

  return result;
}

//////////////////////////////////////////////////
std::unordered_set<Entity> entitiesFromScopedName(
    const std::string &_scopedName, const EntityComponentManager &_ecm,
    Entity _relativeTo, const std::string &_delim)
{
  if (_delim.empty())
  {
    gzwarn << "Can't process scoped name [" << _scopedName
            << "] with empty delimiter." << std::endl;
    return {};
  }

  // Split names
  std::vector<std::string> names;
  size_t pos1 = 0;
  size_t pos2 = _scopedName.find(_delim);
  while (pos2 != std::string::npos)
  {
    names.push_back(_scopedName.substr(pos1, pos2 - pos1));
    pos1 = pos2 + _delim.length();
    pos2 = _scopedName.find(_delim, pos1);
  }
  names.push_back(_scopedName.substr(pos1, _scopedName.size()-pos1));

  // Holds current entities that match and is updated for each name
  std::vector<Entity> resVector;

  // If there's an entity we're relative to, treat it as the first level result
  if (_relativeTo != kNullEntity)
  {
    resVector = {_relativeTo};
  }

  for (const auto &name : names)
  {
    std::vector<Entity> current;
    if (resVector.empty())
    {
      current = _ecm.EntitiesByComponents(components::Name(name));
    }
    else
    {
      for (auto res : resVector)
      {
        auto matches = _ecm.EntitiesByComponents(components::Name(name),
            components::ParentEntity(res));
        std::copy(std::begin(matches), std::end(matches),
            std::back_inserter(current));
      }
    }
    if (current.empty())
      return {};
    resVector = current;
  }

  return std::unordered_set<Entity>(resVector.begin(), resVector.end());
}

//////////////////////////////////////////////////
ComponentTypeId entityTypeId(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  ComponentTypeId type{kComponentTypeIdInvalid};

  if (_ecm.Component<components::World>(_entity))
  {
    type = components::World::typeId;
  }
  else if (_ecm.Component<components::Model>(_entity))
  {
    type = components::Model::typeId;
  }
  else if (_ecm.Component<components::Light>(_entity))
  {
    type = components::Light::typeId;
  }
  else if (_ecm.Component<components::Link>(_entity))
  {
    type = components::Link::typeId;
  }
  else if (_ecm.Component<components::Collision>(_entity))
  {
    type = components::Collision::typeId;
  }
  else if (_ecm.Component<components::Visual>(_entity))
  {
    type = components::Visual::typeId;
  }
  else if (_ecm.Component<components::Joint>(_entity))
  {
    type = components::Joint::typeId;
  }
  else if (_ecm.Component<components::Sensor>(_entity))
  {
    type = components::Sensor::typeId;
  }
  else if (_ecm.Component<components::Actor>(_entity))
  {
    type = components::Actor::typeId;
  }
  else if (_ecm.Component<components::ParticleEmitter>(_entity))
  {
    type = components::ParticleEmitter::typeId;
  }
  else if (_ecm.Component<components::Projector>(_entity))
  {
    type = components::Projector::typeId;
  }

  return type;
}

//////////////////////////////////////////////////
std::string entityTypeStr(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  std::string type;

  if (_ecm.Component<components::World>(_entity))
  {
    type = "world";
  }
  else if (_ecm.Component<components::Model>(_entity))
  {
    type = "model";
  }
  else if (_ecm.Component<components::Light>(_entity))
  {
    type = "light";
  }
  else if (_ecm.Component<components::Link>(_entity))
  {
    type = "link";
  }
  else if (_ecm.Component<components::Collision>(_entity))
  {
    type = "collision";
  }
  else if (_ecm.Component<components::Visual>(_entity))
  {
    type = "visual";
  }
  else if (_ecm.Component<components::Joint>(_entity))
  {
    type = "joint";
  }
  else if (_ecm.Component<components::Sensor>(_entity))
  {
    type = "sensor";
  }
  else if (_ecm.Component<components::Actor>(_entity))
  {
    type = "actor";
  }
  else if (_ecm.Component<components::ParticleEmitter>(_entity))
  {
    type = "particle_emitter";
  }
  else if (_ecm.Component<components::Projector>(_entity))
  {
    type = "projector";
  }

  return type;
}

//////////////////////////////////////////////////
Entity worldEntity(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  auto entity = _entity;
  while (nullptr == _ecm.Component<components::World>(entity))
  {
    // Keep going up the tree
    auto parentComp = _ecm.Component<components::ParentEntity>(entity);
    if (!parentComp)
    {
      entity = kNullEntity;
      break;
    }
    entity = parentComp->Data();
  }
  return entity;
}

//////////////////////////////////////////////////
Entity worldEntity(const EntityComponentManager &_ecm)
{
  return _ecm.EntityByComponents(components::World());
}

//////////////////////////////////////////////////
std::string removeParentScope(const std::string &_name,
                              const std::string &_delim)
{
  auto sepPos = _name.find(_delim);
  if (sepPos == std::string::npos || (sepPos + _delim.size()) > _name.size())
    return _name;

  return _name.substr(sepPos + _delim.size());
}

//////////////////////////////////////////////////
std::string asFullPath(const std::string &_uri, const std::string &_filePath)
{
  // No path, return unmodified
  if (_filePath.empty())
  {
    return _uri;
  }

#ifdef __APPLE__
  const std::string absPrefix = "/";
  // Not a relative path, return unmodified
  if (_uri.find("://") != std::string::npos ||
      _uri.compare(0, absPrefix.size(), absPrefix) == 0)
  {
    return _uri;
  }
#else
  if (_uri.find("://") != std::string::npos ||
      !common::isRelativePath(_uri))
  {
    return _uri;
  }
#endif

  // When SDF is loaded from a string instead of a file
  if (std::string(sdf::kSdfStringSource) == _filePath)
  {
    gzwarn << "Can't resolve full path for relative path ["
            << _uri << "]. Loaded from a data-string." << std::endl;
    return _uri;
  }

  // Remove file name from path
  auto path = common::parentPath(_filePath);
  auto uri = _uri;

  // If path is URI, use "/" separator for all platforms
  if (path.find("://") != std::string::npos)
  {
    std::replace(uri.begin(), uri.end(), '\\', '/');
    return path + "/" + uri;
  }

  // In case relative path doesn't match platform
#ifdef _WIN32
  std::replace(uri.begin(), uri.end(), '/', '\\');
#else
  std::replace(uri.begin(), uri.end(), '\\', '/');
#endif

  // Use platform-specific separator
  return common::joinPaths(path,  uri);
}

//////////////////////////////////////////////////
std::vector<std::string> resourcePaths()
{
  std::vector<std::string> gzPaths;
  char *gzPathCStr = std::getenv(kResourcePathEnv.c_str());
  if (gzPathCStr && *gzPathCStr != '\0')
  {
    gzPaths = common::Split(gzPathCStr, common::SystemPaths::Delimiter());
  }
  // TODO(CH3): Deprecated. Remove on tock.
  else
  {
    gzPathCStr = std::getenv(kResourcePathEnvDeprecated.c_str());
    if (gzPathCStr && *gzPathCStr != '\0')
    {
      gzwarn << "Using deprecated environment variable ["
             << kResourcePathEnvDeprecated
             << "] to find resources. Please use ["
             << kResourcePathEnv <<" instead." << std::endl;
      gzPaths = common::Split(gzPathCStr, ':');
    }
  }

  gzPaths.erase(std::remove_if(gzPaths.begin(), gzPaths.end(),
      [](const std::string &_path)
      {
        return _path.empty();
      }), gzPaths.end());

  return gzPaths;
}

//////////////////////////////////////////////////
void addResourcePaths(const std::vector<std::string> &_paths)
{
  // SDF paths (for <include>s)
  std::vector<std::string> sdfPaths;
  char *sdfPathCStr = std::getenv(kSdfPathEnv.c_str());
  if (sdfPathCStr && *sdfPathCStr != '\0')
  {
    sdfPaths = common::Split(sdfPathCStr, common::SystemPaths::Delimiter());
  }

  // Gazebo Common file paths (for <uri>s)
  auto systemPaths = common::systemPaths();
  std::vector<std::string> commonPaths;
  char *commonPathCStr = std::getenv(systemPaths->FilePathEnv().c_str());
  if (commonPathCStr && *commonPathCStr != '\0')
  {
    commonPaths = common::Split(commonPathCStr,
        common::SystemPaths::Delimiter());
  }

  // Gazebo resource paths
  std::vector<std::string> gzPaths;
  char *gzPathCStr = std::getenv(kResourcePathEnv.c_str());
  if (gzPathCStr && *gzPathCStr != '\0')
  {
    gzPaths = common::Split(gzPathCStr, common::SystemPaths::Delimiter());
  }
  // TODO(CH3): Deprecated. Remove on tock.
  else
  {
    gzPathCStr = std::getenv(kResourcePathEnvDeprecated.c_str());
    if (gzPathCStr && *gzPathCStr != '\0')
    {
      gzwarn << "Using deprecated environment variable ["
             << kResourcePathEnvDeprecated
             << "] to find resources. Please use ["
             << kResourcePathEnv <<" instead." << std::endl;
      gzPaths = common::Split(gzPathCStr, ':');
    }
  }

  // Add new paths to gzPaths
  for (const auto &path : _paths)
  {
    if (std::find(gzPaths.begin(), gzPaths.end(), path) == gzPaths.end())
    {
      gzPaths.push_back(path);
    }
  }

  // Append Gz paths to SDF / Ign paths
  for (const auto &path : gzPaths)
  {
    if (std::find(sdfPaths.begin(), sdfPaths.end(), path) == sdfPaths.end())
    {
      sdfPaths.push_back(path);
    }

    if (std::find(commonPaths.begin(),
        commonPaths.end(), path) == commonPaths.end())
    {
      commonPaths.push_back(path);
    }
  }

  // Update the vars
  std::string sdfPathsStr;
  for (const auto &path : sdfPaths)
    sdfPathsStr += common::SystemPaths::Delimiter() + path;

  common::setenv(kSdfPathEnv.c_str(), sdfPathsStr.c_str());

  std::string commonPathsStr;
  for (const auto &path : commonPaths)
    commonPathsStr += common::SystemPaths::Delimiter() + path;

  common::setenv(
    systemPaths->FilePathEnv().c_str(), commonPathsStr.c_str());

  std::string gzPathsStr;
  for (const auto &path : gzPaths)
    gzPathsStr += common::SystemPaths::Delimiter() + path;

  common::setenv(kResourcePathEnv.c_str(), gzPathsStr.c_str());

  // TODO(CH3): Deprecated. Remove on tock.
  common::setenv(kResourcePathEnvDeprecated.c_str(), gzPathsStr.c_str());

  // Force re-evaluation
  // SDF is evaluated at find call
  systemPaths->SetFilePathEnv(systemPaths->FilePathEnv());
}

//////////////////////////////////////////////////
sim::Entity topLevelModel(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  auto entity = _entity;

  // search up the entity tree and find the model with no parent models
  // (there is the possibility of nested models)
  Entity modelEntity = kNullEntity;
  while (entity)
  {
    if (_ecm.Component<components::Model>(entity))
      modelEntity = entity;

    // stop searching if we are at the root of the tree
    auto parentComp = _ecm.Component<components::ParentEntity>(entity);
    if (!parentComp)
      break;

    entity = parentComp->Data();
  }

  return modelEntity;
}

//////////////////////////////////////////////////
std::string topicFromScopedName(const Entity &_entity,
    const EntityComponentManager &_ecm, bool _excludeWorld)
{
  std::string topic = scopedName(_entity, _ecm, "/", true);

  if (_excludeWorld)
  {
    // Exclude the world name. If the entity is a world, then return an
    // empty string.
    topic = _ecm.Component<components::World>(_entity) ? "" :
      removeParentScope(removeParentScope(topic, "/"), "/");
  }

  return transport::TopicUtils::AsValidTopic("/" + topic);
}

//////////////////////////////////////////////////
std::string validTopic(const std::vector<std::string> &_topics)
{
  for (const auto &topic : _topics)
  {
    auto validTopic = transport::TopicUtils::AsValidTopic(topic);
    if (validTopic.empty())
    {
      gzerr << "Topic [" << topic << "] is invalid, ignoring." << std::endl;
      continue;
    }
    if (validTopic != topic)
    {
      gzdbg << "Topic [" << topic << "] changed to valid topic ["
             << validTopic << "]" << std::endl;
    }
    return validTopic;
  }
  return std::string();
}

//////////////////////////////////////////////////
Entity entityFromMsg(const EntityComponentManager &_ecm,
    const msgs::Entity &_msg)
{
  if (_msg.id() != kNullEntity)
  {
    return _msg.id();
  }

  // If there's no ID, check name + type
  if (_msg.type() == msgs::Entity::NONE)
  {
    return kNullEntity;
  }

  auto entities = entitiesFromScopedName(_msg.name(), _ecm);
  if (entities.empty())
  {
    return kNullEntity;
  }

  for (const auto &entity : entities)
  {
    if (_msg.type() == msgs::Entity::LIGHT &&
        _ecm.Component<components::Light>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::MODEL &&
        _ecm.Component<components::Model>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::LINK &&
        _ecm.Component<components::Link>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::VISUAL &&
        _ecm.Component<components::Visual>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::COLLISION &&
        _ecm.Component<components::Collision>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::SENSOR &&
        _ecm.Component<components::Sensor>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::JOINT &&
        _ecm.Component<components::Joint>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::ACTOR &&
        _ecm.Component<components::Actor>(entity))
    {
      return entity;
    }

    if (_msg.type() == msgs::Entity::WORLD &&
        _ecm.Component<components::World>(entity))
    {
      return entity;
    }
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> sphericalCoordinates(Entity _entity,
    const EntityComponentManager &_ecm)
{
  auto sphericalCoordinatesComp =
      _ecm.Component<components::SphericalCoordinates>(
      worldEntity(_entity, _ecm));
  if (nullptr == sphericalCoordinatesComp)
  {
    return std::nullopt;
  }

  auto xyzPose = worldPose(_entity, _ecm);

  // lat / lon / elevation in rad / rad / m
  auto rad = sphericalCoordinatesComp->Data().PositionTransform(
      xyzPose.Pos(),
      math::SphericalCoordinates::LOCAL2,
      math::SphericalCoordinates::SPHERICAL);

  // Return degrees
  return math::Vector3d(GZ_RTOD(rad.X()), GZ_RTOD(rad.Y()), rad.Z());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> getGridFieldCoordinates(
  const EntityComponentManager &_ecm,
  const math::Vector3d& _worldPosition,
  const std::shared_ptr<components::EnvironmentalData>& _gridField
  )
{

  auto origin =
    _ecm.Component<components::SphericalCoordinates>(worldEntity(_ecm));
  if (!origin)
  {
    if (_gridField->reference == math::SphericalCoordinates::SPHERICAL)
    {
      // If the reference frame is spherical, we must have some world reference
      // coordinates.
      gzerr << "World has no spherical coordinates,"
          << " but data was loaded with spherical reference plane"
          << std::endl;
      return std::nullopt;
    }
    else
    {
      // No need to transform
      return _worldPosition;
    }
  }
  auto position = origin->Data().PositionTransform(
      _worldPosition, math::SphericalCoordinates::LOCAL2,
      _gridField->reference);
  if (_gridField->reference == math::SphericalCoordinates::SPHERICAL &&
    _gridField->units == components::EnvironmentalData::ReferenceUnits::DEGREES)
  {
    position.X(GZ_RTOD(position.X()));
    position.Y(GZ_RTOD(position.Y()));
  }
  return position;
}

//////////////////////////////////////////////////
// Getting the first .sdf file in the path
std::string findFuelResourceSdf(const std::string &_path)
{
  if (!common::exists(_path))
    return "";

  for (common::DirIter file(_path); file != common::DirIter(); ++file)
  {
    std::string current(*file);
    if (!common::isFile(current))
      continue;

    auto fileName = common::basename(current);
    auto fileExtensionIndex = fileName.rfind(".");
    auto fileExtension = fileName.substr(fileExtensionIndex + 1);

    if (fileExtension == "sdf")
    {
      return current;
    }
  }
  return "";
}

//////////////////////////////////////////////////
std::string resolveSdfWorldFile(const std::string &_sdfFile,
    const std::string &_fuelResourceCache)
{
  std::string filePath;

  // Check Fuel if it's a URL
  auto sdfUri = common::URI(_sdfFile);
  if (sdfUri.Scheme() == "http" || sdfUri.Scheme() == "https")
  {
    fuel_tools::ClientConfig config;
    if (!_fuelResourceCache.empty())
      config.SetCacheLocation(_fuelResourceCache);
    fuel_tools::FuelClient fuelClient(config);

    std::string fuelCachePath;
    if (fuelClient.CachedWorld(common::URI(_sdfFile), fuelCachePath))
    {
      filePath = findFuelResourceSdf(fuelCachePath);
    }
    else if (auto result = fuelClient.DownloadWorld(
          common::URI(_sdfFile), fuelCachePath))
    {
      filePath = findFuelResourceSdf(fuelCachePath);
    }
    else
    {
      gzwarn << "Fuel couldn't download URL [" << _sdfFile
        << "], error: [" << result.ReadableResult() << "]"
        << std::endl;
    }
  }

  if (filePath.empty())
  {
    common::SystemPaths systemPaths;

    // Worlds from environment variable
    systemPaths.SetFilePathEnv(kResourcePathEnv);

    // Worlds installed with gz-sim
    systemPaths.AddFilePaths(gz::sim::getWorldInstallDir());

    filePath = systemPaths.FindFile(_sdfFile);
  }

  return filePath;
}

const common::Mesh *loadMesh(const sdf::Mesh &_meshSdf)
{
  const common::Mesh *mesh = nullptr;
  auto &meshManager = *common::MeshManager::Instance();
  if (common::URI(_meshSdf.Uri()).Scheme() == "name")
  {
    // if it has a name:// scheme, see if the mesh
    // exists in the mesh manager and load it by name
    const std::string basename = common::basename(_meshSdf.Uri());
    mesh = meshManager.MeshByName(basename);
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh by name [" << basename
             << "]." << std::endl;
      return nullptr;
    }
  }
  else if (meshManager.IsValidFilename(_meshSdf.Uri()))
  {
    // load mesh by file path
    auto fullPath = asFullPath(_meshSdf.Uri(), _meshSdf.FilePath());
    mesh = meshManager.Load(fullPath);
    if (nullptr == mesh)
    {
      gzwarn << "Failed to load mesh from [" << fullPath
             << "]." << std::endl;
      return nullptr;
    }
  }
  else
  {
    gzwarn << "Failed to load mesh [" << _meshSdf.Uri()
           << "]." << std::endl;
    return nullptr;
  }
  return mesh;
}
}
}
}
