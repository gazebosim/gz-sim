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

#ifndef __APPLE__
#include <filesystem>
#endif
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "ignition/gazebo/Util.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
//////////////////////////////////////////////////
math::Pose3d worldPose(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  // work out pose in world frame
  math::Pose3d pose = _ecm.Component<components::Pose>(_entity)->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p)
  {
    // get pose of parent entity
    auto parentPose = _ecm.Component<components::Pose>(p->Data());
    if (!parentPose)
      break;
    // transform pose
    pose = pose + parentPose->Data();
    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
  }
  return pose;
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
    std::string prefix;
    if (_ecm.Component<components::World>(entity))
    {
      prefix = "world";
    }
    else if (_ecm.Component<components::Model>(entity))
    {
      prefix = "model";
    }
    else if (_ecm.Component<components::Light>(entity))
    {
      prefix = "light";
    }
    else if (_ecm.Component<components::Link>(entity))
    {
      prefix = "link";
    }
    else if (_ecm.Component<components::Collision>(entity))
    {
      prefix = "collision";
    }
    else if (_ecm.Component<components::Visual>(entity))
    {
      prefix = "visual";
    }
    else if (_ecm.Component<components::Joint>(entity))
    {
      prefix = "joint";
    }
    else if (_ecm.Component<components::Sensor>(entity))
    {
      prefix = "sensor";
    }
    else
    {
      ignwarn << "Skipping entity [" << name
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
  // Not a relative path, return unmodified
  if (_uri.find("://") != std::string::npos ||
      !std::filesystem::path(_uri).is_relative())
  {
    return _uri;
  }
#endif

  // When SDF is loaded from a string instead of a file
  if ("data-string" == _filePath)
  {
    ignwarn << "Can't resolve full path for relative path ["
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
  // Environment variable holding resource paths
  const std::string kResourcePathEnv{"IGN_GAZEBO_RESOURCE_PATH"};

  std::vector<std::string> gzPaths;
  char *gzPathCStr = getenv(kResourcePathEnv.c_str());
  if (gzPathCStr && *gzPathCStr != '\0')
  {
    gzPaths = common::Split(gzPathCStr, ':');
  }

  gzPaths.erase(remove_if(gzPaths.begin(), gzPaths.end(),
      std::mem_fun_ref(&std::string::empty)), gzPaths.end());

  return gzPaths;
}

//////////////////////////////////////////////////
void addResourcePaths(const std::vector<std::string> &_paths)
{
  // Environment variable holding resource paths
  const std::string kResourcePathEnv{"IGN_GAZEBO_RESOURCE_PATH"};

  // Environment variable used by SDFormat to find URIs inside <include>
  const std::string kSdfPathEnv{"SDF_PATH"};

  // SDF paths (for <include>s)
  std::vector<std::string> sdfPaths;
  char *sdfPathCStr = getenv(kSdfPathEnv.c_str());
  if (sdfPathCStr && *sdfPathCStr != '\0')
  {
    sdfPaths = common::Split(sdfPathCStr, ':');
  }

  // Ignition file paths (for <uri>s)
  auto systemPaths = common::systemPaths();
  std::vector<std::string> ignPaths;
  char *ignPathCStr = getenv(systemPaths->FilePathEnv().c_str());
  if (ignPathCStr && *ignPathCStr != '\0')
  {
    ignPaths = common::Split(ignPathCStr, ':');
  }

  // Gazebo resource paths
  std::vector<std::string> gzPaths;
  char *gzPathCStr = getenv(kResourcePathEnv.c_str());
  if (gzPathCStr && *gzPathCStr != '\0')
  {
    gzPaths = common::Split(gzPathCStr, ':');
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

    if (std::find(ignPaths.begin(), ignPaths.end(), path) == ignPaths.end())
    {
      ignPaths.push_back(path);
    }
  }

  // Update the vars
  std::string sdfPathsStr;
  for (const auto &path : sdfPaths)
    sdfPathsStr += ':' + path;

  setenv(kSdfPathEnv.c_str(), sdfPathsStr.c_str(), 1);

  std::string ignPathsStr;
  for (const auto &path : ignPaths)
    ignPathsStr += ':' + path;

  setenv(systemPaths->FilePathEnv().c_str(), ignPathsStr.c_str(), 1);

  std::string gzPathsStr;
  for (const auto &path : gzPaths)
    gzPathsStr += ':' + path;

  setenv(kResourcePathEnv.c_str(), gzPathsStr.c_str(), 1);

  // Force re-evaluation
  // SDF is evaluated at find call
  systemPaths->SetFilePathEnv(systemPaths->FilePathEnv());
}
}
}
}

