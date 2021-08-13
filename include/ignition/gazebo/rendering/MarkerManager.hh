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
#ifndef IGNITION_GAZEBO_MARKERMANAGER_HH_
#define IGNITION_GAZEBO_MARKERMANAGER_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/rendering/Export.hh>

#include "ignition/rendering/RenderTypes.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
// Forward declare private data class.
class MarkerManagerPrivate;

/// \brief Creates, deletes, and maintains marker visuals. Only the
/// Scene class should instantiate and use this class.
class IGNITION_GAZEBO_RENDERING_VISIBLE MarkerManager
{
  /// \brief Constructor
  public: MarkerManager();

  /// \brief Destructor
  public: virtual ~MarkerManager();

  /// \brief Set the simulation time.
  /// \param[in] _time The provided time.
  public: void SetSimTime(const std::chrono::steady_clock::duration &_time);

  /// \brief Set the scene to manage
  /// \param[in] _scene Scene pointer.
  public: void SetScene(rendering::ScenePtr _scene);

  /// \brief Get the scene
  /// \return Pointer to scene
  public: rendering::ScenePtr Scene() const;

  /// \brief Update MarkerManager
  public: void Update();

  /// \brief Initialize the marker manager.
  /// \param[in] _scene Reference to the scene.
  /// \return True on success
  public: bool Init(const ignition::rendering::ScenePtr &_scene);

  /// \brief Set the marker service topic name.
  /// \param[in] _name Name of service
  public: void SetTopic(const std::string &_name);

  /// \internal
  /// \brief Private data pointer
  private: std::unique_ptr<MarkerManagerPrivate> dataPtr;
};
}

}
}
#endif
