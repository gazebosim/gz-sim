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
#ifndef GZ_SIM_MARKERMANAGER_HH_
#define GZ_SIM_MARKERMANAGER_HH_

#include <memory>
#include <string>

#include <gz/sim/config.hh>
#include <gz/sim/rendering/Export.hh>

#include "gz/rendering/RenderTypes.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
// Forward declare private data class.
class MarkerManagerPrivate;

/// \brief Creates, deletes, and maintains marker visuals. Only the
/// Scene class should instantiate and use this class.
class GZ_SIM_RENDERING_VISIBLE MarkerManager
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
  public: bool Init(const gz::rendering::ScenePtr &_scene);

  /// \brief Set the marker service topic name.
  /// \param[in] _name Name of service
  public: void SetTopic(const std::string &_name);

  /// \brief Clear the marker manager
  /// Clears internal resources stored in the marker manager.
  /// Note: this does not actually destroy the objects.
  public: void Clear();

  /// \internal
  /// \brief Private data pointer
  private: std::unique_ptr<MarkerManagerPrivate> dataPtr;
};
}

}
}
#endif
