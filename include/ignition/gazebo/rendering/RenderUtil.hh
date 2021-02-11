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
#ifndef IGNITION_GAZEBO_RENDERUTIL_HH_
#define IGNITION_GAZEBO_RENDERUTIL_HH_

#include <memory>
#include <string>
#include <vector>

#include <sdf/Sensor.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include "ignition/gazebo/rendering/SceneManager.hh"
#include "ignition/gazebo/rendering/MarkerManager.hh"


namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // forward declaration
  class RenderUtilPrivate;

  /// \class RenderUtil RenderUtil.hh ignition/gazebo/gui/plugins/RenderUtil.hh
  class IGNITION_GAZEBO_VISIBLE RenderUtil
  {
    /// \brief Constructor
    public: explicit RenderUtil();

    /// \brief Destructor
    public: ~RenderUtil();

    /// \brief Initialize the renderer. Must be called in the rendering thread.
    public: void Init();

    /// \brief Count of pending sensors. Must be called in the rendering thread.
    /// \return Number of sensors to be added on the next `Update` call
    ///
    /// In the case that RenderUtil has not been initialized, this method
    /// will return -1.
    public: int PendingSensors() const;

    /// \brief Main update function. Must be called in the rendering thread.
    public: void Update();

    /// \brief Get a pointer to the scene
    /// \return Pointer to the scene
    public: rendering::ScenePtr Scene() const;

    /// \brief Helper Update function for updating the scene
    /// \param[in] _info Sim update info
    /// \param[in] _ecm Mutable reference to the entity component manager
    public: void UpdateECM(const UpdateInfo &_info,
                           EntityComponentManager &_ecm);

    /// \brief Helper PostUpdate function for updating the scene
    public: void UpdateFromECM(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm);

    /// \brief Set the rendering engine to use
    /// \param[in] _engineName Name of the rendering engine.
    public: void SetEngineName(const std::string &_engineName);

    /// \brief Get the name of the rendering engine used
    /// \return Name of the rendering engine
    public: std::string EngineName() const;

    /// \brief Set the scene to use
    /// \param[in] _sceneName Name of the engine.
    public: void SetSceneName(const std::string &_sceneName);

    /// \brief Get the name of the rendering scene used
    /// \return Name of the rendering scene.
    public: std::string SceneName() const;

    /// \brief Set background color of render window
    /// \param[in] _color Color of render window background
    public: void SetBackgroundColor(const math::Color &_color);

    /// \brief Set ambient light of render window
    /// \param[in] _ambient Color of ambient light
    public: void SetAmbientLight(const math::Color &_ambient);

    /// \brief Show grid view in the scene
    public: void ShowGrid();

    /// \brief Set whether to use the current GL context
    /// \param[in] _enable True to use the current GL context
    public: void SetUseCurrentGLContext(bool _enable);

    /// \brief Set whether to create rendering sensors
    /// \param[in] _enable True to create rendering sensors
    /// \param[in] _createSensorCb Callback function for creating the sensors
    /// The callback function args are: sensor entity, sensor sdf
    /// and parent name, it returns the name of the rendering sensor created.
    public: void SetEnableSensors(bool _enable, std::function<
        std::string(const gazebo::Entity &, const sdf::Sensor &,
          const std::string &)> _createSensorCb = {});

    /// \brief Set the callback function for removing the sensors
    /// \param[in] _removeSensorCb Callback function for removing the sensors
    /// The callback function arg is the sensor entity to remove
    public : void SetRemoveSensorCb(
        std::function<void(const gazebo::Entity &)> _removeSensorCb);

    /// \brief View collisions of specified entity which are shown in orange
    /// \param[in] _entity Entity to view collisions
    public: void ViewCollisions(const Entity &_entity);

    /// \brief Get the scene manager
    /// Returns reference to the scene manager.
    public: class SceneManager &SceneManager();

    /// \brief Get the marker manager
    /// Returns reference to the marker manager.
    public: class MarkerManager &MarkerManager();

    /// \brief Get simulation time that the current rendering state corresponds
    /// to
    /// \returns Simulation time.
    public: std::chrono::steady_clock::duration SimTime() const;

    /// \brief Set the entity being selected
    /// \param[in] _node Node representing the selected entity
    public: void SetSelectedEntity(const rendering::NodePtr &_node);

    /// \brief Get the entity being selected. This will only return the
    /// last entity selected.
    public: rendering::NodePtr IGN_DEPRECATED(4) SelectedEntity() const;

    /// \brief Get the entities currently selected, in order of selection.
    /// \return Vector of currently selected entities
    public: std::vector<Entity> SelectedEntities() const;

    /// \brief Clears the set of selected entities and lowlights all of them.
    public: void DeselectAllEntities();

    /// \brief Set whether the transform controls are currently being dragged.
    /// \param[in] _active True if active.
    public: void SetTransformActive(bool _active);

    /// \brief Private data pointer.
    private: std::unique_ptr<RenderUtilPrivate> dataPtr;
  };
}
}
}
#endif
