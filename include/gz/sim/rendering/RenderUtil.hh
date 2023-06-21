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
#ifndef GZ_SIM_RENDERUTIL_HH_
#define GZ_SIM_RENDERUTIL_HH_

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <sdf/Sensor.hh>

#include <gz/sim/config.hh>
#include <gz/sim/rendering/Export.hh>
#include <gz/sim/System.hh>

#include "gz/sim/rendering/SceneManager.hh"
#include "gz/sim/rendering/MarkerManager.hh"


namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
  // forward declaration
  class RenderUtilPrivate;

  /// \class RenderUtil RenderUtil.hh gz/sim/gui/plugins/RenderUtil.hh
  class GZ_SIM_RENDERING_VISIBLE RenderUtil
  {
    /// \brief Constructor
    public: explicit RenderUtil();

    /// \brief Destructor
    public: ~RenderUtil();

    /// \brief Initialize the renderer. Must be called in the rendering thread.
    public: void Init();

    /// \brief Destroy the renderer. Must be called in the rendering thread.
    public: void Destroy();

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

    /// \brief Helper function to create visuals for new entities created in
    /// ECM. This function is intended to be used by other GUI plugins when
    /// new entities are created on the GUI side.
    /// \param[in] _ecm Const reference to the entity component manager
    /// \param[in] _entities Entities to create visuals for.
    public: void CreateVisualsForEntities(const EntityComponentManager &_ecm,
                                          const std::set<Entity> &_entities);

    /// \brief Set the rendering engine to use
    /// \param[in] _engineName Name of the rendering engine.
    public: void SetEngineName(const std::string &_engineName);

    /// \brief Get the name of the rendering engine used
    /// \return Name of the rendering engine
    public: std::string EngineName() const;

    /// \brief Set the API backend the rendering engine will use
    /// \param[in] _apiBackend Name of the api backend.
    /// See --render-engine-server-api-backend for possible options
    public: void SetApiBackend(const std::string &_apiBackend);

    /// \brief Get the API backend the rendering engine used
    /// \return Name of the API backend. May be empty.
    public: std::string ApiBackend() const;

    /// \brief Set the headless mode
    /// \param[in] _headless Set to true to enable headless mode.
    public: void SetHeadlessRendering(const bool &_headless);

    /// \brief Get the headless mode
    /// \return True if headless mode is enable, false otherwise.
    public: bool HeadlessRendering() const;

    /// \brief Set the scene to use
    /// \param[in] _sceneName Name of the engine.
    public: void SetSceneName(const std::string &_sceneName);

    /// \brief Get the name of the rendering scene used
    /// \return Name of the rendering scene.
    public: std::string SceneName() const;

    /// \brief Set the scene to use.
    /// \param[in] _scene Pointer to the scene.
    public: void SetScene(const rendering::ScenePtr &_scene);

    /// \brief Set background color of render window. This will override
    /// other sources, such as from SDF.
    /// \param[in] _color Color of render window background
    public: void SetBackgroundColor(const math::Color &_color);

    /// \brief Set ambient light of render window. This will override
    /// other sources, such as from SDF.
    /// \param[in] _ambient Color of ambient light
    public: void SetAmbientLight(const math::Color &_ambient);

    /// \brief Set whether to enable sky in the scene
    /// \param[in] _enabled True to enable sky, false to disable sky
    public: void SetSkyEnabled(bool _enabled);

    /// \brief Show grid view in the scene
    public: void ShowGrid();

    /// \brief Set whether to use the current GL context
    /// \param[in] _enable True to use the current GL context
    public: void SetUseCurrentGLContext(bool _enable);

    /// \brief Set the Window ID
    /// \param[in] _winID Window ID
    public: void SetWinID(const std::string &_winID);

    /// \brief Set whether to create rendering sensors
    /// \param[in] _enable True to create rendering sensors
    /// \param[in] _createSensorCb Callback function for creating the sensors
    /// The callback function args are: sensor entity, sensor sdf
    /// and parent name, it returns the name of the rendering sensor created.
    public: void SetEnableSensors(bool _enable, std::function<
        std::string(const sim::Entity &, const sdf::Sensor &,
          const std::string &)> _createSensorCb = {});

    /// \brief Set the callback function for removing the sensors
    /// \param[in] _removeSensorCb Callback function for removing the sensors
    /// The callback function arg is the sensor entity to remove
    public : void SetRemoveSensorCb(
        std::function<void(const sim::Entity &)> _removeSensorCb);

    /// \brief View an entity as transparent
    /// \param[in] _entity Entity to view as transparent
    public: void ViewTransparent(const Entity &_entity);

    /// \brief View center of mass of specified entity
    /// \param[in] _entity Entity to view center of mass
    public: void ViewCOM(const Entity &_entity);

    /// \brief View inertia of specified entity
    /// \param[in] _entity Entity to view inertia
    public: void ViewInertia(const Entity &_entity);

    /// \brief View joints of specified entity
    /// \param[in] _entity Entity to view joints
    public: void ViewJoints(const Entity &_entity);

    /// \brief View wireframes of specified entity
    /// \param[in] _entity Entity to view wireframes
    public: void ViewWireframes(const Entity &_entity);

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

    /// \brief Get the entities currently selected, in order of selection.
    /// \return Vector of currently selected entities
    public: const std::vector<Entity> &SelectedEntities() const;

    /// \brief Clears the set of selected entities and lowlights all of them.
    public: void DeselectAllEntities();

    /// \brief Init render engine plugins paths. This lets gz-rendering know
    /// paths to find render engine plugins
    public: void InitRenderEnginePluginPaths();

    /// \brief Helper function to get all child links of a model entity.
    /// \param[in] _entity Entity to find child links
    /// \return Vector of child links found for the parent entity
    private: std::vector<Entity> FindChildLinks(const Entity &_entity);

    /// \brief Helper function to hide wireboxes for an entity
    /// \param[in] _entity Entity to hide wireboxes
    private: void HideWireboxes(const Entity &_entity);

    /// \brief Set whether the transform controls are currently being dragged.
    /// \param[in] _active True if active.
    public: void SetTransformActive(bool _active);

    /// \brief Set the event manager to use
    /// \param[in] _mgr Event manager to set to.
    public: void SetEventManager(EventManager *_mgr);

    /// \brief Private data pointer.
    private: std::unique_ptr<RenderUtilPrivate> dataPtr;
  };
}
}
}
#endif
