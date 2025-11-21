/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <mutex>
#include <chrono>
#include <optional>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderingIface.hh>

#include <gz/math/Color.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/Events.hh>

#include "LedPlugin.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace std::chrono_literals;

/// \brief Struct to define a step in the LED Mode Sequence
struct LedModeStep
{
  /// \brief Whether the LED should stay on indefinitely
  bool alwaysOn{false};

  /// \brief Whether the LED should stay on indefinitely
  math::Color ledColor{math::Color(1, 0, 0, 1)};

  /// \brief Seconds for which the LED should stay on.
  /// Used when alwaysOn is false
  std::chrono::duration<double> ledOnTime{1s};
};

/// \brief Struct to define the LED Mode
struct LedMode
{
  /// \brief Name of the LED Mode
  std::string name{""};

  /// \brief A list steps in sequence for the LED mode
  std::vector<LedModeStep> modeSequenceSteps;

  /// \brief Static factory method to create an instance of LedMode
  /// using the sdf element
  /// \param[in] _sdf SDF Element from which you want to construct
  /// the LED Mode
  public: static std::optional<LedMode> fromSDF(sdf::ElementConstPtr _sdf)
  {
    LedMode ledMode;

    if (!_sdf->HasAttribute("name"))
    {
      gzerr << "[LED PLUGIN] [LED MODE] Name attribute is "
            << "missing. Can't construct led mode" << std::endl;

      return std::nullopt;
    }

    ledMode.name = _sdf->Get<std::string>("name");

    if (!_sdf->HasElement("step"))
    {
      gzerr << "[LED PLUGIN] [LED MODE] No steps are given for "
            << " the LED mode: " << ledMode.name << ". Can't "
            << "construct led mode" << std::endl;

      return std::nullopt;
    }

    sdf::ElementConstPtr stepElem = _sdf->FindElement("step");
    while (stepElem)
    {
      LedModeStep ledModeStep;

      if (stepElem->HasAttribute("always_one"))
      {
        bool alwaysOn = stepElem->Get<bool>("always_on");
        ledModeStep.alwaysOn = alwaysOn;
      }

      if (stepElem->HasElement("color"))
      {
        math::Color ledColor = stepElem->Get<math::Color>("color");
        ledModeStep.ledColor = ledColor;
      }

      if (stepElem->HasElement("on_time"))
      {
        if (!ledModeStep.alwaysOn)
        {
          ledModeStep.ledOnTime = std::chrono::duration<double>(
            stepElem->Get<double>("on_time"));
        }
        else
        {
          gzwarn << "LED Mode: " << ledMode.name << " step is set to "
                 << "always on, on_time will be ignored" << std::endl;
        }
      }

      ledMode.modeSequenceSteps.push_back(ledModeStep);
      stepElem = stepElem->GetNextElement("step");
    }

    return ledMode;
  }
};

// Private data class
class gz::sim::systems::LedPluginPrivate
{
  /// \brief Callback invoked by the rendering thread before a render update
  public: void OnSceneUpdate();

  /// \brief Function to search for Visual By Entity ID
  public: rendering::VisualPtr FindEntityVisual(rendering::ScenePtr _scene,
    gz::sim::Entity _entity);

  /// \brief Connection to the pre-render event
  public: common::ConnectionPtr sceneUpdateConn{nullptr};

  /// \brief Pointer to EventManager
  public: EventManager *eventMgr{nullptr};

  /// \brief Entity of the model to blink
  public: Entity visualEntity;

  /// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene{nullptr};

  /// \brief Visual whose color will be changed.
  public: rendering::VisualPtr ledVisual{nullptr};

  /// \brief Pointer to the Material of the visual
  public: rendering::MaterialPtr ledMaterial{nullptr};

  /// \brief Default LED Mode to use
  public: LedMode defaultLedMode;

  /// \brief List of all the modes defined by the user
  public: std::vector<LedMode> allLedModes;

  /// \brief Time the current cycle started.
  public: std::chrono::duration<double> cycleStartTime{0s};

  /// \brief The current simulation time.
  public: std::chrono::duration<double> currentSimTime{0s};

  /// \brief Mutex to protect sim time updates
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
LedPlugin::LedPlugin()
    : System(), dataPtr(std::make_unique<LedPluginPrivate>())
{
}

/////////////////////////////////////////////////
LedPlugin::~LedPlugin()
{
}

/////////////////////////////////////////////////
void LedPlugin::Configure(
  const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &, gz::sim::EventManager &_eventMgr)
{
  this->dataPtr->visualEntity = _entity;
  this->dataPtr->eventMgr = &_eventMgr;

  this->dataPtr->allLedModes = std::vector<LedMode>();

  if (_sdf->HasElement("mode"))
  {
    sdf::ElementConstPtr ledModesElem = _sdf->FindElement("mode");

    while (ledModesElem)
    {

      auto ledMode = LedMode::fromSDF(ledModesElem);

      if (ledMode.has_value())
      {
        this->dataPtr->allLedModes.push_back(ledMode.value());
      }

      ledModesElem = ledModesElem->GetNextElement("mode");
    }
  }
  else
  {
    // TODO(jasmeet0915): Define a proper behavior for this case
    gzwarn << "[LED PLUGIN] No LED Modes has been defined "
           << "for the LED Plugin: " << _sdf->GetName()
           << " no LED will be used." << std::endl;
  }

  // Check and set default mode
  if (_sdf->HasElement("default_mode"))
  {
    std::string defaultModeName = _sdf->Get<std::string>("default_mode");

    auto defaultModeIter = std::find_if(
      this->dataPtr->allLedModes.begin(), this->dataPtr->allLedModes.end(),
      [&](const LedMode _ledMode)
      {
        if (_ledMode.name == defaultModeName)
        {
          return true;
        }

        return false;
      });
    
    if (defaultModeIter == this->dataPtr->allLedModes.end())
    {
      gzwarn << "[LED PLUGIN] Could not find default mode name: " << defaultModeName
             << " in the led modes mentioned. Using the first mode as default." << std::endl;
      this->dataPtr->defaultLedMode = this->dataPtr->allLedModes.front();
    }
    else
    {
      this->dataPtr->defaultLedMode = *(defaultModeIter);
    }
  }
  else
  {
    gzwarn << "[LED PLUGIN] No default led mode mentioned."
           << " in the led modes mentioned. Using the first mode as default." << std::endl;
    this->dataPtr->defaultLedMode = this->dataPtr->allLedModes.front();
  }

  gzmsg << "[LED PLUGIN] Successfully loaded the LED plugin with "
        << this->dataPtr->allLedModes.size() << " modes and "
        << this->dataPtr->defaultLedMode.name << " as the default mode" << std::endl;

  // Connect to the pre render event
  this->dataPtr->sceneUpdateConn = this->dataPtr->eventMgr->Connect<gz::sim::events::SceneUpdate>(
      std::bind(&LedPluginPrivate::OnSceneUpdate, this->dataPtr.get()));

  gzmsg << "[LED PLUGIN] Initialized LedPlugin Plugin" << std::endl;
}

/////////////////////////////////////////////////
void LedPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  // Set the current sim time in the PreUpdate function
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime =
    std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1>>>(_info.simTime);
}

//////////////////////////////////////////////////
gz::rendering::VisualPtr LedPluginPrivate::FindEntityVisual(
  gz::rendering::ScenePtr _scene, gz::sim::Entity _entity)
{
  for (unsigned int i = 0; i < _scene->VisualCount(); ++i)
  {
    gz::rendering::VisualPtr visual = _scene->VisualByIndex(i);
    if (visual->HasUserData("gazebo-entity"))
    {
      auto userData = visual->UserData("gazebo-entity");
      if (_entity == std::get<uint64_t>(userData))
      {
        return visual;
      }
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
void LedPluginPrivate::OnSceneUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Get a pointer to the rendering scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  if (!this->scene->IsInitialized())
  {
    gzmsg << "Scene is not initialized" << std::endl;
    return;
  }

  // Get the pointer to the visual using the Entity ID and its material if not already set
  if (!this->ledVisual)
  {
    this->ledVisual = this->FindEntityVisual(this->scene, this->visualEntity);
    if (!this->ledVisual)
    {
      gzerr << "Visual with the id: " << this->visualEntity <<
        " was not found. Maybe entity for plugin is not a Visual?" << std::endl;
      return;
    }
    this->ledMaterial = this->ledVisual->GeometryByIndex(0u)->Material();
  }

  // If material is not set, print an error and return
  if (!this->ledMaterial)
  {
    gzerr << "Could not access the material for the visual" << std::endl;
    return;
  }

  // Set the cycle start time. This is used to calculate the elapsed time in every PreRender event
  if (this->cycleStartTime == std::chrono::duration<double>::zero() ||
      this->cycleStartTime > this->currentSimTime)
  {
    this->cycleStartTime = this->currentSimTime;
  }
  std::chrono::duration<double> elapsed = this->currentSimTime - this->cycleStartTime;

  // Set the material of the visual with the current color
  // this->ledMaterial->SetDiffuse(this->currentColor);
  // this->ledMaterial->SetAmbient(this->currentColor);
  // this->ledMaterial->SetEmissive(this->currentColor);
  // this->ledMaterial->SetSpecular(this->currentColor);
}

GZ_ADD_PLUGIN(LedPlugin,
              System,
              LedPlugin::ISystemConfigure,
              LedPlugin::ISystemPreUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(LedPlugin, "gz::sim::systems::LedPlugin")
