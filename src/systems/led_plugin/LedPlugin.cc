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

#include <string>
#include <mutex>
#include <chrono>
#include <optional>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include <gz/plugin/Register.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/light.pb.h>

#include <gz/math/Color.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Types.hh>
#include "gz/sim/Model.hh"
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Visual.hh>
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/VisualCmd.hh"
#include "gz/sim/components/LightCmd.hh"

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
  math::Color ledColor{math::Color::White};

  /// \brief Seconds for which the LED should stay on.
  /// Used when alwaysOn is false
  std::chrono::duration<double> ledOnTime{1s};

  /// \brief Intensity for any lights as part of the LED
  /// Defaults to 1.0
  double lightIntensity{1.0};
};

/// \brief Struct to define the individual LED
struct Led
{
  /// \brief Name of the LED
  std::string ledName{""};

  /// \brief Scoped name of the Visual of the LED
  std::string scopedVisualName{""};

  /// \brief Scoped name of the light of the LED
  std::string scopedLightName{""};

  /// \brief Entity of the visual to be used with this LED
  sim::Entity ledVisualEntity{kNullEntity};

  /// \brief Entity of the light to be used with this LED
  sim::Entity ledLightEntity{kNullEntity};

  /// \brief The default color to use when the LED is reset
  math::Color defaultColor{math::Color::White};

  /// \brief The default intensity to use for the light when LED is reset
  double defaultIntensity{0.0};

  public: static std::optional<Led> fromSDF(sdf::ElementConstPtr _sdf,
    const EntityComponentManager &_ecm)
  {
    Led led;

    if (!_sdf->HasAttribute("name"))
    {
      gzerr << "[LED PLUGIN][LED] Name attribute for the LED is "
            << "missing. Can't construct LED" << std::endl;
      return std::nullopt;
    }

    led.ledName = _sdf->Get<std::string>("name");
    gzmsg << "[LED PLUGIN][LED] Creating led: " << led.ledName << std::endl;

    // Use the visual name to find the Visual entity provided for LED
    if (_sdf->HasElement("visual_name"))
    {
      led.scopedVisualName = _sdf->Get<std::string>("visual_name");
      std::unordered_set<Entity> visualEntities =
        gz::sim::entitiesFromScopedName(led.scopedVisualName, _ecm);
      gzmsg << "Found " << visualEntities.size()
            << " entities for the visual named: "
            << led.scopedVisualName << std::endl;

      if (visualEntities.empty())
      {
        gzerr << "[LED PLUGIN][LED] No visuals found"
              << " with the name: "
              << led.scopedVisualName
              << ". Can't use visuals for the LED"
              << std::endl;
      }
      else
      {
        if (visualEntities.size() > 1)
        {
          gzerr << "[LED PLUGIN][LED] Multiple visuals"
                << " found with the name: "
                << led.scopedVisualName
                << ". Using the first one found"
                << std::endl;
        }

        led.ledVisualEntity = *visualEntities.begin();
      }
    }

    // Use the light name to find the Light entity provided for LED
    if (_sdf->HasElement("light_name"))
    {
      led.scopedLightName = _sdf->Get<std::string>("light_name");
      std::unordered_set<Entity> lightEntities =
        gz::sim::entitiesFromScopedName(led.scopedLightName, _ecm);

      gzmsg << "Found " << lightEntities.size()
            << " entities for the light named: "
            << led.scopedLightName << std::endl;

      if (lightEntities.empty())
      {
        gzerr << "[LED PLUGIN][LED] No lights found"
              << " with the name: "
              << led.scopedLightName
              << ". Can't use lights for the LED"
              << std::endl;
      }
      else
      {
        if (lightEntities.size() > 1)
        {
          gzerr << "[LED PLUGIN][LED] Multiple lights"
                << " found with the name: "
                << led.scopedLightName
                << ". Using the first one found"
                << std::endl;
        }

        led.ledLightEntity = *lightEntities.begin();
      }
    }

    // Read the default state of the LED if provided
    if (_sdf->HasElement("default_state"))
    {
      sdf::ElementConstPtr defaultStateElem =
        _sdf->FindElement("default_state");

      if (defaultStateElem->HasElement("color"))
      {
        led.defaultColor = defaultStateElem->Get<math::Color>("color");
      }

      if (defaultStateElem->HasElement("intensity"))
      {
        led.defaultIntensity = defaultStateElem->Get<double>("intensity");
      }
    }

    return led;
  }
};

/// \brief Struct to define the LED Mode
struct LedMode
{
  /// \brief Name of the LED Mode
  std::string name{""};

  /// \brief List of string names of the active LEDs in this mode
  std::vector<std::string> activeLedNames;

  /// \brief A list steps in sequence for the LED mode
  std::vector<LedModeStep> modeSequenceSteps;

  /// \brief Static factory method to create an instance of LedMode
  /// using the sdf element
  /// \param[in] _sdf SDF Element from which you want to construct
  /// the LED Mode
  public: static std::optional<LedMode> fromSDF(sdf::ElementConstPtr _sdf)
  {
    LedMode ledMode;

    // Read and set the name of the LED Mode
    if (!_sdf->HasAttribute("name"))
    {
      gzerr << "[LED PLUGIN] [LED MODE] Name attribute is "
            << "missing. Can't construct led mode" << std::endl;

      return std::nullopt;
    }

    ledMode.name = _sdf->Get<std::string>("name");
    gzmsg << "Adding LED Mode: " << ledMode.name << std::endl;

    // Read the active LEDs for this mode if any
    if (_sdf->HasElement("active_leds"))
    {
      sdf::ElementConstPtr activeLedsElem = _sdf->FindElement("active_leds");

      if (activeLedsElem->HasElement("led"))
      {
        sdf::ElementConstPtr ledElem = activeLedsElem->FindElement("led");

        while (ledElem)
        {
          std::string ledName = ledElem->Get<std::string>();
          ledMode.activeLedNames.push_back(ledName);

          gzmsg << "[LED PLUGIN][LED MODE] Mode [" << ledMode.name
                << "] uses LED: " << ledName << std::endl;

          ledElem = ledElem->GetNextElement("led");
        }
      }
    }

    // Read the different steps involved in this LED Mode
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

      if (stepElem->HasAttribute("always_on"))
      {
        ledModeStep.alwaysOn = stepElem->Get<bool>("always_on");
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

      if (stepElem->HasElement("intensity"))
      {
        ledModeStep.lightIntensity = stepElem->Get<double>("intensity");
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
  /// \brief Callback executed to change the current LED mode.
  /// \param[in] _req String specifying the name of the required mode.
  /// \param[out] _resp Boolean for successful mode change
  public: bool OnLedModeChange(
    const msgs::StringMsg &_req, msgs::Boolean &_resp);

  /// \brief
  /// \param[in] _visualEntity
  /// \param[in] _ecm
  /// \param[in] _materialColor
  public: void SetVisualProperties(const Entity &_visualEntity,
    EntityComponentManager &_ecm, const math::Color &_materialColor);

  /// \brief
  /// \param[in] _lightEntity
  /// \param[in] _ecm
  /// \param[in] _lightColor
  public: void SetLightProperties(const Entity &_lightEntity,
    EntityComponentManager &_ecm, const math::Color &_lightColor,
    const double _lightIntensity);

  /// \brief
  /// \param[in] _ecm
  public: void ResetLEDs(EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief String name of the LED group
  public: std::string ledGroupName;

  /// \brief List of all the LEDs in group
  public: std::unordered_map<std::string, Led> allLedsInGroup;

  /// \brief Boolean that tells if all the LEDs are ready or not
  public: bool ledsReady{false};

  /// \brief Whether LEDs are in the "off" / "reset" state.
  /// When true, LEDs stay reset until a new mode is explicitly requested.
  public: bool ledsOff{false};

  /// \brief Startup LED Mode to use
  public: LedMode startupLedMode;

  /// \brief Current LED Mode to use
  public: LedMode currentLedMode;

  /// \brief Index of the current step being executed of the mode
  public: size_t currentModeStepIdx{0};

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
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &)
{
  // Create a Model interface instance from the model entity of this plugin
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "LedPlugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->allLedModes = std::vector<LedMode>();

  // Construct the name for the mode change service of the led group
  std::string modeChangeServiceName;
  if (_sdf->HasElement("led_group_name"))
  {
    this->dataPtr->ledGroupName = _sdf->Get<std::string>("led_group_name");
  }
  else
  {
    gzwarn << "No name is specified for the led group, model name"
           << " will be used for the change_mode service name" << std::endl;
    this->dataPtr->ledGroupName = "led_" + this->dataPtr->model.Name(_ecm);
  }

  modeChangeServiceName =
    "/" + this->dataPtr->ledGroupName
    + "/change_led_mode";

  // Read and create different LEDs as part of the group
  if (_sdf->HasElement("led"))
  {
    sdf::ElementConstPtr ledElem = _sdf->FindElement("led");

    while (ledElem)
    {
      auto led = Led::fromSDF(ledElem, _ecm);

      if (led.has_value())
      {
        this->dataPtr->allLedsInGroup.insert(
          {led.value().ledName, led.value()});
      }

      ledElem = ledElem->GetNextElement("led");
    }
  }
  else
  {
    gzerr << "[LED PLUGIN] No LEDs have been defined "
          << "for the LED group: " << this->dataPtr->ledGroupName << std::endl;
    return;
  }

  // Read the described LED modes
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
    gzerr << "[LED PLUGIN] No LED Modes has been defined "
          << "for the LED group: " << this->dataPtr->ledGroupName << std::endl;
    return;
  }

  // Set the startup led mode
  if (!this->dataPtr->allLedModes.empty())
  {
    if (_sdf->HasElement("startup_mode"))
    {
      std::string startupModeName =
        _sdf->Get<std::string>("startup_mode");

      auto startupModeIter = std::find_if(
        this->dataPtr->allLedModes.begin(), this->dataPtr->allLedModes.end(),
        [&](const LedMode _ledMode)
        {
          if (_ledMode.name == startupModeName)
          {
            return true;
          }

          return false;
        });

      if (startupModeIter == this->dataPtr->allLedModes.end())
      {
        gzwarn << "[LED PLUGIN] Could not find"
              << " startup mode name: "
              << startupModeName
              << " in the led modes mentioned."
              << " Using the first mode as startup."
              << std::endl;
        this->dataPtr->startupLedMode =
          this->dataPtr->allLedModes.front();
      }
      else
      {
        this->dataPtr->startupLedMode = *(startupModeIter);
      }
    }
    else
    {
      gzwarn << "[LED PLUGIN] No startup led mode"
            << " mentioned. Using the first"
            << " mode as startup." << std::endl;
      this->dataPtr->startupLedMode =
        this->dataPtr->allLedModes.front();
    }
  }
  else
  {
    gzerr << "[LED PLUGIN] No LED Modes were created "
           << "successfully, plugin won't work" << std::endl;

    return;
  }

  this->dataPtr->currentLedMode = this->dataPtr->startupLedMode;
  gzmsg << "[LED PLUGIN] Successfully loaded the LED plugin with "
        << this->dataPtr->allLedsInGroup.size() << " LEDs, "
        << this->dataPtr->allLedModes.size() << " modes and "
        << this->dataPtr->startupLedMode.name
        << " as the startup mode" << std::endl;

  // Advertise the LED Mode change service
  auto validModeChangeServiceName =
    transport::TopicUtils::AsValidTopic(modeChangeServiceName);
  if (validModeChangeServiceName.empty())
  {
    gzerr << "Failed to create valid mode change service. Name not valid: ["
          << validModeChangeServiceName << "]" << std::endl;
    return;
  }

  this->dataPtr->node.Advertise(validModeChangeServiceName,
    &LedPluginPrivate::OnLedModeChange, this->dataPtr.get());

  gzmsg << "[LED PLUGIN] Initialized LedPlugin Plugin" << std::endl;
}

/////////////////////////////////////////////////
void LedPlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  // Set the current sim time in the PreUpdate function
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime =
    std::chrono::duration_cast<
      std::chrono::duration<double,
        std::ratio<1, 1>>>(_info.simTime);

  // Return if no LEDs were created successfully
  if (this->dataPtr->allLedsInGroup.empty())
  {
    return;
  }

  // Return if no LED modes were defined
  if (this->dataPtr->allLedModes.empty())
  {
    return;
  }

  // Reset the LEDs if they are not ready at the moment
  if (!this->dataPtr->ledsReady)
  {
    this->dataPtr->ResetLEDs(_ecm);
    this->dataPtr->ledsReady = true;
  }

  // Set the current step from the current LED Mode
  LedModeStep currentLedModeStep =
    this->dataPtr->currentLedMode.modeSequenceSteps[
      this->dataPtr->currentModeStepIdx];

  // Change the visual and light properties of the
  // LEDs based on the current mode
  if (this->dataPtr->currentLedMode.activeLedNames.empty())
  {
    // Set the state of all LEDs in the group according to the current LED Mode
    for (const auto &led : this->dataPtr->allLedsInGroup)
    {
      // Set the visual properties if the visual entity is not null
      if (led.second.ledVisualEntity != kNullEntity)
      {
        this->dataPtr->SetVisualProperties(
          led.second.ledVisualEntity, _ecm, currentLedModeStep.ledColor);
      }

      // Set the light properties if the light entity is not null
      if (led.second.ledLightEntity != kNullEntity)
      {
        this->dataPtr->SetLightProperties(
          led.second.ledLightEntity, _ecm,
          currentLedModeStep.ledColor,
          currentLedModeStep.lightIntensity);
      }
    }
  }
  else
  {
    // Set the state of the LEDs in the activeLedNames
    // for the current LED Mode
    for (const std::string &ledName :
      this->dataPtr->currentLedMode.activeLedNames)
    {
      // Set the visual properties if the visual entity is not null
      if (this->dataPtr->allLedsInGroup[ledName].ledVisualEntity != kNullEntity)
      {
        this->dataPtr->SetVisualProperties(
          this->dataPtr->allLedsInGroup[ledName]
            .ledVisualEntity,
          _ecm, currentLedModeStep.ledColor);
      }

      // Set the light properties if the light entity is not null
      if (this->dataPtr->allLedsInGroup[ledName].ledLightEntity != kNullEntity)
      {
        this->dataPtr->SetLightProperties(
          this->dataPtr->allLedsInGroup[ledName]
            .ledLightEntity,
          _ecm, currentLedModeStep.ledColor,
          currentLedModeStep.lightIntensity);
      }
    }
  }

  // If this step is supposed to be always on then just return
  if (currentLedModeStep.alwaysOn)
  {
    return;
  }

  // Set the cycle start time. This is used to calculate
  // the elapsed time in every PreUpdate event
  if (this->dataPtr->cycleStartTime == std::chrono::duration<double>::zero() ||
      this->dataPtr->cycleStartTime > this->dataPtr->currentSimTime)
  {
    this->dataPtr->cycleStartTime = this->dataPtr->currentSimTime;
  }
  std::chrono::duration<double> elapsed =
    this->dataPtr->currentSimTime
    - this->dataPtr->cycleStartTime;

  // If we have crossed the elapsed time of
  // the step on time, then move on to the next step
  if (elapsed > currentLedModeStep.ledOnTime)
  {
    this->dataPtr->currentModeStepIdx++;

    // If we have reached the end of the steps, cycle back to the first one
    if (this->dataPtr->currentModeStepIdx ==
        this->dataPtr->currentLedMode.modeSequenceSteps.size())
    {
      this->dataPtr->currentModeStepIdx = 0;
    }

    // Reset the cycle start time
    this->dataPtr->cycleStartTime = std::chrono::duration<double>::zero();
  }
}

//////////////////////////////////////////////////
void LedPluginPrivate::SetVisualProperties(const Entity &_visualEntity,
  EntityComponentManager &_ecm, const math::Color &_materialColor)
{
  msgs::Visual visualMsg;
  visualMsg.set_id(_visualEntity);

  visualMsg.mutable_material()->mutable_ambient()->set_r(_materialColor.R());
  visualMsg.mutable_material()->mutable_ambient()->set_g(_materialColor.G());
  visualMsg.mutable_material()->mutable_ambient()->set_b(_materialColor.B());
  visualMsg.mutable_material()->mutable_ambient()->set_a(_materialColor.A());

  visualMsg.mutable_material()->mutable_diffuse()->set_r(_materialColor.R());
  visualMsg.mutable_material()->mutable_diffuse()->set_g(_materialColor.G());
  visualMsg.mutable_material()->mutable_diffuse()->set_b(_materialColor.B());
  visualMsg.mutable_material()->mutable_diffuse()->set_a(_materialColor.A());

  visualMsg.mutable_material()->mutable_specular()->set_r(_materialColor.R());
  visualMsg.mutable_material()->mutable_specular()->set_g(_materialColor.G());
  visualMsg.mutable_material()->mutable_specular()->set_b(_materialColor.B());
  visualMsg.mutable_material()->mutable_specular()->set_a(_materialColor.A());

  visualMsg.mutable_material()->mutable_emissive()->set_r(_materialColor.R());
  visualMsg.mutable_material()->mutable_emissive()->set_g(_materialColor.G());
  visualMsg.mutable_material()->mutable_emissive()->set_b(_materialColor.B());
  visualMsg.mutable_material()->mutable_emissive()->set_a(_materialColor.A());

  std::function<bool(const msgs::Visual &, const msgs::Visual &)> visualEq =
  [](const msgs::Visual &_a, const msgs::Visual &_b)
  {
    auto aMaterial = _a.material(), bMaterial = _b.material();

    return
      _a.name() == _b.name() &&
      _a.id() == _b.id() &&
      math::equal(
        aMaterial.ambient().r(), bMaterial.ambient().r(), 1e-6f) &&
      math::equal(
        aMaterial.ambient().g(), bMaterial.ambient().g(), 1e-6f) &&
      math::equal(
        aMaterial.ambient().b(), bMaterial.ambient().b(), 1e-6f) &&
      math::equal(
        aMaterial.ambient().a(), bMaterial.ambient().a(), 1e-6f) &&
      math::equal(
        aMaterial.diffuse().r(), bMaterial.diffuse().r(), 1e-6f) &&
      math::equal(
        aMaterial.diffuse().g(), bMaterial.diffuse().g(), 1e-6f) &&
      math::equal(
        aMaterial.diffuse().b(), bMaterial.diffuse().b(), 1e-6f) &&
      math::equal(
        aMaterial.diffuse().a(), bMaterial.diffuse().a(), 1e-6f) &&
      math::equal(
        aMaterial.specular().r(), bMaterial.specular().r(), 1e-6f) &&
      math::equal(
        aMaterial.specular().g(), bMaterial.specular().g(), 1e-6f) &&
      math::equal(
        aMaterial.specular().b(), bMaterial.specular().b(), 1e-6f) &&
      math::equal(
        aMaterial.specular().a(), bMaterial.specular().a(), 1e-6f) &&
      math::equal(
        aMaterial.emissive().r(), bMaterial.emissive().r(), 1e-6f) &&
      math::equal(
        aMaterial.emissive().g(), bMaterial.emissive().g(), 1e-6f) &&
      math::equal(
        aMaterial.emissive().b(), bMaterial.emissive().b(), 1e-6f) &&
      math::equal(
        aMaterial.emissive().a(), bMaterial.emissive().a(), 1e-6f);
  };

  auto visualCmdComp = _ecm.Component<components::VisualCmd>(_visualEntity);
  if (!visualCmdComp)
  {
    _ecm.CreateComponent(_visualEntity, components::VisualCmd(visualMsg));
  }
  else
  {
    auto state = visualCmdComp->SetData(visualMsg, visualEq) ?
      ComponentState::OneTimeChange : ComponentState::NoChange;
    _ecm.SetChanged(_visualEntity, components::VisualCmd::typeId, state);
  }
}

//////////////////////////////////////////////////
void LedPluginPrivate::SetLightProperties(
  const Entity &_lightEntity,
  EntityComponentManager &_ecm,
  const math::Color &_lightColor,
  const double _lightIntensity)
{
  const auto* currentLightMsg = _ecm.Component<components::Light>(_lightEntity);

  msgs::Light lightMsg;
  lightMsg.set_id(_lightEntity);
  lightMsg.set_intensity(_lightIntensity);

  lightMsg.mutable_diffuse()->set_r(_lightColor.R());
  lightMsg.mutable_diffuse()->set_g(_lightColor.G());
  lightMsg.mutable_diffuse()->set_b(_lightColor.B());
  lightMsg.mutable_diffuse()->set_a(_lightColor.A());

  lightMsg.mutable_specular()->set_r(_lightColor.R());
  lightMsg.mutable_specular()->set_g(_lightColor.G());
  lightMsg.mutable_specular()->set_b(_lightColor.B());
  lightMsg.mutable_specular()->set_a(_lightColor.A());

  // Set the rest of the properties with the current Light component data
  lightMsg.set_range(currentLightMsg->Data().AttenuationRange());
  lightMsg.set_attenuation_constant(
    currentLightMsg->Data().ConstantAttenuationFactor());
  lightMsg.set_attenuation_linear(
    currentLightMsg->Data().LinearAttenuationFactor());
  lightMsg.set_attenuation_quadratic(
    currentLightMsg->Data().QuadraticAttenuationFactor());

  std::function<bool(const msgs::Light &, const msgs::Light &)> lightEq =
  [](const msgs::Light &_a, const msgs::Light &_b)
  {
    return
      _a.name() == _b.name() &&
      _a.id() == _b.id() &&
      _a.type() == _b.type() &&
      math::equal(
        _a.intensity(), _b.intensity(), 1e-6f) &&
      math::equal(
        _a.diffuse().r(), _b.diffuse().r(), 1e-6f) &&
      math::equal(
        _a.diffuse().g(), _b.diffuse().g(), 1e-6f) &&
      math::equal(
        _a.diffuse().b(), _b.diffuse().b(), 1e-6f) &&
      math::equal(
        _a.diffuse().a(), _b.diffuse().a(), 1e-6f) &&
      math::equal(
        _a.specular().r(), _b.specular().r(), 1e-6f) &&
      math::equal(
        _a.specular().g(), _b.specular().g(), 1e-6f) &&
      math::equal(
        _a.specular().b(), _b.specular().b(), 1e-6f) &&
      math::equal(
        _a.specular().a(), _b.specular().a(), 1e-6f);
  };

  auto lightCmdComp = _ecm.Component<components::LightCmd>(_lightEntity);
  if (!lightCmdComp)
  {
    _ecm.CreateComponent(_lightEntity, components::LightCmd(lightMsg));
  }
  else
  {
    auto state = lightCmdComp->SetData(lightMsg, lightEq) ?
      ComponentState::OneTimeChange : ComponentState::NoChange;
    _ecm.SetChanged(_lightEntity, components::LightCmd::typeId, state);
  }
}

//////////////////////////////////////////////////
void LedPluginPrivate::ResetLEDs(EntityComponentManager &_ecm)
{
  for (const auto &led : this->allLedsInGroup)
  {
    if (led.second.ledVisualEntity != kNullEntity)
    {
      this->SetVisualProperties(
        led.second.ledVisualEntity,
        _ecm, led.second.defaultColor);
    }

    if (led.second.ledLightEntity != kNullEntity)
    {
      this->SetLightProperties(led.second.ledLightEntity, _ecm,
        led.second.defaultColor, led.second.defaultIntensity);
    }
  }
}

//////////////////////////////////////////////////
bool LedPluginPrivate::OnLedModeChange(const msgs::StringMsg &_req,
  msgs::Boolean &_resp)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  std::string requestedModeName = _req.data();
  gzmsg << "[LED PLUGIN] [ON MODE CHANGE] received request to change mode to: "
        << requestedModeName << std::endl;

  auto ledModeIter = std::find_if(
    this->allLedModes.begin(),
    this->allLedModes.end(),
    [&](LedMode _mode)
    {
      if (_mode.name == requestedModeName)
      {
        return true;
      }
      return false;
    });

  // If the requested mode was not found
  if (ledModeIter == this->allLedModes.end())
  {
    gzerr << "[LED PLUGIN] Requested LED Mode: " << requestedModeName
          << " was not described" << std::endl;

    _resp.set_data(false);
    return false;
  }

  gzmsg << "[LED PLUGIN] [ON MODE CHANGE] Changing led mode from: "
        << this->currentLedMode.name
        << " to: " << requestedModeName
        << std::endl;

  this->currentLedMode = *(ledModeIter);
  this->currentModeStepIdx = 0;
  this->cycleStartTime = std::chrono::duration<double>::zero();
  gzmsg << "[LED PLUGIN] [ON MODE CHANGE] Current led mode set to: "
        << this->currentLedMode.name << std::endl;

  _resp.set_data(true);

  // Setting LEDs as not ready as we want them
  // to be reset after a mode change
  // This is done because we cannot directly reset
  // the LEDs from here as ECM is required
  this->ledsReady = false;
  return true;
}

GZ_ADD_PLUGIN(LedPlugin,
              System,
              LedPlugin::ISystemConfigure,
              LedPlugin::ISystemPreUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(LedPlugin, "gz::sim::systems::LedPlugin")
