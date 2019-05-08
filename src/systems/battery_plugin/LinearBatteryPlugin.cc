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

#include "LinearBatteryPlugin.hh"

#include <string>
#include <functional>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Util.hh>
#include <ignition/common/Battery.hh>

#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/BatterySoC.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinearBatteryPluginPrivate
{
  /// \brief Reset the plugin
  public: void Reset();

  /// \brief Get the current state of charge of the battery.
  /// \return State of charge of the battery in range [0.0, 1.0].
  public: double StateOfCharge() const;

  /// \brief Name of model, only used for printing warning when battery drains.
  public: std::string modelName;

  /// \brief Pointer to battery contained in link.
  public: common::BatteryPtr battery;

  /// \brief Whether warning that battery has drained has been printed once.
  public: bool drainPrinted{false};

  /// \brief Battery consumer identifier.
  /// Current implementation limits one consumer (Model) per battery.
  public: int32_t consumerId;

  /// \brief Battery entity
  public: Entity batteryEntity{kNullEntity};

  /// \brief Open-circuit voltage.
  /// E(t) = e0 + e1 * Q(t) / c
  public: double e0{0.0};
  public: double e1{0.0};

  /// \brief Initial battery charge in Ah.
  public: double q0{0.0};

  /// \brief Battery capacity in Ah.
  public: double c{0.0};

  /// \brief Battery inner resistance in Ohm.
  public: double r{0.0};

  /// \brief Current low-pass filter characteristic time in seconds.
  public: double tau{0.0};

  /// \brief Raw battery current in A.
  public: double iraw{0.0};

  /// \brief Smoothed battery current in A.
  public: double ismooth{0.0};

  /// \brief Instantaneous battery charge in Ah.
  public: double q{0.0};

  /// \brief State of charge
  public: double soc{1.0};

  /// \brief Battery current for a historic time window
  public: std::deque<double> iList;

  /// \brief Time interval for a historic time window
  public: std::deque<double> dtList;

  /// \brief Simulation time handled during a single update.
  public: std::chrono::steady_clock::duration stepSize;
};

/////////////////////////////////////////////////
LinearBatteryPlugin::LinearBatteryPlugin()
    : System(), dataPtr(std::make_unique<LinearBatteryPluginPrivate>())
{
}

/////////////////////////////////////////////////
LinearBatteryPlugin::~LinearBatteryPlugin()
{
  this->dataPtr->Reset();

  if (this->dataPtr->battery)
  {
    // Consumer-specific
    if (this->dataPtr->consumerId != -1)
    {
      this->dataPtr->battery->RemoveConsumer(this->dataPtr->consumerId);
    }

    // This is needed so that common::Battery stops calling the update function
    //   of this object, when this object is destroyed. Else seg fault in test,
    //   though no seg fault in actual run.
    this->dataPtr->battery->ResetUpdateFunc();
  }
}

/////////////////////////////////////////////////
void LinearBatteryPlugin::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  // Store the pointer to the model this battery is under
  Model model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Linear battery plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->modelName = model.Name(_ecm);

  if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
    this->dataPtr->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

  if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
    this->dataPtr->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

  if (_sdf->HasElement("initial_charge"))
  {
    this->dataPtr->q0 = _sdf->Get<double>("initial_charge");
    this->dataPtr->q = this->dataPtr->q0;
  }

  if (_sdf->HasElement("capacity"))
    this->dataPtr->c = _sdf->Get<double>("capacity");

  if (_sdf->HasElement("resistance"))
    this->dataPtr->r = _sdf->Get<double>("resistance");

  if (_sdf->HasElement("smooth_current_tau"))
    this->dataPtr->tau = _sdf->Get<double>("smooth_current_tau");

  if (_sdf->HasElement("battery_name") && _sdf->HasElement("voltage"))
  {
    auto batteryName = _sdf->Get<std::string>("battery_name");
    auto initVoltage = _sdf->Get<double>("voltage");

    // Create battery entity and component
    this->dataPtr->batteryEntity = _ecm.CreateEntity();
    // Initialize with initial voltage
    _ecm.CreateComponent(this->dataPtr->batteryEntity,
      components::BatterySoC());
    _ecm.CreateComponent(this->dataPtr->batteryEntity, components::Name(
      batteryName));
    _ecm.SetParentEntity(this->dataPtr->batteryEntity, _entity);

    // Create actual battery and assign update function
    this->dataPtr->battery = std::make_shared<common::Battery>(batteryName,
      initVoltage);
    this->dataPtr->battery->Init();
    this->dataPtr->battery->SetUpdateFunc(
      std::bind(&LinearBatteryPlugin::OnUpdateVoltage, this,
        std::placeholders::_1));
  }
  else
  {
    ignerr << "No <battery_name> or <voltage> specified. Both are required.\n";
  }

  // Consumer-specific
  if (_sdf->HasElement("power_load"))
  {
    auto powerLoad = _sdf->Get<double>("power_load");
    this->dataPtr->consumerId = this->dataPtr->battery->AddConsumer();
    bool success = this->dataPtr->battery->SetPowerLoad(
      this->dataPtr->consumerId, powerLoad);
    if (!success)
      ignerr << "Failed to set consumer power load." << std::endl;
  }
  else
  {
    ignwarn << "Required attribute power_load missing "
            << "in LinearBatteryPlugin SDF" << std::endl;
  }

  ignmsg << "LinearBatteryPlugin configured. Battery name: "
         << this->dataPtr->battery->Name() << std::endl;
  igndbg << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage()
         << std::endl;
}

/////////////////////////////////////////////////
void LinearBatteryPluginPrivate::Reset()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->q = this->q0;
}

/////////////////////////////////////////////////
double LinearBatteryPluginPrivate::StateOfCharge() const
{
  return this->soc;
}

//////////////////////////////////////////////////
void LinearBatteryPlugin::Update(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Update actual battery
  this->dataPtr->stepSize = _info.dt;
  if (this->dataPtr->battery)
  {
    this->dataPtr->battery->Update();

    // Update component
    auto batteryComp =
      _ecm.Component<components::BatterySoC>(this->dataPtr->batteryEntity);
    batteryComp->Data() = this->dataPtr->StateOfCharge();
  }
}

/////////////////////////////////////////////////
double LinearBatteryPlugin::OnUpdateVoltage(
  const common::Battery *_battery)
{
  IGN_ASSERT(_battery != nullptr, "common::Battery is null.");

  if (fabs(_battery->Voltage()) < 1e-3)
    return 0.0;
  if (this->dataPtr->StateOfCharge() < 0)
    return _battery->Voltage();

  auto prevSocInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

  // Seconds
  double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
    this->dataPtr->stepSize).count()) * 1e-9;
  double totalpower = 0.0;
  double k = dt / this->dataPtr->tau;

  for (auto powerLoad : _battery->PowerLoads())
    totalpower += powerLoad.second;

  this->dataPtr->iraw = totalpower / _battery->Voltage();

  this->dataPtr->ismooth = this->dataPtr->ismooth + k *
    (this->dataPtr->iraw - this->dataPtr->ismooth);

  // Keep a list of historic currents and time intervals
  if (this->dataPtr->iList.size() >= 100)
  {
    this->dataPtr->iList.pop_front();
    this->dataPtr->dtList.pop_front();
  }
  this->dataPtr->iList.push_back(this->dataPtr->ismooth);
  this->dataPtr->dtList.push_back(dt);

  // Convert dt to hours
  this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) /
    3600.0);

  double voltage = this->dataPtr->e0 + this->dataPtr->e1 * (
    1 - this->dataPtr->q / this->dataPtr->c)
      - this->dataPtr->r * this->dataPtr->ismooth;

  // Estimate state of charge
  double isum = 0.0;
  for (size_t i = 0; i < this->dataPtr->iList.size(); ++i)
    isum += (this->dataPtr->iList[i] * this->dataPtr->dtList[i] / 3600.0);
  this->dataPtr->soc = this->dataPtr->soc - isum / this->dataPtr->c;

  // Throttle debug messages
  auto socInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);
  if (socInt % 10 == 0 && socInt != prevSocInt)
  {
    igndbg << "Battery: " << this->dataPtr->battery->Name() << std::endl;
    igndbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
           << std::endl;
    igndbg << "voltage: " << voltage << std::endl;
    igndbg << "state of charge: " << this->dataPtr->StateOfCharge()
           << " (q " << this->dataPtr->q << ")" << std::endl << std::endl;
  }
  if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->drainPrinted)
  {
    ignwarn << "Model " << this->dataPtr->modelName << " out of battery.\n";
    this->dataPtr->drainPrinted = true;
  }

  return voltage;
}

IGNITION_ADD_PLUGIN(LinearBatteryPlugin,
                    ignition::gazebo::System,
                    LinearBatteryPlugin::ISystemConfigure,
                    LinearBatteryPlugin::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinearBatteryPlugin,
  "ignition::gazebo::systems::LinearBatteryPlugin")
