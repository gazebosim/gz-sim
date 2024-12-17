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

#include <gz/msgs/battery_state.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <algorithm>
#include <atomic>
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include <gz/common/Battery.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "gz/sim/components/BatteryPowerLoad.hh"
#include "gz/sim/components/BatterySoC.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LinearBatteryPluginPrivate
{
  /// \brief Reset the plugin
  public: void Reset();

  /// \brief Get the current state of charge of the battery.
  /// \return State of charge of the battery in range [0.0, 1.0].
  public: double StateOfCharge() const;

  /// \brief Callback executed to start recharging.
  /// \param[in] _req This value should be true.
  public: void OnEnableRecharge(const msgs::Boolean &_req);

  /// \brief Callback executed to stop recharging.
  /// \param[in] _req This value should be true.
  public: void OnDisableRecharge(const msgs::Boolean &_req);

  /// \brief Callback connected to additional topics that can start battery
  /// draining.
  /// \param[in] _data Message data.
  /// \param[in] _size Message data size.
  /// \param[in] _info Information about the message.
  public: void OnBatteryDrainingMsg(
    const char *_data, const size_t _size,
    const gz::transport::MessageInfo &_info);

  /// \brief Callback connected to additional topics that can stop battery
  /// draining.
  /// \param[in] _data Message data.
  /// \param[in] _size Message data size.
  /// \param[in] _info Information about the message.
  public: void OnBatteryStopDrainingMsg(
    const char *_data, const size_t _size,
    const gz::transport::MessageInfo &_info);

  /// \brief Name of model, only used for printing warning when battery drains.
  public: std::string modelName;

  /// \brief Name that identifies a battery.
  public: std::string batteryName;

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

  /// \brief Current low-pass filter characteristic time in seconds [0, 1].
  public: double tau{1.0};

  /// \brief Raw battery current in A.
  public: double iraw{0.0};

  /// \brief Smoothed battery current in A.
  public: double ismooth{0.0};

  /// \brief Instantaneous battery charge in Ah.
  public: double q{0.0};

  /// \brief State of charge [0, 1].
  public: double soc{1.0};

  /// \brief Recharge status
  public: std::atomic_bool startCharging{false};

  /// \brief Hours taken to fully charge battery
  public: double tCharge{0.0};

  /// \TODO(caguero) Remove this flag in Gazebo Dome.
  /// \brief Flag to enable some battery fixes.
  public: bool fixIssue225{false};

  /// \TODO(caguero) Remove in Gazebo Dome.
  /// \brief Battery current for a historic time window
  public: std::deque<double> iList;

  /// \TODO(caguero) Remove in Gazebo Dome.
  /// \brief Time interval for a historic time window
  public: std::deque<double> dtList;

  /// \brief Simulation time handled during a single update.
  public: std::chrono::steady_clock::duration stepSize;

  /// \brief Flag on whether the battery should start draining
  public: bool startDraining = false;

  /// \brief The start time when battery starts draining in seconds
  public: int drainStartTime = -1;

  /// \brief Book keep the last time printed, so as to not pollute dbg messages
  /// in minutes
  public: int lastPrintTime = -1;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Gazebo communication node
  public: transport::Node node;

  /// \brief Battery state of charge message publisher
  public: transport::Node::Publisher statePub;

  /// \brief Initial power load set trough config
  public: double initialPowerLoad = 0.0;

  /// \brief Flag to invert the current sign
  public: bool invertCurrentSign{false};
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
  auto model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "Linear battery plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->model = model;
  this->dataPtr->modelName = model.Name(_ecm);

  if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
    this->dataPtr->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

  if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
    this->dataPtr->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

  if (_sdf->HasElement("capacity"))
    this->dataPtr->c = _sdf->Get<double>("capacity");

  if (this->dataPtr->c <= 0)
  {
    gzerr << "No <capacity> or incorrect value specified. Capacity should be "
           << "greater than 0.\n";
    return;
  }

  this->dataPtr->q0 = this->dataPtr->c;
  if (_sdf->HasElement("initial_charge"))
  {
    this->dataPtr->q0 = _sdf->Get<double>("initial_charge");
    if (this->dataPtr->q0 > this->dataPtr->c || this->dataPtr->q0 < 0)
    {
      gzerr << "<initial_charge> value should be between [0, <capacity>]."
             << std::endl;
      this->dataPtr->q0 =
        std::max(0.0, std::min(this->dataPtr->q0, this->dataPtr->c));
      gzerr << "Setting <initial_charge> to [" << this->dataPtr->q0
             << "] instead." << std::endl;
    }
  }

  this->dataPtr->q = this->dataPtr->q0;

  if (_sdf->HasElement("resistance"))
    this->dataPtr->r = _sdf->Get<double>("resistance");

  if (_sdf->HasElement("smooth_current_tau"))
  {
    this->dataPtr->tau = _sdf->Get<double>("smooth_current_tau");
    if (this->dataPtr->tau <= 0)
    {
      gzerr << "<smooth_current_tau> value should be positive. "
             << "Using [1] instead." << std::endl;
      this->dataPtr->tau = 1;
    }
  }

  if (_sdf->HasElement("fix_issue_225"))
    this->dataPtr->fixIssue225 = _sdf->Get<bool>("fix_issue_225");

  if (_sdf->HasElement("invert_current_sign"))
    this->dataPtr->invertCurrentSign =
      _sdf->Get<bool>("invert_current_sign");

  if (_sdf->HasElement("battery_name") && _sdf->HasElement("voltage"))
  {
    this->dataPtr->batteryName = _sdf->Get<std::string>("battery_name");
    auto initVoltage = _sdf->Get<double>("voltage");

    // Create battery entity and some components
    this->dataPtr->batteryEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(this->dataPtr->batteryEntity, components::Name(
      this->dataPtr->batteryName));
    _ecm.SetParentEntity(this->dataPtr->batteryEntity, _entity);

    // Create actual battery and assign update function
    this->dataPtr->battery = std::make_shared<common::Battery>(
      this->dataPtr->batteryName, initVoltage);
    this->dataPtr->battery->Init();
    this->dataPtr->battery->SetUpdateFunc(
      std::bind(&LinearBatteryPlugin::OnUpdateVoltage, this,
        std::placeholders::_1));
  }
  else
  {
    gzerr << "No <battery_name> or <voltage> specified. Both are required.\n";
    return;
  }

  if (_sdf->HasElement("enable_recharge"))
  {
    auto isCharging = _sdf->Get<bool>("enable_recharge");
    if (isCharging)
    {
      if (_sdf->HasElement("charging_time"))
        this->dataPtr->tCharge = _sdf->Get<double>("charging_time");
      else
      {
        gzerr << "No <charging_time> specified. "
                  "Parameter required to enable recharge.\n";
        return;
      }

      std::string enableRechargeTopic = "/model/" + this->dataPtr->modelName +
        "/battery/" + _sdf->Get<std::string>("battery_name") +
        "/recharge/start";
      std::string disableRechargeTopic = "/model/" + this->dataPtr->modelName +
        "/battery/" + _sdf->Get<std::string>("battery_name") +
        "/recharge/stop";

      auto validEnableRechargeTopic = transport::TopicUtils::AsValidTopic(
          enableRechargeTopic);
      auto validDisableRechargeTopic = transport::TopicUtils::AsValidTopic(
          disableRechargeTopic);
      if (validEnableRechargeTopic.empty() || validDisableRechargeTopic.empty())
      {
        gzerr << "Failed to create valid topics. Not valid: ["
               << enableRechargeTopic << "] and [" << disableRechargeTopic
               << "]" << std::endl;
        return;
      }

      this->dataPtr->node.Advertise(validEnableRechargeTopic,
        &LinearBatteryPluginPrivate::OnEnableRecharge, this->dataPtr.get());
      this->dataPtr->node.Advertise(validDisableRechargeTopic,
        &LinearBatteryPluginPrivate::OnDisableRecharge, this->dataPtr.get());

      if (_sdf->HasElement("recharge_by_topic"))
      {
        this->dataPtr->node.Subscribe(validEnableRechargeTopic,
          &LinearBatteryPluginPrivate::OnEnableRecharge, this->dataPtr.get());
        this->dataPtr->node.Subscribe(validDisableRechargeTopic,
          &LinearBatteryPluginPrivate::OnDisableRecharge, this->dataPtr.get());
      }
    }
  }

  // Consumer-specific
  if (_sdf->HasElement("power_load"))
  {
    this->dataPtr->initialPowerLoad = _sdf->Get<double>("power_load");
    this->dataPtr->consumerId = this->dataPtr->battery->AddConsumer();
    bool success = this->dataPtr->battery->SetPowerLoad(
      this->dataPtr->consumerId, this->dataPtr->initialPowerLoad);
    if (!success)
      gzerr << "Failed to set consumer power load." << std::endl;
  }
  else
  {
    gzwarn << "Required attribute power_load missing "
            << "in LinearBatteryPlugin SDF" << std::endl;
  }

  if (_sdf->HasElement("start_draining"))
    this->dataPtr->startDraining = _sdf->Get<bool>("start_draining");

  // Subscribe to power draining topics, if any.
  if (_sdf->HasElement("power_draining_topic"))
  {
    sdf::ElementConstPtr sdfElem = _sdf->FindElement("power_draining_topic");
    while (sdfElem)
    {
      const auto &topic = sdfElem->Get<std::string>();
      this->dataPtr->node.SubscribeRaw(topic,
          std::bind(&LinearBatteryPluginPrivate::OnBatteryDrainingMsg,
          this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3));
      gzmsg << "LinearBatteryPlugin subscribes to power draining topic ["
             << topic << "]." << std::endl;
      sdfElem = sdfElem->GetNextElement("power_draining_topic");
    }
  }

  // Subscribe to stop power draining topics, if any.
  if (_sdf->HasElement("stop_power_draining_topic"))
  {
    sdf::ElementConstPtr sdfElem =
        _sdf->FindElement("stop_power_draining_topic");
    while (sdfElem)
    {
      const auto &topic = sdfElem->Get<std::string>();
      this->dataPtr->node.SubscribeRaw(topic,
          std::bind(&LinearBatteryPluginPrivate::OnBatteryStopDrainingMsg,
          this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3));
      gzmsg << "LinearBatteryPlugin subscribes to stop power draining topic ["
            << topic << "]." << std::endl;
      sdfElem = sdfElem->GetNextElement("power_draining_topic");
    }
  }

  gzmsg << "LinearBatteryPlugin configured. Battery name: "
         << this->dataPtr->battery->Name() << std::endl;
  gzdbg << "Battery initial voltage: " << this->dataPtr->battery->InitVoltage()
         << std::endl;

  this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;
  // Initialize battery with initial calculated state of charge
  _ecm.CreateComponent(this->dataPtr->batteryEntity,
      components::BatterySoC(this->dataPtr->soc));

  // Setup battery state topic
  std::string stateTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/battery/" + this->dataPtr->battery->Name() + "/state"};

  auto validStateTopic = transport::TopicUtils::AsValidTopic(stateTopic);
  if (validStateTopic.empty())
  {
    gzerr << "Failed to create valid state topic ["
           << stateTopic << "]" << std::endl;
    return;
  }

  transport::AdvertiseMessageOptions opts;
  opts.SetMsgsPerSec(50);
  this->dataPtr->statePub = this->dataPtr->node.Advertise<msgs::BatteryState>(
    validStateTopic, opts);
}

/////////////////////////////////////////////////
void LinearBatteryPluginPrivate::Reset()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->q = this->q0;
  this->startDraining = false;
}

/////////////////////////////////////////////////
double LinearBatteryPluginPrivate::StateOfCharge() const
{
  return this->soc;
}

//////////////////////////////////////////////////
void LinearBatteryPluginPrivate::OnEnableRecharge(
  const msgs::Boolean &/*_req*/)
{
  gzdbg << "Request for start charging received" << std::endl;
  this->startCharging = true;
}

//////////////////////////////////////////////////
void LinearBatteryPluginPrivate::OnDisableRecharge(
  const msgs::Boolean &/*_req*/)
{
  gzdbg << "Request for stop charging received" << std::endl;
  this->startCharging = false;
}

//////////////////////////////////////////////////
void LinearBatteryPluginPrivate::OnBatteryDrainingMsg(
  const char *, const size_t, const gz::transport::MessageInfo &)
{
  this->startDraining = true;
}

//////////////////////////////////////////////////
void LinearBatteryPluginPrivate::OnBatteryStopDrainingMsg(
  const char *, const size_t, const gz::transport::MessageInfo &)
{
  this->startDraining = false;
}

//////////////////////////////////////////////////
void LinearBatteryPlugin::PreUpdate(
  const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("LinearBatteryPlugin::PreUpdate");

  if (!this->dataPtr->battery)
    return;

  // Recalculate the total power load among consumers
  double total_power_load = this->dataPtr->initialPowerLoad;
  _ecm.Each<components::BatteryPowerLoad>(
    [&](const Entity & /*_entity*/,
        const components::BatteryPowerLoad *_batteryPowerLoadInfo)->bool
    {
      if (_batteryPowerLoadInfo->Data().batteryId ==
          this->dataPtr->batteryEntity)
      {
        total_power_load = total_power_load +
            _batteryPowerLoadInfo->Data().batteryPowerLoad;
      }
      return true;
    });

  bool success = this->dataPtr->battery->SetPowerLoad(
      this->dataPtr->consumerId, total_power_load);
  if (!success)
      gzerr << "Failed to set consumer power load." << std::endl;

  // Start draining the battery if the robot has started moving
  if (!this->dataPtr->startDraining)
  {
    const std::vector<Entity> joints = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(),
      components::Joint());

    for (Entity jointEntity : joints)
    {
      const auto *jointVelocityCmd =
        _ecm.Component<components::JointVelocityCmd>(jointEntity);
      if (jointVelocityCmd)
      {
        for (double jointVel : jointVelocityCmd->Data())
        {
          if (fabsf(static_cast<float>(jointVel)) > 0)
          {
            this->dataPtr->startDraining = true;
            return;
          }
        }
      }

      const auto *jointForceCmd =
        _ecm.Component<components::JointForceCmd>(jointEntity);
      if (jointForceCmd)
      {
        for (double jointForce : jointForceCmd->Data())
        {
          if (fabsf(static_cast<float>(jointForce)) > 0)
          {
            this->dataPtr->startDraining = true;
            return;
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void LinearBatteryPlugin::Update(const UpdateInfo &_info,
                                 EntityComponentManager &_ecm)
{
  GZ_PROFILE("LinearBatteryPlugin::Update");

  if (!this->dataPtr->battery)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (_info.paused)
    return;

  if (!this->dataPtr->startDraining && !this->dataPtr->startCharging)
    return;

  // Find the time at which battery starts to drain
  int simTime = static_cast<int>(
    std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
  if (this->dataPtr->drainStartTime == -1)
    this->dataPtr->drainStartTime = simTime;

  // Print drain time in minutes
  int drainTime = (simTime - this->dataPtr->drainStartTime) / 60;
  if (drainTime != this->dataPtr->lastPrintTime)
  {
    this->dataPtr->lastPrintTime = drainTime;
    gzdbg << "[Battery Plugin] Battery drain: " << drainTime <<
      " minutes passed.\n";
  }

  // Update actual battery
  this->dataPtr->stepSize = _info.dt;

  // Sanity check: tau should be between [dt, +inf).
  double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
    this->dataPtr->stepSize).count()) * 1e-9;
  if (this->dataPtr->tau < dt)
  {
    gzerr << "<smooth_current_tau> should be in the range [dt, +inf) but is "
           << "configured with [" << this->dataPtr->tau << "]. We'll be using "
           << "[" << dt << "] instead" << std::endl;
    this->dataPtr->tau = dt;
  }

  if (this->dataPtr->battery)
  {
    this->dataPtr->battery->Update();

    // Update component
    auto *batteryComp =
      _ecm.Component<components::BatterySoC>(this->dataPtr->batteryEntity);

    batteryComp->Data() = this->dataPtr->StateOfCharge();
  }
}

//////////////////////////////////////////////////
void LinearBatteryPlugin::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("LinearBatteryPlugin::PostUpdate");
  // Nothing left to do if paused or the publisher wasn't created.
  if (_info.paused || !this->dataPtr->statePub)
    return;

  if (!this->dataPtr->battery)
    return;

  // Publish battery state
  msgs::BatteryState msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
  msg.set_voltage(this->dataPtr->battery->Voltage());
  if (this->dataPtr->invertCurrentSign)
    msg.set_current(-this->dataPtr->ismooth);
  else
    msg.set_current(this->dataPtr->ismooth);
  msg.set_charge(this->dataPtr->q);
  msg.set_capacity(this->dataPtr->c);

  if (this->dataPtr->fixIssue225)
    msg.set_percentage(this->dataPtr->soc * 100);
  else
    msg.set_percentage(this->dataPtr->soc);

  if (this->dataPtr->startCharging)
    msg.set_power_supply_status(msgs::BatteryState::CHARGING);
  else if (this->dataPtr->startDraining)
    msg.set_power_supply_status(msgs::BatteryState::DISCHARGING);
  else if (this->dataPtr->StateOfCharge() > 0.9)
    msg.set_power_supply_status(msgs::BatteryState::FULL);
  else
    msg.set_power_supply_status(msgs::BatteryState::NOT_CHARGING);
  this->dataPtr->statePub.Publish(msg);
}

/////////////////////////////////////////////////
double LinearBatteryPlugin::OnUpdateVoltage(
  const common::Battery *_battery)
{
  GZ_ASSERT(_battery != nullptr, "common::Battery is null.");

  if (fabs(_battery->Voltage()) < 1e-3 && !this->dataPtr->startCharging)
    return 0.0;
  if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->startCharging)
    return _battery->Voltage();

  auto prevSocInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);

  // Seconds
  double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
    this->dataPtr->stepSize).count()) * 1e-9;
  double totalpower = 0.0;
  double k = dt / this->dataPtr->tau;

  if (this->dataPtr->startDraining)
  {
    for (auto powerLoad : _battery->PowerLoads())
      totalpower += powerLoad.second;
  }

  this->dataPtr->iraw = totalpower / _battery->Voltage();

  // compute charging current
  auto iCharge = this->dataPtr->c / this->dataPtr->tCharge;

  // add charging current to battery
  if (this->dataPtr->startCharging && this->dataPtr->StateOfCharge() < 0.9)
    this->dataPtr->iraw -= iCharge;

  this->dataPtr->ismooth = this->dataPtr->ismooth + k *
    (this->dataPtr->iraw - this->dataPtr->ismooth);

  if (!this->dataPtr->fixIssue225)
  {
    if (this->dataPtr->iList.size() >= 100)
    {
      this->dataPtr->iList.pop_front();
      this->dataPtr->dtList.pop_front();
    }
    this->dataPtr->iList.push_back(this->dataPtr->ismooth);
    this->dataPtr->dtList.push_back(dt);
  }

  // Convert dt to hours
  this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) /
    3600.0);

  // open circuit voltage
  double voltage = this->dataPtr->e0 + this->dataPtr->e1 * (
    1 - this->dataPtr->q / this->dataPtr->c)
      - this->dataPtr->r * this->dataPtr->ismooth;

  // Estimate state of charge
  if (this->dataPtr->fixIssue225)
    this->dataPtr->soc = this->dataPtr->q / this->dataPtr->c;
  else
  {
    double isum = 0.0;
    for (size_t i = 0; i < this->dataPtr->iList.size(); ++i)
      isum += (this->dataPtr->iList[i] * this->dataPtr->dtList[i] / 3600.0);
    this->dataPtr->soc = this->dataPtr->soc - isum / this->dataPtr->c;
  }

  // Throttle debug messages
  auto socInt = static_cast<int>(this->dataPtr->StateOfCharge() * 100);
  if (socInt % 10 == 0 && socInt != prevSocInt)
  {
    gzdbg << "Battery: " << this->dataPtr->battery->Name() << std::endl;
    gzdbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
           << std::endl;
    gzdbg << "charging status: " << std::boolalpha
           << this->dataPtr->startCharging << std::endl;
    gzdbg << "charging current: " << iCharge << std::endl;
    gzdbg << "voltage: " << voltage << std::endl;
    gzdbg << "state of charge: " << this->dataPtr->StateOfCharge()
           << " (q " << this->dataPtr->q << ")" << std::endl << std::endl;
  }
  if (this->dataPtr->StateOfCharge() < 0 && !this->dataPtr->drainPrinted)
  {
    gzwarn << "Model " << this->dataPtr->modelName << " out of battery.\n";
    this->dataPtr->drainPrinted = true;
  }

  return voltage;
}

GZ_ADD_PLUGIN(LinearBatteryPlugin,
                    System,
                    LinearBatteryPlugin::ISystemConfigure,
                    LinearBatteryPlugin::ISystemPreUpdate,
                    LinearBatteryPlugin::ISystemUpdate,
                    LinearBatteryPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(LinearBatteryPlugin,
  "gz::sim::systems::LinearBatteryPlugin")
