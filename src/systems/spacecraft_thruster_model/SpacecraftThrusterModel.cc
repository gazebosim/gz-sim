/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright (C) 2024 Open Source Robotics Foundation
 * Copyright (C) 2022 Benjamin Perseghetti, Rudis Laboratories
 * Copyright (C) 2024 Pedro Roque, DCS, KTH, Sweden
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

#include "SpacecraftThrusterModel.hh"

#include <mutex>
#include <string>
#include <optional>
#include <chrono>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#define DEBUG 0

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::SpacecraftThrusterModelPrivate
{
  /// \brief Callback for actuator commands.
  public: void OnActuatorMsg(const msgs::Actuators &_msg);

  /// \brief Apply link forces and moments based on propeller state.
  public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Parent link Entity
  public: Entity parentLinkEntity;

  /// \brief Parent link name
  public: std::string parentLinkName;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Topic for actuator commands.
  public: std::string commandSubTopic;

  /// \brief Topic namespace.
  public: std::string robotNamespace;

  /// \brief Simulation time tracker
  public: double simTime = 0.01;

  /// \brief Index of motor in Actuators msg on multirotor_base.
  public: int actuatorNumber = 0;

  /// \brief Duty cycle frequency
  public: double dutyCycleFrequency = 10.0;

  /// \brief Cycle start time
  public: double cycleStartTime = 0.0;

  /// \brief Sampling time with the cycle period.
  public: double samplingTime = 0.01;

  /// \brief Actuator maximum thrust
  public: double maxThrust = 0.0;

  /// \brief Received Actuators message. This is nullopt if no message has been
  /// received.
  public: std::optional<msgs::Actuators> recvdActuatorsMsg;

  /// \brief Mutex to protect recvdActuatorsMsg.
  public: std::mutex recvdActuatorsMsgMutex;

  /// \brief Gazebo communication node.
  public: transport::Node node;
};

//////////////////////////////////////////////////
SpacecraftThrusterModel::SpacecraftThrusterModel()
  : dataPtr(std::make_unique<SpacecraftThrusterModelPrivate>())
{
}

//////////////////////////////////////////////////
void SpacecraftThrusterModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "SpacecraftThrusterModel plugin should be attached to a model "
          << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  this->dataPtr->robotNamespace.clear();

  if (sdfClone->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace =
        sdfClone->Get<std::string>("robotNamespace");
  }
  else
  {
    gzwarn << "No robotNamespace set using entity name.\n";
    this->dataPtr->robotNamespace = this->dataPtr->model.Name(_ecm);
  }

  // Get params from SDF
  if (sdfClone->HasElement("jointName"))
  {
    this->dataPtr->jointName = sdfClone->Get<std::string>("jointName");
  }

  if (this->dataPtr->jointName.empty())
  {
    gzerr << "SpacecraftThrusterModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("linkName"))
  {
    this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");
  }

  if (this->dataPtr->linkName.empty())
  {
    gzerr << "SpacecraftThrusterModel found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("actuatorNumber"))
  {
    this->dataPtr->actuatorNumber =
      sdfClone->GetElement("actuatorNumber")->Get<int>();
  }
  else
  {
    gzerr << "Please specify a actuator_number.\n";
  }

  if (sdfClone->HasElement("maxThrust"))
  {
    this->dataPtr->maxThrust = sdfClone->GetElement("maxThrust")->Get<double>();
  }
  else
  {
    gzerr << "Please specify actuator "
          << this->dataPtr->actuatorNumber <<" maxThrust.\n";
  }

  if (sdfClone->HasElement("dutyCycleFrequency"))
  {
    this->dataPtr->dutyCycleFrequency =
          sdfClone->GetElement("dutyCycleFrequency")->Get<double>();
  }
  else
  {
    gzerr << "Please specify actuator "
          << this->dataPtr->actuatorNumber <<" dutyCycleFrequency.\n";
  }

  sdfClone->Get<std::string>("commandSubTopic",
      this->dataPtr->commandSubTopic, this->dataPtr->commandSubTopic);

  // Subscribe to actuator command messages
  std::string topic = transport::TopicUtils::AsValidTopic(
      this->dataPtr->robotNamespace + "/" + this->dataPtr->commandSubTopic);
  if (topic.empty())
  {
    gzerr << "Failed to create topic for [" << this->dataPtr->robotNamespace
           << "]" << std::endl;
    return;
  }
  else
  {
    gzdbg << "Listening to topic: " << topic << std::endl;
  }
  this->dataPtr->node.Subscribe(topic,
      &SpacecraftThrusterModelPrivate::OnActuatorMsg, this->dataPtr.get());
}

//////////////////////////////////////////////////
void SpacecraftThrusterModelPrivate::OnActuatorMsg(
    const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
  this->recvdActuatorsMsg = _msg;
}

//////////////////////////////////////////////////
void SpacecraftThrusterModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SpacecraftThrusterModel::PreUpdate");
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joint or links haven't been identified yet, look for them
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);

    const auto parentLinkName = _ecm.Component<components::ParentLinkName>(
        this->dataPtr->jointEntity);
    this->dataPtr->parentLinkName = parentLinkName->Data();
  }

  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->parentLinkEntity == kNullEntity)
  {
    this->dataPtr->parentLinkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->parentLinkName);
  }

  if (this->dataPtr->jointEntity == kNullEntity ||
      this->dataPtr->linkEntity == kNullEntity ||
      this->dataPtr->parentLinkEntity == kNullEntity)
    return;

  // skip UpdateForcesAndMoments if needed components are missing
  bool doUpdateForcesAndMoments = true;

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
    doUpdateForcesAndMoments = false;
  }
  if (!_ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
        components::WorldLinearVelocity());
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->parentLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->parentLinkEntity,
        components::WorldPose());
    doUpdateForcesAndMoments = false;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->simTime = std::chrono::duration<double>(_info.simTime).count();
  if (doUpdateForcesAndMoments)
  {
    this->dataPtr->UpdateForcesAndMoments(_ecm);
  }
}

//////////////////////////////////////////////////
void SpacecraftThrusterModelPrivate::UpdateForcesAndMoments(
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("SpacecraftThrusterModelPrivate::UpdateForcesAndMoments");

  std::optional<msgs::Actuators> msg;
  auto actuatorMsgComp =
      _ecm.Component<components::Actuators>(this->model.Entity());

  // Actuators messages can come in from transport or via a component. If a
  // component is available, it takes precedence.
  if (actuatorMsgComp)
  {
    msg = actuatorMsgComp->Data();
  }
  else
  {
    std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
    if (this->recvdActuatorsMsg.has_value())
    {
      msg = *this->recvdActuatorsMsg;
      this->recvdActuatorsMsg.reset();
    }
  }

  if (msg.has_value())
  {
    if (this->actuatorNumber > msg->normalized_size() - 1)
    {
      gzerr << "You tried to access index " << this->actuatorNumber
        << " of the Actuator array which is of size "
        << msg->velocity_size() << std::endl;
      return;
    }
  }
  else
  {
    return;
  }

  // METHOD:
  //
  // targetDutyCycle starts as a normalized value between 0 and 1, so we
  // need to convert it to the corresponding time in the duty cycle
  // period
  //   |________|    |________
  //   |    ^   |    |        |
  // __|    |   |____|        |__
  //   a    b    c   d
  // a: cycle start time
  // b: sampling time
  // c: ref duty cycle
  // d: cycle period
  double targetDutyCycle =
    msg->normalized(this->actuatorNumber) * (1.0 / this->dutyCycleFrequency);
  if (DEBUG && this->actuatorNumber == 0)
    std::cout << this->actuatorNumber
              << ": target duty cycle: " << targetDutyCycle << std::endl;

  // Calculate cycle start time
  if (this->samplingTime >= 1.0/this->dutyCycleFrequency) {
    if (DEBUG && this->actuatorNumber == 0)
      std::cout << this->actuatorNumber
                << ": Cycle completed. Resetting cycle start time."
                << std::endl;
    this->cycleStartTime = this->simTime;
  }

  // Calculate sampling time instant within the cycle
  this->samplingTime = this->simTime - this->cycleStartTime;
  if (DEBUG && this->actuatorNumber == 0)
    std::cout << this->actuatorNumber
              << ": PWM Period: " << 1.0/this->dutyCycleFrequency
              << " Cycle Start time: " << this->cycleStartTime
              << " Sampling time: " << this->samplingTime << std::endl;

  // Apply force if the sampling time is less than the target ON duty cycle
  double force = this->samplingTime <= targetDutyCycle ? this->maxThrust : 0.0;
  if (DEBUG && this->actuatorNumber == 0)
    std::cout << this->actuatorNumber
              << ": Force: " << force
              << "  Sampling time: " << this->samplingTime
              << "  Tgt duty cycle: " << targetDutyCycle << std::endl;

  // Apply force to the link
  Link link(this->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);
  link.AddWorldForce(_ecm,
    worldPose->Rot().RotateVector(math::Vector3d(0, 0, force)));
  if (DEBUG && this->actuatorNumber == 0)
    std::cout << this->actuatorNumber
              << ": Input Value: " << msg->normalized(this->actuatorNumber)
              << "  Calc. Force: " << force << std::endl;
}

GZ_ADD_PLUGIN(SpacecraftThrusterModel,
                    System,
                    SpacecraftThrusterModel::ISystemConfigure,
                    SpacecraftThrusterModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(SpacecraftThrusterModel,
                          "gz::sim::systems::SpacecraftThrusterModel")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(SpacecraftThrusterModel,
                          "ignition::gazebo::systems::SpacecraftThrusterModel")
