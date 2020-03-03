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

#include <ignition/msgs/actuators.pb.h>
#include <ignition/msgs/twist.pb.h>

#include <limits>

#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/math/Inertial.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Actuators.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "MulticopterVelocityControl.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;
using namespace multicopter_control;

//////////////////////////////////////////////////
void MulticopterVelocityControl::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->model = Model(_entity);

  if (!this->model.Valid(_ecm))
  {
    ignerr << "MulticopterVelocityControl plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement("comLinkName"))
  {
    this->comLinkName = sdfClone->Get<std::string>("comLinkName");
  }

  if (this->comLinkName.empty())
  {
    ignerr << "found an empty comLinkName parameter. Failed to initialize.\n";
    return;
  }

  // Get the link entity
  this->comLinkEntity = this->model.LinkByName(_ecm, this->comLinkName);

  if (this->comLinkEntity == kNullEntity)
  {
    ignerr << "Link " << this->comLinkName
           << " could not be found. Failed to initialize.\n";
    return;
  }

  createFrameDataComponents(_ecm, this->comLinkEntity);

  VehicleParameters vehicleParams;

  math::Inertiald vehicleInertial;
  // Compute the vehicle's moment of inertia and mass assuming that all the
  // links in the model belong to the vehicle.
  for (const Entity &link :
       _ecm.ChildrenByComponents(this->model.Entity(), components::Link()))
  {
    auto inertial = _ecm.Component<components::Inertial>(link);
    if (nullptr == inertial)
    {
      ignerr << "Could not find inertial component on on link "
             << this->comLinkName << std::endl;
      return;
    }
    vehicleInertial += inertial->Data();
  }

  vehicleParams.mass = vehicleInertial.MassMatrix().Mass();
  vehicleParams.inertia = math::eigen3::convert(vehicleInertial.Moi());
  if (sdfClone->HasElement("rotorConfiguration"))
  {
    vehicleParams.rotorConfiguration =
        loadRotorConfiguration(_ecm, sdfClone->GetElement("rotorConfiguration"),
                               this->model, this->comLinkEntity);
    // DEBUG
    // std::cout << "Found "
    //           << vehicleParams.rotorConfiguration.size()
    //           << " rotors" << std::endl;
    // for (const auto &rotor : vehicleParams.rotorConfiguration)
    // {
    //   std::cout << rotor.angle << " "
    //             << rotor.armLength << " " << rotor.forceConstant << " "
    //             << rotor.momentConstant << " " << rotor.direction << "\n\n";
    // }
  }
  else
  {
    ignerr << "Please specify rotorConfiguration.\n";
  }

  this->rotorVelocities.resize(vehicleParams.rotorConfiguration.size());

  auto worldEntity = _ecm.EntityByComponents(components::World());

  if (kNullEntity == worldEntity)
  {
    ignerr << "World entity missing." << std::endl;
    return;
  }

  // Get the world acceleration (defined in world frame)
  auto gravityComp = _ecm.Component<components::Gravity>(worldEntity);

  if (nullptr == gravityComp)
  {
    ignerr << "World missing gravity." << std::endl;
    return;
  }

  vehicleParams.gravity = math::eigen3::convert(gravityComp->Data());

  LeeVelocityControllerParameters controllerParameters;

  if (sdfClone->HasElement("velocityGain"))
  {
    controllerParameters.velocityGain =
        math::eigen3::convert(sdfClone->Get<math::Vector3d>("velocityGain"));
  }
  else
  {
    ignerr << "Please specify velocityGain for MulticopterVelocityControl.\n";
    return;
  }

  if (sdfClone->HasElement("attitudeGain"))
  {
    controllerParameters.attitudeGain =
        math::eigen3::convert(sdfClone->Get<math::Vector3d>("attitudeGain"));
  }
  else
  {
    ignerr << "Please specify attitudeGain for MulticopterVelocityControl.\n";
    return;
  }

  if (sdfClone->HasElement("angularRateGain"))
  {
    controllerParameters.angularRateGain =
        math::eigen3::convert(sdfClone->Get<math::Vector3d>("angularRateGain"));
  }
  else
  {
    ignerr << "Please specify angularRateGain MulticopterVelocityControl.\n";
    return;
  }

  if (sdfClone->HasElement("maximumLinearAcceleration"))
  {
    controllerParameters.maxLinearAcceleration = math::eigen3::convert(
        sdfClone->Get<math::Vector3d>("maximumLinearAcceleration"));
  }
  else
  {
    controllerParameters.maxLinearAcceleration.setConstant(
        std::numeric_limits<double>::max());
  }

  if (sdfClone->HasElement("maximumLinearVelocity"))
  {
    this->maximumLinearVelocity =
        sdfClone->Get<math::Vector3d>("maximumLinearVelocity").Abs();
  }
  else
  {
    this->maximumLinearVelocity.Set(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
  }

  if (sdfClone->HasElement("maximumAngularVelocity"))
  {
    this->maximumAngularVelocity =
        sdfClone->Get<math::Vector3d>("maximumAngularVelocity").Abs();
  }
  else
  {
    this->maximumAngularVelocity.Set(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max());
  }

  this->velocityController = LeeVelocityController::MakeController(
      controllerParameters, vehicleParams);

  if (nullptr == this->velocityController)
  {
    ignerr << "Error while creating the LeeVelocityController\n";
    return;
  }

  math::Vector3d linearVelocityMean{0, 0, 0};
  sdfClone->Get<math::Vector3d>("linearVelocityNoiseMean",
      linearVelocityMean, linearVelocityMean);

  math::Vector3d linearVelocityStdDev{0, 0, 0};
  sdfClone->Get<math::Vector3d>("linearVelocityNoiseStdDev",
      linearVelocityStdDev, linearVelocityStdDev);

  math::Vector3d angularVelocityMean{0, 0, 0};
  sdfClone->Get<math::Vector3d>("angularVelocityNoiseMean",
      angularVelocityMean, angularVelocityMean);

  math::Vector3d angularVelocityStdDev{0, 0, 0};
  sdfClone->Get<math::Vector3d>("angularVelocityNoiseStdDev",
      angularVelocityStdDev, angularVelocityStdDev);

  this->noiseParameters.linearVelocityMean =
      math::eigen3::convert(linearVelocityMean);
  this->noiseParameters.linearVelocityStdDev =
      math::eigen3::convert(linearVelocityStdDev);
  this->noiseParameters.angularVelocityMean =
      math::eigen3::convert(angularVelocityMean);
  this->noiseParameters.angularVelocityStdDev =
      math::eigen3::convert(angularVelocityStdDev);

  if (sdfClone->HasElement("robotNamespace"))
  {
    this->robotNamespace =
        sdfClone->Get<std::string>("robotNamespace");
  }
  else
  {
    ignerr << "Please specify a robotNamespace.\n";
    return;
  }

  sdfClone->Get<std::string>("commandSubTopic",
      this->commandSubTopic, this->commandSubTopic);

  sdfClone->Get<std::string>("enableSubTopic",
      this->enableSubTopic, this->enableSubTopic);

  // Subscribe to actuator command messages
  std::string topic{this->robotNamespace + "/" + this->commandSubTopic};

  this->node.Subscribe(topic, &MulticopterVelocityControl::OnTwist, this);
  ignmsg << "MulticopterVelocityControl subscribing to Twist messages on ["
         << topic << "]" << std::endl;

  std::string enableTopic{this->robotNamespace + "/" + this->enableSubTopic};
  this->node.Subscribe(enableTopic, &MulticopterVelocityControl::OnEnable,
                       this);
  ignmsg << "MulticopterVelocityControl subscribing to Boolean messages on ["
         << enableTopic << "]" << std::endl;

  // Create the Actuators component to take control of rotor speeds
  this->rotorVelocitiesMsg.mutable_velocity()->Resize(
      this->rotorVelocities.size(), 0);

  _ecm.CreateComponent(this->model.Entity(),
                       components::Actuators(this->rotorVelocitiesMsg));

  this->initialized = true;
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MulticopterVelocityControl::PreUpdate");

  if (!this->initialized)
  {
    return;
  }

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
  {
    return;
  }

  if (!this->controllerActive)
  {
    // If the last published rotor velocities were not 0, publish zero
    // velocities
    if (this->rotorVelocities.squaredNorm() > 0)
    {
      this->rotorVelocities.setZero();
      this->PublishRotorVelocities(_ecm, this->rotorVelocities);
      // Clear the cmdVelMsg so that the system waits for a new command after
      // being renabled.
      std::lock_guard<std::mutex> lock(this->cmdVelMsgMutex);
      this->cmdVelMsg.reset();
    }
    return;
  }

  EigenTwist cmdVel;
  {
    std::lock_guard<std::mutex> lock(this->cmdVelMsgMutex);
    if (!this->cmdVelMsg.has_value())
    {
      return;
    }

    // Clip with max linear velocity
    math::Vector3d linear = msgs::Convert(this->cmdVelMsg->linear());
    linear.Min(this->maximumLinearVelocity);
    linear.Max(-this->maximumLinearVelocity);

    math::Vector3d angular = msgs::Convert(this->cmdVelMsg->angular());
    angular.Min(this->maximumAngularVelocity);
    angular.Max(-this->maximumAngularVelocity);

    cmdVel.linear = math::eigen3::convert(linear);
    cmdVel.angular = math::eigen3::convert(angular);
  }

  std::optional<FrameData> frameData =
      getFrameData(_ecm, this->comLinkEntity, this->noiseParameters);
  if (!frameData.has_value())
  {
    // Errors would have already been printed
    return;
  }

  this->velocityController->CalculateRotorVelocities(*frameData, cmdVel,
                                                     this->rotorVelocities);

  this->PublishRotorVelocities(_ecm, this->rotorVelocities);
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::OnTwist(
    const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdVelMsgMutex);
  this->cmdVelMsg = _msg;
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::OnEnable(
    const msgs::Boolean &_msg)
{
  this->controllerActive = _msg.data();
}

//////////////////////////////////////////////////
void MulticopterVelocityControl::PublishRotorVelocities(
    ignition::gazebo::EntityComponentManager &_ecm,
    const Eigen::VectorXd &_vels)
{
  if (_vels.size() != this->rotorVelocitiesMsg.velocity_size())
  {
    this->rotorVelocitiesMsg.mutable_velocity()->Resize(_vels.size(), 0);
  }
  for (int i = 0; i < this->rotorVelocities.size(); ++i)
  {
    this->rotorVelocitiesMsg.set_velocity(i, _vels(i));
  }
  // Publish the message by setting the Actuators component on the model entity.
  // This assumes that the MulticopterMotorModel system is attached to this
  // model
  auto actuatorMsgComp =
      _ecm.Component<components::Actuators>(this->model.Entity());

  if (actuatorMsgComp)
  {
    auto compFunc = [](const msgs::Actuators &_a, const msgs::Actuators &_b)
    {
      return std::equal(_a.velocity().begin(), _a.velocity().end(),
                        _b.velocity().begin());
    };
    auto state = actuatorMsgComp->SetData(this->rotorVelocitiesMsg, compFunc)
                     ? ComponentState::PeriodicChange
                     : ComponentState::NoChange;
    _ecm.SetChanged(this->model.Entity(), components::Actuators::typeId, state);
  }
  else
  {
    _ecm.CreateComponent(this->model.Entity(),
                         components::Actuators(this->rotorVelocitiesMsg));
  }
}

IGNITION_ADD_PLUGIN(MulticopterVelocityControl,
                    ignition::gazebo::System,
                    MulticopterVelocityControl::ISystemConfigure,
                    MulticopterVelocityControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    MulticopterVelocityControl,
    "ignition::gazebo::systems::MulticopterVelocityControl")
