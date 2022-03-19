/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <memory>
#include <mutex>
#include <string>

#include <ignition/msgs/double.pb.h>

#include <ignition/math/Helpers.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "Thruster.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::ThrusterPrivateData
{
  /// \brief The mode of operation
  public: enum OperationMode {
    /// \brief Takes in a force commmand and spins the propeller at an
    /// appropriate rate.
    ForceCmd = 0,
    /// \brief Takes in angular velocity commands in radians per second and
    /// calculates the appropriate force.
    AngVelCmd
  } opmode = OperationMode::ForceCmd;

  /// \brief Mutex for read/write access to class
  public: std::mutex mtx;

  /// \brief Thrust output by propeller in N
  public: double thrust = 0.0;

  /// \brief Desired propeller angular velocity in rad / s
  public: double propellerAngVel = 0.0;

  /// \brief The link entity which will spin
  public: Entity linkEntity;

  /// \brief Axis along which the propeller spins. Expressed in the joint
  /// frame. Addume this doesn't change during simulation.
  public: math::Vector3d jointAxis;

  /// \brief Joint pose in the child link frame. Assume this doesn't change
  /// during the simulation.
  public: math::Pose3d jointPose;

  /// \brief Propeller koint entity
  public: Entity jointEntity;

  /// \brief ignition node for handling transport
  public: transport::Node node;

  /// \brief Publisher for feedback of data
  public: transport::Node::Publisher pub;

  /// \brief The PID which controls the propeller. This isn't used if
  /// velocityControl is true.
  public: math::PID propellerController;

  /// \brief Velocity Control mode - this disables the propellerController
  /// and writes the angular velocity directly to the joint. default: false
  public: bool velocityControl = false;

  /// \brief Maximum input force [N] for the propellerController,
  /// default: 1000N
  public: double cmdMax = 1000;

  /// \brief Minimum input force [N] for the propellerController,
  /// default: -1000N
  public: double cmdMin = -1000;

  /// \brief Thrust coefficient relating the propeller angular velocity to the
  /// thrust
  public: double thrustCoefficient = 1;

  /// \brief Density of fluid in kgm^-3, default: 1000kgm^-3
  public: double fluidDensity = 1000;

  /// \brief Diameter of propeller in m, default: 0.02
  public: double propellerDiameter = 0.02;

  /// \brief callback for handling thrust update
  public: void OnCmdThrust(const ignition::msgs::Double &_msg);

  /// \brief callback for handling angular velocity update
  public: void OnCmdAngVel(const ignition::msgs::Double &_msg);

  /// \brief function which computes angular velocity from thrust
  /// \param[in] _thrust Thrust in N
  /// \return Angular velocity in rad/s
  public: double ThrustToAngularVec(double _thrust);

  /// \brief function which computers thrust from angular velocity
  /// \param[in] _angVel Angular Velocity in rad/s
  /// \return Thrust in Newtons
  public: double AngularVelToThrust(double _angVel);
};

/////////////////////////////////////////////////
Thruster::Thruster():
  dataPtr(std::make_unique<ThrusterPrivateData>())
{
  // do nothing
}

/////////////////////////////////////////////////
void Thruster::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);
  auto modelName = model.Name(_ecm);

  // Get namespace
  std::string ns = modelName;
  if (_sdf->HasElement("namespace"))
  {
    ns = _sdf->Get<std::string>("namespace");
  }

  // Get joint name
  if (!_sdf->HasElement("joint_name"))
  {
    ignerr << "Missing <joint_name>. Plugin won't be initialized."
           << std::endl;
    return;
  }
  auto jointName = _sdf->Get<std::string>("joint_name");

  // Get thrust coefficient
  if (_sdf->HasElement("thrust_coefficient"))
  {
    this->dataPtr->thrustCoefficient = _sdf->Get<double>("thrust_coefficient");
  }

  // Get propeller diameter
  if (_sdf->HasElement("propeller_diameter"))
  {
    this->dataPtr->propellerDiameter = _sdf->Get<double>("propeller_diameter");
  }

  // Get fluid density, default to water otherwise
  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Get the operation mode
  if (_sdf->HasElement("use_angvel_cmd"))
  {
    this->dataPtr->opmode = _sdf->Get<bool>("use_angvel_cmd") ?
      ThrusterPrivateData::OperationMode::AngVelCmd :
      ThrusterPrivateData::OperationMode::ForceCmd;
  }

  this->dataPtr->jointEntity = model.JointByName(_ecm, jointName);
  if (kNullEntity == this->dataPtr->jointEntity)
  {
    ignerr << "Failed to find joint [" << jointName << "] in model ["
           << modelName << "]. Plugin not initialized." << std::endl;
    return;
  }

  this->dataPtr->jointAxis =
    _ecm.Component<ignition::gazebo::components::JointAxis>(
    this->dataPtr->jointEntity)->Data().Xyz();

  this->dataPtr->jointPose = _ecm.Component<components::Pose>(
      this->dataPtr->jointEntity)->Data();

  // Get link entity
  auto childLink =
      _ecm.Component<ignition::gazebo::components::ChildLinkName>(
      this->dataPtr->jointEntity);
  this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

  if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ForceCmd)
  {
    // Keeping cmd_pos for backwards compatibility
    // TODO(chapulina) Deprecate cmd_pos, because the commands aren't positions
    std::string thrusterTopicOld =
      ignition::transport::TopicUtils::AsValidTopic(
        "/model/" + ns + "/joint/" + jointName + "/cmd_pos");

    this->dataPtr->node.Subscribe(
      thrusterTopicOld,
      &ThrusterPrivateData::OnCmdThrust,
      this->dataPtr.get());

    // Subscribe to force commands
    std::string thrusterTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/cmd_thrust");

    this->dataPtr->node.Subscribe(
      thrusterTopic,
      &ThrusterPrivateData::OnCmdThrust,
      this->dataPtr.get());

    ignmsg << "Thruster listening to commands in [" << thrusterTopic << "]"
          << std::endl;

    std::string feedbackTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/ang_vel");
    this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::Double>(
      feedbackTopic
    );
  }
  else
  {
    igndbg << "Using angular velocity mode" << std::endl;
    // Subscribe to angvel commands
    std::string thrusterTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/cmd_vel");

    this->dataPtr->node.Subscribe(
      thrusterTopic,
      &ThrusterPrivateData::OnCmdAngVel,
      this->dataPtr.get());

    ignmsg << "Thruster listening to commands in [" << thrusterTopic << "]"
          << std::endl;

    std::string feedbackTopic = ignition::transport::TopicUtils::AsValidTopic(
      "/model/" + ns + "/joint/" + jointName + "/force");
    this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::Double>(
      feedbackTopic
    );
  }

  // Create necessary components if not present.
  enableComponent<components::AngularVelocity>(_ecm, this->dataPtr->linkEntity);
  enableComponent<components::WorldAngularVelocity>(_ecm,
      this->dataPtr->linkEntity);

  double minThrustCmd = this->dataPtr->cmdMin;
  double maxThrustCmd = this->dataPtr->cmdMax;
  if (_sdf->HasElement("max_thrust_cmd"))
  {
    maxThrustCmd = _sdf->Get<double>("max_thrust_cmd");
  }
  if (_sdf->HasElement("min_thrust_cmd"))
  {
    minThrustCmd = _sdf->Get<double>("min_thrust_cmd");
  }
  if (maxThrustCmd < minThrustCmd)
  {
    ignerr << "<max_thrust_cmd> must be greater than or equal to "
           << "<min_thrust_cmd>. Revert to using default values: "
           << "min: " << this->dataPtr->cmdMin << ", "
           << "max: " << this->dataPtr->cmdMax << std::endl;
  }
  else
  {
    this->dataPtr->cmdMax = maxThrustCmd;
    this->dataPtr->cmdMin = minThrustCmd;
  }

  if (_sdf->HasElement("velocity_control"))
  {
    this->dataPtr->velocityControl = _sdf->Get<bool>("velocity_control");
  }

  if (!this->dataPtr->velocityControl)
  {
    igndbg << "Using PID controller for propeller joint." << std::endl;

    double p         =  0.1;
    double i         =  0;
    double d         =  0;
    double iMax      =  1;
    double iMin      = -1;
    double cmdMax    = this->dataPtr->ThrustToAngularVec(this->dataPtr->cmdMax);
    double cmdMin    = this->dataPtr->ThrustToAngularVec(this->dataPtr->cmdMin);
    double cmdOffset =  0;

    if (_sdf->HasElement("p_gain"))
    {
      p = _sdf->Get<double>("p_gain");
    }
    if (!_sdf->HasElement("i_gain"))
    {
      i = _sdf->Get<double>("i_gain");
    }
    if (!_sdf->HasElement("d_gain"))
    {
      d = _sdf->Get<double>("d_gain");
    }

    this->dataPtr->propellerController.Init(
      p,
      i,
      d,
      iMax,
      iMin,
      cmdMax,
      cmdMin,
      cmdOffset);
  }
  else
  {
    igndbg << "Using velocity control for propeller joint." << std::endl;
  }
}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnCmdThrust(const ignition::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  this->thrust = ignition::math::clamp(ignition::math::fixnan(_msg.data()),
    this->cmdMin, this->cmdMax);

  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  this->propellerAngVel = this->ThrustToAngularVec(this->thrust);
}

/////////////////////////////////////////////////
void ThrusterPrivateData::OnCmdAngVel(const ignition::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  this->propellerAngVel =
    ignition::math::clamp(ignition::math::fixnan(_msg.data()),
      this->cmdMin, this->cmdMax);

  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  this->thrust = this->AngularVelToThrust(this->propellerAngVel);
}

/////////////////////////////////////////////////
double ThrusterPrivateData::ThrustToAngularVec(double _thrust)
{
  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  auto propAngularVelocity = sqrt(abs(
    _thrust /
      (this->fluidDensity
      * this->thrustCoefficient * pow(this->propellerDiameter, 4))));

  propAngularVelocity *= (_thrust * this->thrustCoefficient > 0) ? 1: -1;

  return propAngularVelocity;
}

/////////////////////////////////////////////////
double ThrusterPrivateData::AngularVelToThrust(double _angVel)
{
  // Thrust is proportional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  return this->thrustCoefficient * pow(this->propellerDiameter, 4)
    * abs(_angVel) * _angVel * this->fluidDensity;
}

/////////////////////////////////////////////////
void Thruster::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  ignition::gazebo::Link link(this->dataPtr->linkEntity);

  auto pose = worldPose(this->dataPtr->linkEntity, _ecm);

  // TODO(arjo129): add logic for custom coordinate frame
  // Convert joint axis to the world frame
  const auto linkWorldPose = worldPose(this->dataPtr->linkEntity, _ecm);
  auto jointWorldPose = linkWorldPose * this->dataPtr->jointPose;
  auto unitVector =
      jointWorldPose.Rot().RotateVector(this->dataPtr->jointAxis).Normalize();

  double desiredThrust;
  double desiredPropellerAngVel;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    desiredThrust = this->dataPtr->thrust;
    desiredPropellerAngVel = this->dataPtr->propellerAngVel;
  }

  msgs::Double angvel;
  // PID control
  double torque = 0.0;
  if (!this->dataPtr->velocityControl)
  {
    auto currentAngular = (link.WorldAngularVelocity(_ecm))->Dot(unitVector);
    auto angularError = currentAngular - desiredPropellerAngVel;
    if (abs(angularError) > 0.1)
    {
      torque = this->dataPtr->propellerController.Update(angularError,
          _info.dt);
    }
    angvel.set_data(currentAngular);
  }
  // Velocity control
  else
  {
    auto velocityComp =
    _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
      this->dataPtr->jointEntity);
    if (velocityComp == nullptr)
    {
      _ecm.CreateComponent(this->dataPtr->jointEntity,
        components::JointVelocityCmd({desiredPropellerAngVel}));
    }
    else
    {
      velocityComp->Data()[0] = desiredPropellerAngVel;
    }
    angvel.set_data(desiredPropellerAngVel);
  }

  if (this->dataPtr->opmode == ThrusterPrivateData::OperationMode::ForceCmd)
  {
    this->dataPtr->pub.Publish(angvel);
  }
  else
  {
    msgs::Double force;
    force.set_data(desiredThrust);
    this->dataPtr->pub.Publish(force);
  }
  // Force: thrust
  // Torque: propeller rotation, if using PID
  link.AddWorldWrench(
    _ecm,
    unitVector * desiredThrust,
    unitVector * torque);
}

IGNITION_ADD_PLUGIN(
  Thruster, System,
  Thruster::ISystemConfigure,
  Thruster::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Thruster, "ignition::gazebo::systems::Thruster")
