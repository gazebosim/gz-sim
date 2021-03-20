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

#include <ignition/math/Helpers.hh>

#include "ThrusterPlugin.hh"

namespace tethys_thrusters
{
class ThrusterPrivateData
{
  public: std::mutex mtx;

  public: double thrust = 0.0;
  
  public: ignition::gazebo::Entity linkEntity;
  
  public: ignition::math::Vector3d jointAxis;
  
  public: ignition::transport::Node node;

  public: ignition::math::PID rpmController;

  public: double cmdMax = 1000;

  public: double cmdMin = -1000;

  public: double thrustCoefficient;

  public: double fluidDensity = 1000;

  public: double propellerDiameter;

  public: void OnCmdThrust(const ignition::msgs::Double &_msg);
};

ThrusterPlugin::ThrusterPlugin()
{
    this->dataPtr = std::make_unique<ThrusterPrivateData>();
}

ThrusterPlugin::~ThrusterPlugin()
{

}

void ThrusterPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  // Get namespace
  std::string ns {""};
  if (_sdf->HasElement("namespace"))
  {
    ns = _sdf->Get<std::string>("namespace");
  }

  // Get joint name
  if (!_sdf->HasElement("joint_name")) 
  {
    ignerr << "No joint to treat as propeller found \n";
    return;
  }
  auto jointName = _sdf->Get<std::string>("joint_name");

  // Get thrust coefficient
  if(!_sdf->HasElement("thrust_coefficient"))
  {
    ignerr << "Failed to get thrust_coefficient" << "\n";
    return;
  }
  this->dataPtr->thrustCoefficient = _sdf->Get<double>("thrust_coefficient");

  // Get propeller diameter
  if(!_sdf->HasElement("propeller_diameter"))
  {
    ignerr << "Failed to get propeller_diameter \n";
  }
  this->dataPtr->propellerDiameter = _sdf->Get<double>("propeller_diameter");

  // Get fluid density, default to water otherwise
  if(_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }
  igndbg << "Setting fluid density to: " << this->dataPtr->fluidDensity << "\n";

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);

  auto jointEntity = model.JointByName(_ecm, jointName);
  auto childLink =
    _ecm.Component<ignition::gazebo::components::ChildLinkName>(jointEntity);

  this->dataPtr->jointAxis =
    _ecm.Component<ignition::gazebo::components::JointAxis>(jointEntity)
      ->Data().Xyz();

  std::string thrusterTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + ns + "/joint/" + jointName + "/cmd_pos");
  this->dataPtr->node.Subscribe(thrusterTopic, &ThrusterPrivateData::OnCmdThrust,
    this->dataPtr.get());

  // Get link entity
  this->dataPtr->linkEntity = model.LinkByName(_ecm, childLink->Data());

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
      this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
      ignition::gazebo::components::AngularVelocity());
  }

    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
      this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
      ignition::gazebo::components::WorldAngularVelocity());
  }

  double p         =  0.1;
  double i         =  0;
  double d         =  0;
  double iMax      =  1;
  double iMin      = -1;
  double cmdMax    = this->dataPtr->cmdMax;
  double cmdMin    = this->dataPtr->cmdMin;
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

  this->dataPtr->rpmController.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
}

void ThrusterPrivateData::OnCmdThrust(const ignition::msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(mtx);
  this->thrust = ignition::math::clamp(ignition::math::fixnan(_msg.data()),
    this->cmdMin, this->cmdMax);
}

void ThrusterPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  ignition::gazebo::Link link(this->dataPtr->linkEntity);

  auto pose = worldPose(this->dataPtr->linkEntity, _ecm);

  //TODO: add logic for custom coordinate frame
  auto unitVector = pose.Rot().RotateVector(this->dataPtr->jointAxis.Normalize());

  std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
  // Thrust is proprtional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  auto desired = sqrt(
    abs(this->dataPtr->thrust /
      (this->dataPtr->fluidDensity
      * this->dataPtr->thrustCoefficient * pow(this->dataPtr->propellerDiameter, 4))));
  
  desired *= (this->dataPtr->thrust > 0) ? 1: -1;
  auto currentAngular = (link.WorldAngularVelocity(_ecm))->Dot(unitVector);
  auto angularError = currentAngular - desired;
  double torque = 0.0;
  if(abs(angularError) > 0.1)
    torque = this->dataPtr->rpmController.Update(angularError, _info.dt);

  link.AddWorldWrench(_ecm, unitVector * this->dataPtr->thrust, unitVector * torque);
}
} //end namespace tethys thrusters

IGNITION_ADD_PLUGIN(
  tethys_thrusters::ThrusterPlugin,
  ignition::gazebo::System,
  tethys_thrusters::ThrusterPlugin::ISystemConfigure,
  tethys_thrusters::ThrusterPlugin::ISystemPreUpdate)
