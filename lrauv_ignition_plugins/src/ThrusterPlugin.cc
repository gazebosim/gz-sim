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
  
  public: ignition::gazebo::Entity _linkEntity;
  
  public: ignition::math::Vector3d _jointAxis;
  
  public: ignition::transport::Node node;

  public: ignition::math::PID _rpmController;

  public: double cmdMax = 1000;

  public: double cmdMin = -1000;

  public: double _thrust_coefficient;

  public: double _fluid_density = 1000;

  public: double _propeller_diameter;

  public: void OnCmdThrust(const ignition::msgs::Double &_msg);
};

ThrusterPlugin::ThrusterPlugin()
{
    _data = std::make_unique<ThrusterPrivateData>();
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
  auto joint_name = _sdf->Get<std::string>("joint_name");

  // Get thrust coefficient
  if(!_sdf->HasElement("thrust_coefficient"))
  {
    ignerr << "Failed to get thrust_coefficient" << "\n";
    return;
  }
  _data->_thrust_coefficient = _sdf->Get<double>("thrust_coefficient");

  // Get propeller diameter
  if(!_sdf->HasElement("propeller_diameter"))
  {
    ignerr << "Failed to get propeller_diameter \n";
  }
  _data->_propeller_diameter = _sdf->Get<double>("propeller_diameter");

  // Get fluid density, default to water otherwise
  if(_sdf->HasElement("fluid_density"))
  {
    _data->_fluid_density = _sdf->Get<double>("fluid_density");
  }
  igndbg << "Setting fluid density to: " << _data->_fluid_density << "\n";

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);

  auto joint_entity = model.JointByName(_ecm, joint_name);
  auto child_link =
    _ecm.Component<ignition::gazebo::components::ChildLinkName>(joint_entity);

  _data->_jointAxis = 
    _ecm.Component<ignition::gazebo::components::JointAxis>(joint_entity)
      ->Data().Xyz();

  std::string thrusterTopic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" + ns + "/joint/" + joint_name + "/cmd_pos");
  _data->node.Subscribe(thrusterTopic, &ThrusterPrivateData::OnCmdThrust,
    _data.get());

  // Get link entity
  _data->_linkEntity = model.LinkByName(_ecm, child_link->Data());

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
      _data->_linkEntity))
  {
    _ecm.CreateComponent(_data->_linkEntity,
      ignition::gazebo::components::AngularVelocity());
  }

    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
      _data->_linkEntity))
  {
    _ecm.CreateComponent(_data->_linkEntity,
      ignition::gazebo::components::WorldAngularVelocity());
  }

  double p         =  0.1;
  double i         =  0;
  double d         =  0;
  double iMax      =  1;
  double iMin      = -1;
  double cmdMax    = this->_data->cmdMax;
  double cmdMin    = this->_data->cmdMin;
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

  _data->_rpmController.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
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

  ignition::gazebo::Link link(_data->_linkEntity);

  auto pose = worldPose(_data->_linkEntity, _ecm);

  //TODO: add logic for custom coordinate frame
  auto unit_vector = pose.Rot().RotateVector(_data->_jointAxis.Normalize());

  std::lock_guard<std::mutex> lock(_data->mtx);
  // Thrust is proprtional to the Rotation Rate squared
  // See Thor I Fossen's  "Guidance and Control of ocean vehicles" p. 246
  auto desired = sqrt(
    abs(_data->thrust / 
      (_data->_fluid_density 
      * _data->_thrust_coefficient * pow(_data->_propeller_diameter, 4))));
  
  desired *= (_data->thrust > 0) ? 1: -1;   
  auto _current_angular = (link.WorldAngularVelocity(_ecm))->Dot(unit_vector);
  auto _angular_error = _current_angular - desired;
  double _torque = 0.0;
  if(abs(_angular_error) > 0.1)
    _torque = _data->_rpmController.Update(_angular_error, _info.dt);

  link.AddWorldWrench(_ecm, unit_vector * _data->thrust, unit_vector * _torque);
}
} //end namespace tethys thrusters

IGNITION_ADD_PLUGIN(
  tethys_thrusters::ThrusterPlugin,
  ignition::gazebo::System,
  tethys_thrusters::ThrusterPlugin::ISystemConfigure,
  tethys_thrusters::ThrusterPlugin::ISystemPreUpdate)
