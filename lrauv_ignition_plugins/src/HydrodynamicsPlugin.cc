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

#include "HydrodynamicsPlugin.hh"

#include <Eigen/Eigen>

namespace tethys_hydro
{

class HydrodynamicsPrivateData
{
  /// \brief Values to set via Plugin Parameters.
  /// Plugin Parameter: Added mass in surge, X_\dot{u}.
  public: double paramXdotU;
  
  /// \brief Plugin Parameter: Added mass in sway, Y_\dot{v}.
  public: double paramYdotV;
  
  /// \brief Plugin Parameter: Added mass in heave, Z_\dot{w}.
  public: double paramZdotW;
  
  /// \brief Plugin Parameter: Added mass in roll, K_\dot{p}.
  public: double paramKdotP;
  
  /// \brief Plugin Parameter: Added mass in pitch, M_\dot{q}.
  public: double paramMdotQ;
  
  /// \brief Plugin Parameter: Added mass in yaw, N_\dot{r}.
  public: double paramNdotR;
  
  /// \brief Plugin Parameter: Linear drag in surge.
  public: double paramXu;
  
  /// \brief Plugin Parameter: Quadratic drag in surge.
  public: double paramXuu;
  
  /// \brief Plugin Parameter: Linear drag in sway.
  public: double paramYv;
  
  /// \brief Plugin Parameter: Quadratic drag in sway.
  public: double paramYvv;
  
  /// \brief Plugin Parameter: Linear drag in heave.
  public: double paramZw;
  
  /// \brief Plugin Parameter: Quadratic drag in heave.
  public: double paramZww;
  
  /// \brief Plugin Parameter: Linear drag in roll.
  public: double paramKp;
  
  /// \brief Plugin Parameter: Quadratic drag in roll.
  public: double paramKpp;
  
  /// \brief Plugin Parameter: Linear drag in pitch.
  public: double paramMq;
  
  /// \brief Plugin Parameter: Quadratic drag in pitch.
  public: double paramMqq;
  
  /// \brief Plugin Parameter: Linear drag in yaw.
  public: double paramNr;
  
  /// \brief Plugin Parameter: Quadratic drag in yaw.
  public: double paramNrr;
  
  /// \brief Water density [kg/m^3].
  public: double waterDensity;

  public: Eigen::VectorXd prevState;

  /// Link entity
  public: ignition::gazebo::Entity linkEntity;
};


void AddAngularVelocityComponent(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::AngularVelocity());
  }
    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldAngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldAngularVelocity());
  }
}

void AddWorldPose (
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
}

void AddWorldLinearVelocity(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldLinearVelocity());
  }
}

double SdfParamDouble(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string& _field,
    double _default)
{
  if(!_sdf->HasElement(_field))
  {
    return _default;
  }
  return _sdf->Get<double>(_field);
}

HydrodynamicsPlugin::HydrodynamicsPlugin()
{
  _data = std::make_unique<HydrodynamicsPrivateData>();
}

HydrodynamicsPlugin::~HydrodynamicsPlugin()
{
    
}

void HydrodynamicsPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/
)
{
  _data->waterDensity     = SdfParamDouble(_sdf, "waterDensity", 997.7735);
  _data->paramXdotU       = SdfParamDouble(_sdf, "xDotU"       , 5);
  _data->paramYdotV       = SdfParamDouble(_sdf, "yDotV"       , 5);
  _data->paramZdotW       = SdfParamDouble(_sdf, "zDotW"       , 0.1);
  _data->paramKdotP       = SdfParamDouble(_sdf, "kDotP"       , 0.1);
  _data->paramMdotQ       = SdfParamDouble(_sdf, "mDotQ"       , 0.1);
  _data->paramNdotR       = SdfParamDouble(_sdf, "nDotR"       , 1);
  _data->paramXu          = SdfParamDouble(_sdf, "xU"          , 20);
  _data->paramXuu         = SdfParamDouble(_sdf, "xUU"         , 0);
  _data->paramYv          = SdfParamDouble(_sdf, "yV"          , 20);
  _data->paramYvv         = SdfParamDouble(_sdf, "yVV"         , 0);
  _data->paramZw          = SdfParamDouble(_sdf, "zW"          , 20);
  _data->paramZww         = SdfParamDouble(_sdf, "zWW"         , 0);
  _data->paramKp          = SdfParamDouble(_sdf, "kP"          , 20);
  _data->paramKpp         = SdfParamDouble(_sdf, "kPP"         , 0);
  _data->paramMq          = SdfParamDouble(_sdf, "mQ"          , 20);
  _data->paramMqq         = SdfParamDouble(_sdf, "mQQ"         , 0);
  _data->paramNr          = SdfParamDouble(_sdf, "nR"          , 20);
  _data->paramNrr         = SdfParamDouble(_sdf, "nRR"         , 0);

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);
  auto link_name = _sdf->Get<std::string>("link_name");
  _data->linkEntity = model.LinkByName(_ecm, link_name);

  _data->prevState = Eigen::VectorXd::Zero(6); 

  AddWorldPose(_data->linkEntity, _ecm);
  AddAngularVelocityComponent(_data->linkEntity, _ecm);
  AddWorldLinearVelocity(_data->linkEntity, _ecm);
}

void HydrodynamicsPlugin::PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
{
  if(_info.paused)
    return;

  // These variables are named following Fossen's scheme in "Guidance and Control 
  // of Ocean Vehicles." The `state` vector contains the ship's current velocity
  // in the formate [x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel].
  // `stateDot` consists of the first derivative in time of the state vector.
  // `Cmat` corresponds to the Centripetal matrix
  // `Dmat` is the drag matrix
  // `Ma` is the added mass. 
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Ma = Eigen::MatrixXd::Zero(6,6);

  // Get vehicle state
  ignition::gazebo::Link baseLink(_data->linkEntity);
  auto linearVelocity =
    _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(_data->linkEntity);
  auto rotationalVelocity = baseLink.WorldAngularVelocity(_ecm);
  
  if(!linearVelocity)
  {
    ignerr <<"no linear vel" <<"\n";
    return;
  }

  // Transform state to local frame
  auto pose = baseLink.WorldPose(_ecm);
  // Since we are transforming angular and linear velocity we only care about
  // rotation
  auto localLinearVelocity = pose->Rot().Inverse() * linearVelocity->Data();
  auto localRotationalVelocity = pose->Rot().Inverse() * *rotationalVelocity;
  
  state(0) = localLinearVelocity.X();
  state(1) = localLinearVelocity.Y();
  state(2) = localLinearVelocity.Z();

  state(3) = localRotationalVelocity.X();
  state(4) = localRotationalVelocity.Y();
  state(5) = localRotationalVelocity.Z();

  auto dt = (double)_info.dt.count()/1e9;
  stateDot = (state - _data->prevState)/dt;

  _data->prevState = state;

  // Added mass according to Fossen's equations (p 37)
  Ma(0,0) = _data->paramXdotU;
  Ma(1,1) = _data->paramYdotV;
  Ma(2,2) = _data->paramZdotW;
  Ma(3,3) = _data->paramKdotP;
  Ma(4,4) = _data->paramMdotQ;
  Ma(5,5) = _data->paramNdotR;
  const Eigen::VectorXd kAmassVec = Ma * stateDot;

  // Coriollis and Centripetal forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF
  Cmat(0,4) = - _data->paramZdotW * state(2);
  Cmat(0,5) = - _data->paramYdotV * state(1);
  Cmat(1,3) = _data->paramZdotW * state(2);
  Cmat(1,5) = - _data->paramXdotU * state(0);
  Cmat(2,3) = - _data->paramYdotV * state(1);
  Cmat(2,4) = _data->paramXdotU * state(0);
  Cmat(3,1) = - _data->paramZdotW * state(2);
  Cmat(3,2) = _data->paramYdotV * state(1);
  Cmat(3,4) = - _data->paramNdotR * state(5);
  Cmat(3,5) = _data->paramMdotQ * state(4);
  Cmat(4,0) = _data->paramZdotW * state(2);
  Cmat(4,2) = - _data->paramXdotU * state(0);
  Cmat(4,3) = _data->paramNdotR * state(5);
  Cmat(4,5) = - _data->paramKdotP * state(3);
  Cmat(5,0) = _data->paramZdotW * state(2);
  Cmat(5,1) = _data->paramXdotU * state(0);
  Cmat(5,3) = -_data->paramMdotQ * state(4);
  Cmat(5,4) = _data->paramKdotP * state(3);
  const Eigen::VectorXd kCmatVec = - Cmat * state;

  // Damping forces (Fossen P. 43)
  Dmat(0,0) = - _data->paramXu - _data->paramXuu * abs(state(0));
  Dmat(1,1) = - _data->paramYv - _data->paramYvv * abs(state(1));
  Dmat(2,2) = - _data->paramZw - _data->paramZww * abs(state(2));
  Dmat(3,3) = - _data->paramKp - _data->paramKpp * abs(state(3));
  Dmat(4,4) = - _data->paramMq - _data->paramMqq * abs(state(4));
  Dmat(5,5) = - _data->paramNr - _data->paramNrr * abs(state(5));

  const Eigen::VectorXd kDvec = Dmat * state;

  const Eigen::VectorXd kTotalWrench = kAmassVec + kDvec + kCmatVec;

  ignition::math::Vector3d totalForce(-kTotalWrench(0),  -kTotalWrench(1), -kTotalWrench(2));
  ignition::math::Vector3d totalTorque(-kTotalWrench(3),  -kTotalWrench(4), -kTotalWrench(5)); 

  baseLink.AddWorldWrench(_ecm, pose->Rot()*(totalForce), pose->Rot()*totalTorque);
}

};

IGNITION_ADD_PLUGIN(
  tethys_hydro::HydrodynamicsPlugin,
  ignition::gazebo::System,
  tethys_hydro::HydrodynamicsPlugin::ISystemConfigure,
  tethys_hydro::HydrodynamicsPlugin::ISystemPreUpdate)
