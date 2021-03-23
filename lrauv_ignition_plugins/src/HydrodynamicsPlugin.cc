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
  this->dataPtr = std::make_unique<HydrodynamicsPrivateData>();
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
  this->dataPtr->waterDensity     = SdfParamDouble(_sdf, "waterDensity", 997.7735);
  this->dataPtr->paramXdotU       = SdfParamDouble(_sdf, "xDotU"       , 5);
  this->dataPtr->paramYdotV       = SdfParamDouble(_sdf, "yDotV"       , 5);
  this->dataPtr->paramZdotW       = SdfParamDouble(_sdf, "zDotW"       , 0.1);
  this->dataPtr->paramKdotP       = SdfParamDouble(_sdf, "kDotP"       , 0.1);
  this->dataPtr->paramMdotQ       = SdfParamDouble(_sdf, "mDotQ"       , 0.1);
  this->dataPtr->paramNdotR       = SdfParamDouble(_sdf, "nDotR"       , 1);
  this->dataPtr->paramXu          = SdfParamDouble(_sdf, "xU"          , 20);
  this->dataPtr->paramXuu         = SdfParamDouble(_sdf, "xUU"         , 0);
  this->dataPtr->paramYv          = SdfParamDouble(_sdf, "yV"          , 20);
  this->dataPtr->paramYvv         = SdfParamDouble(_sdf, "yVV"         , 0);
  this->dataPtr->paramZw          = SdfParamDouble(_sdf, "zW"          , 20);
  this->dataPtr->paramZww         = SdfParamDouble(_sdf, "zWW"         , 0);
  this->dataPtr->paramKp          = SdfParamDouble(_sdf, "kP"          , 20);
  this->dataPtr->paramKpp         = SdfParamDouble(_sdf, "kPP"         , 0);
  this->dataPtr->paramMq          = SdfParamDouble(_sdf, "mQ"          , 20);
  this->dataPtr->paramMqq         = SdfParamDouble(_sdf, "mQQ"         , 0);
  this->dataPtr->paramNr          = SdfParamDouble(_sdf, "nR"          , 20);
  this->dataPtr->paramNrr         = SdfParamDouble(_sdf, "nRR"         , 0);

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);
  auto link_name = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = model.LinkByName(_ecm, link_name);

  this->dataPtr->prevState = Eigen::VectorXd::Zero(6); 

  AddWorldPose(this->dataPtr->linkEntity, _ecm);
  AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
  AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
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
  ignition::gazebo::Link baseLink(this->dataPtr->linkEntity);
  auto linearVelocity =
    _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->dataPtr->linkEntity);
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
  stateDot = (state - this->dataPtr->prevState)/dt;

  this->dataPtr->prevState = state;

  // Added mass according to Fossen's equations (p 37)
  Ma(0,0) = this->dataPtr->paramXdotU;
  Ma(1,1) = this->dataPtr->paramYdotV;
  Ma(2,2) = this->dataPtr->paramZdotW;
  Ma(3,3) = this->dataPtr->paramKdotP;
  Ma(4,4) = this->dataPtr->paramMdotQ;
  Ma(5,5) = this->dataPtr->paramNdotR;
  const Eigen::VectorXd kAmassVec = Ma * stateDot;

  // Coriollis and Centripetal forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF
  Cmat(0,4) = - this->dataPtr->paramZdotW * state(2);
  Cmat(0,5) = - this->dataPtr->paramYdotV * state(1);
  Cmat(1,3) = this->dataPtr->paramZdotW * state(2);
  Cmat(1,5) = - this->dataPtr->paramXdotU * state(0);
  Cmat(2,3) = - this->dataPtr->paramYdotV * state(1);
  Cmat(2,4) = this->dataPtr->paramXdotU * state(0);
  Cmat(3,1) = - this->dataPtr->paramZdotW * state(2);
  Cmat(3,2) = this->dataPtr->paramYdotV * state(1);
  Cmat(3,4) = - this->dataPtr->paramNdotR * state(5);
  Cmat(3,5) = this->dataPtr->paramMdotQ * state(4);
  Cmat(4,0) = this->dataPtr->paramZdotW * state(2);
  Cmat(4,2) = - this->dataPtr->paramXdotU * state(0);
  Cmat(4,3) = this->dataPtr->paramNdotR * state(5);
  Cmat(4,5) = - this->dataPtr->paramKdotP * state(3);
  Cmat(5,0) = this->dataPtr->paramZdotW * state(2);
  Cmat(5,1) = this->dataPtr->paramXdotU * state(0);
  Cmat(5,3) = - this->dataPtr->paramMdotQ * state(4);
  Cmat(5,4) = this->dataPtr->paramKdotP * state(3);
  const Eigen::VectorXd kCmatVec = - Cmat * state;

  // Damping forces (Fossen P. 43)
  Dmat(1,1) = - this->dataPtr->paramYv - this->dataPtr->paramYvv * abs(state(1));
  Dmat(0,0) = - this->dataPtr->paramXu - this->dataPtr->paramXuu * abs(state(0));
  Dmat(2,2) = - this->dataPtr->paramZw - this->dataPtr->paramZww * abs(state(2));
  Dmat(3,3) = - this->dataPtr->paramKp - this->dataPtr->paramKpp * abs(state(3));
  Dmat(4,4) = - this->dataPtr->paramMq - this->dataPtr->paramMqq * abs(state(4));
  Dmat(5,5) = - this->dataPtr->paramNr - this->dataPtr->paramNrr * abs(state(5));

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
