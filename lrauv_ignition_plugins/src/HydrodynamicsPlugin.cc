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

#include <eigen/eigen.hpp>

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
    // Create an angular velocity component if one is not present.
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      ignition::gazebo::components::WorldPose());
  }
}


double SdfParamDouble(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string& field,
    double default)
{
  if(!_sdf->HasElement(field))
  {
    return default;
  }
  return _sdf->Get<double>(field);
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
  _pimpl->waterDensity     = SdfParamDouble(_sdf, "waterDensity", 997.7735);
  _pimpl->paramXdotU       = SdfParamDouble(_sdf, "xDotU"       , 5);
  _pimpl->paramYdotV       = SdfParamDouble(_sdf, "yDotV"       , 5);
  _pimpl->paramZdotW       = SdfParamDouble(_sdf, "zDotW"       , 0.1);
  _pimpl->paramKdotP       = SdfParamDouble(_sdf, "kDotP"       , 0.1);
  _pimpl->paramMdotQ       = SdfParamDouble(_sdf, "mDotQ"       , 0.1);
  _pimpl->paramNdotR       = SdfParamDouble(_sdf, "nDotR"       , 1);
  _pimpl->paramXu          = SdfParamDouble(_sdf, "xU"          , 20);
  _pimpl->paramXuu         = SdfParamDouble(_sdf, "xUU"         , 0);
  _pimpl->paramYv          = SdfParamDouble(_sdf, "yV"          , 20);
  _pimpl->paramYvv         = SdfParamDouble(_sdf, "yVV"         , 0);
  _pimpl->paramZw          = SdfParamDouble(_sdf, "zW"          , 20);
  _pimpl->paramZww         = SdfParamDouble(_sdf, "zWW"         , 0);
  _pimpl->paramKp          = SdfParamDouble(_sdf, "kP"          , 20);
  _pimpl->paramKpp         = SdfParamDouble(_sdf, "kPP"         , 0);
  _pimpl->paramMq          = SdfParamDouble(_sdf, "mQ"          , 20);
  _pimpl->paramMqq         = SdfParamDouble(_sdf, "mQQ"         , 0);
  _pimpl->paramNr          = SdfParamDouble(_sdf, "nR"          , 20);
  _pimpl->paramNrr         = SdfParamDouble(_sdf, "nRR"         , 0);

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);
  _pimpl->linkEntity = model.LinkByName();

  _pimpl->prevState = Eigen::VectorXd::Zero(6); 

  AddWorldPose(_pimpl->linkEntity, _ecm);
  AddAngularVelocityComponent(_pimpl->linkEntity, _ecm);

}

void HydrodynamicsPlugin::PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
{
  if(_info.paused)
    return;

  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Ma = Eigen::MatrixXd::zero(6,6);

  // Get vehicle state
  ignition::gazebo::Link baseLink(_pimpl->linkEntity);
  auto linearVelocity = baseLink.WorldLinearVelocity(_ecm);
  auto rotationalVelocity = baseLink.WorldAngularVelocity(_ecm);

  state << linearVelocity->X() << linearVelocity->Y() << linearVelocity->Z()
    << rotationalVelocity->X() << rotationalVelocity->Y() << rotationalVelocity->Z();
  
  stateDot = (state - _pimpl->prevState)/_info.dt;

  // Added mass according to Fossen's equations (p 37)
  Ma(0,0) = _pimpl->paramXdotU;
  Ma(1,1) = _pimpl->paramYdotV;
  Ma(2,2) = _pimpl->paramZdotW;
  Ma(3,3) = _pimpl->paramKdotP;
  Ma(4,4) = _pimpl->paramMdotQ;
  Ma(5,5) = _pimpl->paramNdotR;
  const Eigen::VectorXd kAmassVec = -1.0 * Ma * stateDot;

  // Coriollis forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF
  Cmat(0,4) = - _pimpl->paramZdotW * state(2);
  Cmat(0,5) = - _pimpl->paramYdotV * state(1);
  Cmat(1,3) = _pimpl->paramZdotW * state(2);
  Cmat(1,5) = - _pimpl->paramXdotU * state(0);
  Cmat(2,3) = - _pimpl->paramYdotV * state(1);
  Cmat(2,4) = _pimpl->paramXdotU * state(0);
  Cmat(3,1) = - _pimpl->paramZdotW * state(2);
  Cmat(3,2) = _pimpl->paramYdotV * state(1);
  Cmat(3,4) = - _pimpl->paramNdotR * state(5);
  Cmat(3,5) = _pimpl->paramMdotQ * state(4);
  Cmat(4,0) = _pimpl->paramZdotW * state(2);
  Cmat(4,2) = - _pimpl->paramXdotU * state(0);
  Cmat(4,3) = _pimpl->paramNdotR * state(5);
  Cmat(4,5) = - _pimpl->paramKdotP * state(3);
  Cmat(5,0) = _pimpl->paramZdotW * state(2);
  Cmat(5,1) = _pimpl->paramXdotU * state(0);
  Cmat(5,3) = -_pimpl->paramMdotQ * state(4);
  Cmat(5,4) = _pimpl->paramKdotP * state(3);
  const Eigen::VectorXd kCmatVec = -1.0 * Cmat * state;

  // Damping forces (Fossen P. 43)
  Dmat(0,0) = - _pimpl->paramXu - _pimpl->paramXuu * abs(state(0));
  Dmat(1,1) = - _pimpl->paramYv - _pimpl->paramYvv * abs(state(1));
  Dmat(2,2) = - _pimpl->paramZw - _pimpl->paramZww * abs(state(2));
  Dmat(3,3) = - _pimpl->paramKp - _pimpl->paramKpp * abs(state(3));

  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;

  const Eigen::VectorXd kTotalWrench = kAmassVec + kDvec + kCmatVec;

  ign::math::Vector3d totalForce(kTotalWrench(0),  kTotalWrench(1), kTotalWrench(2));
  ign::math::Vector3d totalTorque(kTotalWrench(3),  kTotalWrench(4), kTotalWrench(5)); 

  baseLink.AddWorldWrench(_ecm, totalForce, totalTorque);
}

};