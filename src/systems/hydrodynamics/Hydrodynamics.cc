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
#include <string>

#include <Eigen/Eigen>

#include <gz/plugin/Register.hh>

#include "gz/msgs/vector3d.pb.h"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"

#include "gz/transport/Node.hh"

#include "Hydrodynamics.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Hydrodynamics data class.
class gz::sim::systems::HydrodynamicsPrivateData
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

  /// \brief The ignition transport node
  public: transport::Node node;

  /// \brief Ocean current experienced by this body
  public: math::Vector3d currentVector {0, 0, 0};

  /// \brief Added mass of vehicle;
  /// See: https://en.wikipedia.org/wiki/Added_mass
  public: Eigen::MatrixXd Ma;

  /// \brief Previous state.
  public: Eigen::VectorXd prevState;

  /// \brief Link entity
  public: Entity linkEntity;

  /// \brief Ocean current callback
  public: void UpdateCurrent(const msgs::Vector3d &_msg);

  /// \brief Mutex
  public: std::mutex mtx;
};

/////////////////////////////////////////////////
void HydrodynamicsPrivateData::UpdateCurrent(const msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  this->currentVector = gz::msgs::Convert(_msg);
}

/////////////////////////////////////////////////
void AddAngularVelocityComponent(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::AngularVelocity>(_entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::AngularVelocity());
  }

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<gz::sim::components::WorldAngularVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::WorldAngularVelocity());
  }
}

/////////////////////////////////////////////////
void AddWorldPose(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldPose>(_entity))
  {
    _ecm.CreateComponent(_entity, gz::sim::components::WorldPose());
  }
}

/////////////////////////////////////////////////
void AddWorldLinearVelocity(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::WorldLinearVelocity>(
      _entity))
  {
    _ecm.CreateComponent(_entity,
      gz::sim::components::WorldLinearVelocity());
  }
}

/////////////////////////////////////////////////
double SdfParamDouble(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string& _field,
    double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}

/////////////////////////////////////////////////
Hydrodynamics::Hydrodynamics()
{
  this->dataPtr = std::make_unique<HydrodynamicsPrivateData>();
}

/////////////////////////////////////////////////
Hydrodynamics::~Hydrodynamics()
{
  // Do nothing
}

/////////////////////////////////////////////////
void Hydrodynamics::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/
)
{
  if (_sdf->HasElement("waterDensity"))
  {
    ignwarn <<
      "<waterDensity> parameter is deprecated and will be removed Gazebo G.\n"
      << "\tPlease update your SDF to use <water_density> instead.";
  }

  this->dataPtr->waterDensity     = SdfParamDouble(_sdf, "waterDensity",
                                      SdfParamDouble(_sdf, "water_density", 998)
                                    );
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
  auto model = gz::sim::Model(_entity);

  std::string ns {""};
  std::string currentTopic {"/ocean_current"};
  if (_sdf->HasElement("namespace"))
  {
    ns = _sdf->Get<std::string>("namespace");
    currentTopic = gz::transport::TopicUtils::AsValidTopic(
        "/model/" + ns + "/ocean_current");
  }

  this->dataPtr->node.Subscribe(
    currentTopic,
    &HydrodynamicsPrivateData::UpdateCurrent,
    this->dataPtr.get());

  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "You musk specify a <link_name> for the hydrodynamic"
      << " plugin to act upon";
    return;
  }
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity))
  {
    ignerr << "Link name" << linkName << "does not exist";
    return;
  }

  if(_sdf->HasElement("default_current"))
  {
    this->dataPtr->currentVector = _sdf->Get<math::Vector3d>("default_current");
  }

  this->dataPtr->prevState = Eigen::VectorXd::Zero(6);

  AddWorldPose(this->dataPtr->linkEntity, _ecm);
  AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
  AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);


  // Added mass according to Fossen's equations (p 37)
  this->dataPtr->Ma = Eigen::MatrixXd::Zero(6, 6);

  this->dataPtr->Ma(0, 0) = this->dataPtr->paramXdotU;
  this->dataPtr->Ma(1, 1) = this->dataPtr->paramYdotV;
  this->dataPtr->Ma(2, 2) = this->dataPtr->paramZdotW;
  this->dataPtr->Ma(3, 3) = this->dataPtr->paramKdotP;
  this->dataPtr->Ma(4, 4) = this->dataPtr->paramMdotQ;
  this->dataPtr->Ma(5, 5) = this->dataPtr->paramNdotR;
}

/////////////////////////////////////////////////
void Hydrodynamics::PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // These variables follow Fossen's scheme in "Guidance and Control
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

  // Get vehicle state
  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto linearVelocity =
    _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
  auto rotationalVelocity = baseLink.WorldAngularVelocity(_ecm);

  if (!linearVelocity)
  {
    ignerr << "no linear vel" <<"\n";
    return;
  }

  // Get current vector
  math::Vector3d currentVector;
  {
    std::lock_guard lock(this->dataPtr->mtx);
    currentVector = this->dataPtr->currentVector;
  }
  // Transform state to local frame
  auto pose = baseLink.WorldPose(_ecm);
  // Since we are transforming angular and linear velocity we only care about
  // rotation
  auto localLinearVelocity = pose->Rot().Inverse() *
    (linearVelocity->Data() - currentVector);
  auto localRotationalVelocity = pose->Rot().Inverse() * *rotationalVelocity;

  state(0) = localLinearVelocity.X();
  state(1) = localLinearVelocity.Y();
  state(2) = localLinearVelocity.Z();

  state(3) = localRotationalVelocity.X();
  state(4) = localRotationalVelocity.Y();
  state(5) = localRotationalVelocity.Z();

  auto dt = static_cast<double>(_info.dt.count())/1e9;
  stateDot = (state - this->dataPtr->prevState)/dt;

  this->dataPtr->prevState = state;

  // The added mass
  const Eigen::VectorXd kAmassVec = this->dataPtr->Ma * stateDot;

  // Coriolis and Centripetal forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF
  Cmat(0, 4) = - this->dataPtr->paramZdotW * state(2);
  Cmat(0, 5) = - this->dataPtr->paramYdotV * state(1);
  Cmat(1, 3) =   this->dataPtr->paramZdotW * state(2);
  Cmat(1, 5) = - this->dataPtr->paramXdotU * state(0);
  Cmat(2, 3) = - this->dataPtr->paramYdotV * state(1);
  Cmat(2, 4) =   this->dataPtr->paramXdotU * state(0);
  Cmat(3, 1) = - this->dataPtr->paramZdotW * state(2);
  Cmat(3, 2) =   this->dataPtr->paramYdotV * state(1);
  Cmat(3, 4) = - this->dataPtr->paramNdotR * state(5);
  Cmat(3, 5) =   this->dataPtr->paramMdotQ * state(4);
  Cmat(4, 0) =   this->dataPtr->paramZdotW * state(2);
  Cmat(4, 2) = - this->dataPtr->paramXdotU * state(0);
  Cmat(4, 3) =   this->dataPtr->paramNdotR * state(5);
  Cmat(4, 5) = - this->dataPtr->paramKdotP * state(3);
  Cmat(5, 0) =   this->dataPtr->paramZdotW * state(2);
  Cmat(5, 1) =   this->dataPtr->paramXdotU * state(0);
  Cmat(5, 3) = - this->dataPtr->paramMdotQ * state(4);
  Cmat(5, 4) =   this->dataPtr->paramKdotP * state(3);
  const Eigen::VectorXd kCmatVec = - Cmat * state;

  // Damping forces (Fossen P. 43)
  Dmat(1, 1)
    = - this->dataPtr->paramYv - this->dataPtr->paramYvv * abs(state(1));
  Dmat(0, 0)
    = - this->dataPtr->paramXu - this->dataPtr->paramXuu * abs(state(0));
  Dmat(2, 2)
    = - this->dataPtr->paramZw - this->dataPtr->paramZww * abs(state(2));
  Dmat(3, 3)
    = - this->dataPtr->paramKp - this->dataPtr->paramKpp * abs(state(3));
  Dmat(4, 4)
    = - this->dataPtr->paramMq - this->dataPtr->paramMqq * abs(state(4));
  Dmat(5, 5)
    = - this->dataPtr->paramNr - this->dataPtr->paramNrr * abs(state(5));

  const Eigen::VectorXd kDvec = Dmat * state;

  const Eigen::VectorXd kTotalWrench = kAmassVec + kDvec + kCmatVec;

  gz::math::Vector3d
    totalForce(-kTotalWrench(0), -kTotalWrench(1), -kTotalWrench(2));
  gz::math::Vector3d
    totalTorque(-kTotalWrench(3), -kTotalWrench(4), -kTotalWrench(5));

  baseLink.AddWorldWrench(
    _ecm,
    pose->Rot()*(totalForce),
    pose->Rot()*totalTorque);
}

IGNITION_ADD_PLUGIN(
  Hydrodynamics, System,
  Hydrodynamics::ISystemConfigure,
  Hydrodynamics::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(
  Hydrodynamics,
  "gz::sim::systems::Hydrodynamics")
