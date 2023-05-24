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

#include <ignition/plugin/Register.hh>

#include "ignition/msgs/vector3d.pb.h"

#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/Util.hh"

#include "ignition/transport/Node.hh"

#include "Hydrodynamics.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Hydrodynamics data class.
class ignition::gazebo::systems::HydrodynamicsPrivateData
{
 /// \brief Contains the quadratic stability derivatives like $X_{uv}$,
  /// Y_{vv}, Y_{wv} etc.
  public: double stabilityQuadraticDerivative[216];

  /// \brief Contains the quadratic stability derivatives like $X_{u|u|},
  /// Y_{v|v|}, Y_{w|v|}$ etc.
  public: double stabilityQuadraticAbsDerivative[216];

  /// \brief Contains the linear stability derivatives like $X_u, Y_v,$ etc.
  public: double stabilityLinearTerms[36] = {0};

  /// \brief Water density [kg/m^3].
  public: double waterDensity;

  /// \brief The ignition transport node
  public: transport::Node node;

  /// \brief Plugin Parameter: Disable coriolis as part of equation. This is
  /// occasionally useful for testing.
  public: bool disableCoriolis = false;

  /// \brief Plugin Parameter: Disable added mass as part of equation. This is
  /// occasionally useful for testing.
  public: bool disableAddedMass = false;

  /// \brief Ocean current experienced by this body
  public: math::Vector3d currentVector {0, 0, 0};

  /// \brief Added mass of vehicle;
  /// See: https://en.wikipedia.org/wiki/Added_mass
  public: Eigen::MatrixXd Ma;

  /// \brief Previous state.
  public: Eigen::VectorXd prevState;

  public: Eigen::VectorXd prevStateDot;

  /// Link entity
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
  this->currentVector = ignition::msgs::Convert(_msg);
}

/////////////////////////////////////////////////
void AddAngularVelocityComponent(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(_entity))
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

/////////////////////////////////////////////////
void AddWorldPose(
  const ignition::gazebo::Entity &_entity,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<ignition::gazebo::components::WorldPose>(_entity))
  {
    _ecm.CreateComponent(_entity, ignition::gazebo::components::WorldPose());
  }
}

/////////////////////////////////////////////////
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
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/
)
{
  if (_sdf->HasElement("waterDensity"))
  {
    ignwarn <<
      "<waterDensity> parameter is deprecated and will be removed Ignition G.\n"
      << "\tPlease update your SDF to use <water_density> instead.";
  }

  this->dataPtr->waterDensity = SdfParamDouble(_sdf, "waterDensity",
                                  SdfParamDouble(_sdf, "water_density", 998)
                                );
  // Load stability derivatives
  // Use SNAME 1950 convention to load the coeffecients.
  const auto snameConventionVel = "UVWPQR";
  const auto snameConventionMoment = "xyzkmn";

  bool warnBehaviourChange = false;
  for(auto i = 0; i < 6; i++)
  {
    for(auto j = 0; j < 6; j++)
    {
      std::string prefix = {snameConventionMoment[i]};
      prefix += snameConventionVel[j];
      this->dataPtr->stabilityLinearTerms[i*6 + j] =
        SdfParamDouble(_sdf, prefix, 0);
      for(auto k = 0; k < 6; k++)
      {
        auto fieldName = prefix + snameConventionVel[k];
        this->dataPtr->stabilityQuadraticDerivative[i*36 + j*6 + k] =
          SdfParamDouble(
            _sdf,
            fieldName,
            0);

        if (_sdf->HasElement(fieldName)) {
          warnBehaviourChange = true;
        }

        this->dataPtr->stabilityQuadraticAbsDerivative[i*36 + j*6 + k] =
          SdfParamDouble(
            _sdf,
            prefix + "abs" + snameConventionVel[k],
            0);
      }
    }
  }


  if (warnBehaviourChange)
  {
    ignwarn << "You are using parameters that may cause instabilities "
      << "in your simulation. If your simulation crashes we recommend "
      << "renaming <xUU> -> <xUabsU> and likewise for other axis "
      << "for more information see:" << std::endl
      << "\thttps://github.com/gazebosim/gz-sim/pull/1888" << std::endl;
  }

  // Added mass according to Fossen's equations (p 37)
  this->dataPtr->Ma = Eigen::MatrixXd::Zero(6, 6);
  for(auto i = 0; i < 6; i++)
  {
    for(auto j = 0; j < 6; j++)
    {
      std::string prefix = {snameConventionMoment[i]};
      prefix += "Dot";
      prefix += snameConventionVel[j];
      this->dataPtr->Ma(i, j) = SdfParamDouble(_sdf, prefix, 0);
    }
  }

  _sdf->Get<bool>("disable_coriolis", this->dataPtr->disableCoriolis, false);
  _sdf->Get<bool>("disable_added_mass", this->dataPtr->disableAddedMass, false);

  // Create model object, to access convenient functions
  auto model = ignition::gazebo::Model(_entity);

  std::string ns {""};
  std::string currentTopic {"/ocean_current"};
  if (_sdf->HasElement("namespace"))
  {
    ns = _sdf->Get<std::string>("namespace");
    currentTopic = ignition::transport::TopicUtils::AsValidTopic(
        "/model/" + ns + "/ocean_current");
  }

  this->dataPtr->node.Subscribe(
    currentTopic,
    &HydrodynamicsPrivateData::UpdateCurrent,
    this->dataPtr.get());

  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "You must specify a <link_name> for the hydrodynamic"
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
}

/////////////////////////////////////////////////
void Hydrodynamics::PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // These variables follow Fossen's scheme in "Guidance and Control
  // of Ocean Vehicles." The `state` vector contains the ship's current velocity
  // in the format [x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel].
  // `stateDot` consists of the first derivative in time of the state vector.
  // `Cmat` corresponds to the Centripetal matrix
  // `Dmat` is the drag matrix
  // `Ma` is the added mass.
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  // Get vehicle state
  ignition::gazebo::Link baseLink(this->dataPtr->linkEntity);
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
  // rotation. Also this is where we apply the effects of current to the link
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
  // Negative sign signifies the behaviour change
  const Eigen::VectorXd kAmassVec = - this->dataPtr->Ma * stateDot;

  // Coriolis and Centripetal forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF. We are just considering
  // diagonal terms here. Have yet to add the cross terms here. Also note, since
  // $M_a(0,0) = \dot X_u $ , $M_a(1,1) = \dot Y_v $ and so forth, we simply
  // load the stability derivatives from $M_a$.
  Cmat(0, 4) = - this->dataPtr->Ma(2, 2) * state(2);
  Cmat(0, 5) = - this->dataPtr->Ma(1, 1) * state(1);
  Cmat(1, 3) =   this->dataPtr->Ma(2, 2) * state(2);
  Cmat(1, 5) = - this->dataPtr->Ma(0, 0) * state(0);
  Cmat(2, 3) = - this->dataPtr->Ma(1, 1) * state(1);
  Cmat(2, 4) =   this->dataPtr->Ma(0, 0) * state(0);
  Cmat(3, 1) = - this->dataPtr->Ma(2, 2) * state(2);
  Cmat(3, 2) =   this->dataPtr->Ma(1, 1) * state(1);
  Cmat(3, 4) = - this->dataPtr->Ma(5, 5) * state(5);
  Cmat(3, 5) =   this->dataPtr->Ma(4, 4) * state(4);
  Cmat(4, 0) =   this->dataPtr->Ma(2, 2) * state(2);
  Cmat(4, 2) = - this->dataPtr->Ma(0, 0) * state(0);
  Cmat(4, 3) =   this->dataPtr->Ma(5, 5) * state(5);
  Cmat(4, 5) = - this->dataPtr->Ma(3, 3) * state(3);
  Cmat(5, 0) =   this->dataPtr->Ma(2, 2) * state(2);
  Cmat(5, 1) =   this->dataPtr->Ma(0, 0) * state(0);
  Cmat(5, 3) = - this->dataPtr->Ma(4, 4) * state(4);
  Cmat(5, 4) =   this->dataPtr->Ma(3, 3) * state(3);
  const Eigen::VectorXd kCmatVec = - Cmat * state;

  // Damping forces
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0; j < 6; j++)
    {
      auto coeff = - this->dataPtr->stabilityLinearTerms[i * 6 + j];
      for(int k = 0; k < 6; k++)
      {
        auto index = i * 36 + j * 6 + k;
        auto absCoeff =
          this->dataPtr->stabilityQuadraticAbsDerivative[index] * abs(state(k));
        coeff -= absCoeff;
        auto velCoeff =
          this->dataPtr->stabilityQuadraticDerivative[index] * state(k);
        coeff -= velCoeff;
      }

      Dmat(i, j) = coeff;
    }
  }

  const Eigen::VectorXd kDvec = Dmat * state;

  Eigen::VectorXd kTotalWrench = kDvec;

  if (!this->dataPtr->disableAddedMass)
  {
    kTotalWrench += kAmassVec;
  }
  if (!this->dataPtr->disableCoriolis)
  {
    kTotalWrench += kCmatVec;
  }

  math::Vector3d totalForce(
    -kTotalWrench(0),  -kTotalWrench(1), -kTotalWrench(2));
  math::Vector3d totalTorque(
    -kTotalWrench(3),  -kTotalWrench(4), -kTotalWrench(5));

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
  "ignition::gazebo::systems::Hydrodynamics")
