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

#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/plugin/Register.hh>

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Environment.hh"
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

  /// \brief The Gazebo Transport node
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

  /// \brief Previous derivative of the state
  public: Eigen::VectorXd prevStateDot;

  /// \brief Use current table if true
  public: bool useCurrentTable {false};

  /// \brief Current table xComponent
  public: std::string axisComponents[3];

  public: std::shared_ptr<gz::sim::components::EnvironmentalData>
    gridField;

  public: std::optional<gz::math::InMemorySession<double, double>> session[3];

  /// \brief Link entity
  public: Entity linkEntity;

  /// \brief Ocean current callback
  public: void UpdateCurrent(const msgs::Vector3d &_msg);

  /////////////////////////////////////////////////
  /// \brief Set the current table
  /// \param[in] _ecm - The Entity Component Manager
  /// \param[in] _currTime - The current time
  public: void SetWaterCurrentTable(
    const EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_currTime)
  {
    _ecm.EachNew<components::Environment>([&](const Entity &/*_entity*/,
      const components::Environment *_environment) -> bool
    {
      this->gridField = _environment->Data();

      for (std::size_t i = 0; i < 3; i++)
      {
        if (!this->axisComponents[i].empty())
        {
          if (!this->gridField->frame.Has(this->axisComponents[i]))
          {
            gzwarn << "Environmental sensor could not find field "
              << this->axisComponents[i] << "\n";
            continue;
          }

          this->session[i] =
            this->gridField->frame[this->axisComponents[i]].CreateSession();
          if (!this->gridField->staticTime)
          {
            this->session[i] =
              this->gridField->frame[this->axisComponents[i]].StepTo(
                *this->session[i],
                std::chrono::duration<double>(_currTime).count());
          }

          if(!this->session[i].has_value())
          {
            gzerr << "Exceeded time stamp." << std::endl;
          }
        }
      }
      return true;
    });
  }

  /////////////////////////////////////////////////
  /// \brief Retrieve water current data from the environment
  /// \param[in] _ecm - The Entity Component Manager
  /// \param[in] _currTime - The current time
  /// \param[in] _position - Position of the vehicle in world coordinates.
  /// \return The current vector to be applied at this position and time.
  public: math::Vector3d GetWaterCurrentFromEnvironment(
    const EntityComponentManager &_ecm,
    const std::chrono::steady_clock::duration &_currTime,
    math::Vector3d _position)
  {
    math::Vector3d current(0, 0, 0);

    if (!this->gridField ||
        !(this->session[0].has_value() ||
          this->session[1].has_value() || this->session[2].has_value()))
    {
      return current;
    }

    for (std::size_t i = 0; i < 3; i++)
    {
      if (!this->axisComponents[i].empty())
      {
        if (!this->gridField->staticTime)
        {
          this->session[i] =
            this->gridField->frame[this->axisComponents[i]].StepTo(
              *this->session[i],
              std::chrono::duration<double>(_currTime).count());
        }

        if (!this->session[i].has_value())
        {
          gzerr << "Time exceeded" << std::endl;
          continue;
        }

        auto position = getGridFieldCoordinates(
          _ecm, _position, this->gridField);

        if (!position.has_value())
        {
          gzerr << "Coordinate conversion failed" << std::endl;
          continue;
        }

        auto data = this->gridField->frame[this->axisComponents[i]].LookUp(
          this->session[i].value(), position.value());
        if (!data.has_value())
        {
          auto bounds =
            this->gridField->frame[this->axisComponents[i]].Bounds(
              this->session[i].value());
          gzwarn << "Failed to acquire value perhaps out of field?\n"
            << "Bounds are " << bounds.first << ", "
            << bounds.second << std::endl;
          continue;
        }

        current[i] = data.value();
      }
    }

    return current;
  }
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
  this->dataPtr->waterDensity = SdfParamDouble(_sdf, "water_density", 998);
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
    gzwarn << "You are using parameters that may cause instabilities "
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
    gzerr << "You must specify a <link_name> for the hydrodynamic"
      << " plugin to act upon";
    return;
  }
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity))
  {
    gzerr << "Link name" << linkName << "does not exist";
    return;
  }

  if(_sdf->HasElement("default_current"))
  {
    this->dataPtr->currentVector = _sdf->Get<math::Vector3d>("default_current");
  }

  this->dataPtr->prevState = Eigen::VectorXd::Zero(6);

  if(_sdf->HasElement("lookup_current_x"))
  {
    this->dataPtr->useCurrentTable = true;
    this->dataPtr->axisComponents[0] =
      _sdf->Get<std::string>("lookup_current_x");
  }

  if(_sdf->HasElement("lookup_current_y"))
  {
    this->dataPtr->useCurrentTable = true;
    this->dataPtr->axisComponents[1] =
      _sdf->Get<std::string>("lookup_current_y");
  }

  if(_sdf->HasElement("lookup_current_z"))
  {
    this->dataPtr->useCurrentTable = true;
    this->dataPtr->axisComponents[2] =
      _sdf->Get<std::string>("lookup_current_z");
  }


  AddWorldPose(this->dataPtr->linkEntity, _ecm);
  AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
  AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);
}

/////////////////////////////////////////////////
void Hydrodynamics::PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
{
  if (this->dataPtr->useCurrentTable)
  {
    this->dataPtr->SetWaterCurrentTable(_ecm, _info.simTime);
  }

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
  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto linearVelocity =
    _ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity);
  auto rotationalVelocity = baseLink.WorldAngularVelocity(_ecm);

  if (!linearVelocity)
  {
    gzerr << "no linear vel" <<"\n";
    return;
  }

  // Get current vector
  math::Vector3d currentVector(0, 0, 0);

  if (this->dataPtr->useCurrentTable)
  {
    auto position = baseLink.WorldInertialPose(_ecm);
    if (position.has_value())
    {
      currentVector = this->dataPtr->GetWaterCurrentFromEnvironment(
        _ecm, _info.simTime, position.value().Pos());
    }
  }
  else
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

/////////////////////////////////////////////////
void Hydrodynamics::PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
{
  if (this->dataPtr->useCurrentTable)
  {
    this->dataPtr->SetWaterCurrentTable(_ecm, _info.simTime);
  }
}

GZ_ADD_PLUGIN(
  Hydrodynamics, System,
  Hydrodynamics::ISystemConfigure,
  Hydrodynamics::ISystemPreUpdate,
  Hydrodynamics::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  Hydrodynamics,
  "gz::sim::systems::Hydrodynamics")
