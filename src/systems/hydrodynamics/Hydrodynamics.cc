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
#include <cmath>
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
#include "HydrodynamicsUtils.hh"

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
  public: Eigen::Matrix<double, 6, 6> Ma;

  /// \brief Previous state.
  public: Eigen::Matrix<double, 6, 1> prevState;

  /// \brief Whether this is the first simulation iteration.
  public: bool firstIteration {true};

  /// \brief Whether this link has <fluid_added_mass> in SDF.
  public: bool hasFluidAddedMass {false};

  /// \brief Fluid added mass matrix (positive physical values) from SDF.
  /// Only valid when hasFluidAddedMass is true.
  public: Eigen::Matrix<double, 6, 6> fluidAddedMass;

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
  // Note: Adding added mass here is deprecated for DART users (prefer
  // <fluid_added_mass>), but remains required for other physics engines.
  bool addedMassSpecified = false;
  this->dataPtr->Ma = Eigen::Matrix<double, 6, 6>::Zero();
  for(auto i = 0; i < 6; i++)
  {
    for(auto j = 0; j < 6; j++)
    {
      std::string prefix = {snameConventionMoment[i]};
      prefix += "Dot";
      prefix += snameConventionVel[j];
      this->dataPtr->Ma(i, j) = SdfParamDouble(_sdf, prefix, 0);
      addedMassSpecified = (std::abs(this->dataPtr->Ma(i, j)) > 1e-6)
        || addedMassSpecified;
    }
  }

  _sdf->Get<bool>("disable_coriolis", this->dataPtr->disableCoriolis, false);
  _sdf->Get<bool>("disable_added_mass",
    this->dataPtr->disableAddedMass, false);
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

  // Cache the fluid added mass matrix if present on this link.
  // This is used for the Coriolis current-velocity correction.
  {
    gz::sim::Link link(this->dataPtr->linkEntity);
    auto fluidMa = link.WorldFluidAddedMassMatrix(_ecm);
    if (fluidMa.has_value())
    {
      this->dataPtr->hasFluidAddedMass = true;
      const auto &m = fluidMa.value();
      for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
          this->dataPtr->fluidAddedMass(i, j) = m(i, j);
    }

    // Guard against double-counting added mass.
    // If both native <fluid_added_mass> and plugin added mass parameters are
    // active, forces are applied twice — once implicitly by the physics engine
    // and once explicitly by the plugin.
    if (fluidMa.has_value() && addedMassSpecified &&
        !this->dataPtr->disableAddedMass)
    {
      gzerr << "Link [" << linkName << "] has <fluid_added_mass> in SDF and "
        << "added mass parameters in the Hydrodynamics plugin. This will "
        << "double-count added mass forces. Either remove the plugin's added "
        << "mass parameters (<xDotU>, etc.) or set "
        << "<disable_added_mass>true</disable_added_mass> in the plugin."
        << std::endl;
    }
    // Context-dependent deprecation warning.
    // At this point, fluidMa is not set (otherwise the guard above would fire).
    else if (!this->dataPtr->disableAddedMass && addedMassSpecified)
    {
      // Plugin-only added mass — give nuanced advice covering all backends.
      gzwarn << "Plugin-based added mass has known numerical instabilities. "
        << "If using the DART physics engine, consider migrating to SDF "
        << "<fluid_added_mass> for unconditionally stable added mass "
        << "[http://sdformat.org/spec?ver=1.11&elem=link"
        << "#inertial_fluid_added_mass]. "
        << "Other physics engines (Bullet, MuJoCo) only support "
        << "plugin-based added mass." << std::endl;
    }
  }

  if(_sdf->HasElement("default_current"))
  {
    this->dataPtr->currentVector = _sdf->Get<math::Vector3d>("default_current");
  }

  this->dataPtr->prevState = Eigen::Matrix<double, 6, 1>::Zero();

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
  if (_info.paused)
    return;

  // These variables follow Fossen's scheme in "Guidance and Control
  // of Ocean Vehicles." The `state` vector contains the ship's current velocity
  // in the format [x_vel, y_vel, z_vel, roll_vel, pitch_vel, yaw_vel].
  // `stateDot` consists of the first derivative in time of the state vector.
  // `Cmat` corresponds to the Centripetal matrix
  // `Dmat` is the drag matrix
  // `Ma` is the added mass.
  Eigen::Matrix<double, 6, 1> stateDot;
  Eigen::Matrix<double, 6, 1> state;
  Eigen::Matrix<double, 6, 6> Cmat = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> Dmat = Eigen::Matrix<double, 6, 6>::Zero();

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

  // On the first iteration, initialize prevState from the actual velocity
  // to avoid a transient spike in the acceleration estimate.
  if (this->dataPtr->firstIteration)
  {
    this->dataPtr->prevState = state;
    this->dataPtr->firstIteration = false;
  }

  auto dt = static_cast<double>(_info.dt.count())/1e9;
  stateDot = (state - this->dataPtr->prevState)/dt;

  this->dataPtr->prevState = state;

  Eigen::Matrix<double, 6, 1> kTotalWrench =
    Eigen::Matrix<double, 6, 1>::Zero();

  // The added mass
  if (!this->dataPtr->disableAddedMass)
  {
    // Negative sign signifies the behaviour change
    const Eigen::Matrix<double, 6, 1> kAmassVec =
      - this->dataPtr->Ma * stateDot;
    kTotalWrench += kAmassVec;
  }

  // Coriolis and Centripetal forces for under water vehicles (Fossen P. 37)
  // Note: this is significantly different from VRX because we need to account
  // for the under water vehicle's additional DOF. We are just considering
  // diagonal terms here. Have yet to add the cross terms here. Also note, since
  // $M_a(0,0) = \dot X_u $ , $M_a(1,1) = \dot Y_v $ and so forth, we simply
  // load the stability derivatives from $M_a$.
  if (!this->dataPtr->disableCoriolis)
  {
    Cmat = hydrodynamics::buildCoriolisMatrix(this->dataPtr->Ma, state);
    const Eigen::Matrix<double, 6, 1> kCmatVec = - Cmat * state;
    kTotalWrench += kCmatVec;
  }

  // Damping forces
  Dmat = hydrodynamics::buildDampingMatrix(
    this->dataPtr->stabilityLinearTerms,
    this->dataPtr->stabilityQuadraticAbsDerivative,
    this->dataPtr->stabilityQuadraticDerivative,
    state);

  const Eigen::Matrix<double, 6, 1> kDvec = Dmat * state;

  kTotalWrench += kDvec;

  // Coriolis current-velocity correction for <fluid_added_mass>.
  //
  // DART implicitly computes C_A(v)*v using the body's absolute velocity v.
  // Fossen's equations require C_A(v_r)*v_r where v_r = v - v_current.
  // We apply a correction wrench so the net effect uses relative velocity:
  //   Applied = -kTotalWrench, so we add (Ca_rel*v_r - Ca_abs*v) to
  //   kTotalWrench, which produces external force (Ca_abs*v - Ca_rel*v_r).
  //   Combined with DART's implicit -C_A(v)*v, the total becomes -C_A(v_r)*v_r.
  if (this->dataPtr->hasFluidAddedMass)
  {
    // Build absolute body-frame velocity (without current subtraction).
    // Angular velocity is unaffected by ocean current.
    Eigen::Matrix<double, 6, 1> absState;
    auto localAbsLinearVelocity = pose->Rot().Inverse() *
      linearVelocity->Data();
    absState(0) = localAbsLinearVelocity.X();
    absState(1) = localAbsLinearVelocity.Y();
    absState(2) = localAbsLinearVelocity.Z();
    absState(3) = localRotationalVelocity.X();
    absState(4) = localRotationalVelocity.Y();
    absState(5) = localRotationalVelocity.Z();

    auto Ca_abs = hydrodynamics::buildFullCoriolisMatrix(
      this->dataPtr->fluidAddedMass, absState);
    auto Ca_rel = hydrodynamics::buildFullCoriolisMatrix(
      this->dataPtr->fluidAddedMass, state);

    kTotalWrench += Ca_rel * state - Ca_abs * absState;
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
