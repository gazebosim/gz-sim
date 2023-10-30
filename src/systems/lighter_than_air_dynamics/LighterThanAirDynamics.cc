/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/Inertial.hh"

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"

#include <sdf/Element.hh>

#include "gz/math/Matrix3.hh"

#include "gz/transport/Node.hh"

#include <mutex>
#include <limits>

#include "LighterThanAirDynamics.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private LighterThanAirDynamics data class.
class gz::sim::systems::LighterThanAirDynamicsPrivateData
{

  /// \brief air density [kg/m^3].
  public: double airDensity;

  /// \brief force coefficient of the viscous flow contribution to the hull
  public: double forceViscCoeff;

  /// \brief force coefficient of the inviscid flow contribution to the hull
  public: double forceInviscCoeff;

  /// \brief moment coefficient of the viscous flow contribution to the hull
  public: double momentViscCoeff;

  /// \brief moment coefficient of the inviscid flow contribution to the hull
  public: double momentInviscCoeff;

  /// \brief Top left [3x3] matrix of the [6x6] added mass matrix
  public: math::Matrix3d m11;

  /// \brief Top right [3x3] matrix of the [6x6] added mass matrix
  public: math::Matrix3d m12;

  /// \brief Bottom left [3x3] matrix of the [6x6] added mass matrix
  public: math::Matrix3d m21;

  /// \brief Bottom right [3x3] matrix of the [6x6] added mass matrix
  public: math::Matrix3d m22;

  /// \brief distance measured from the nose, where the flow stopped
  /// being potential
  public: double epsV;

  /// \brief axial drag coefficient of the hull
  public: double axialDragCoeff;

  /// \brief The Gazebo Transport node
  public: transport::Node node;

  /// \brief Link entity
  public: Entity linkEntity;

  /// \brief World frame wind
  public: math::Vector3d windVector {0, 0, 0};

  /// \brief wind current callback
  public: void UpdateWind(const msgs::Vector3d &_msg);

  /// \brief Mutex
  public: std::mutex mtx;
};

/////////////////////////////////////////////////
void LighterThanAirDynamicsPrivateData::UpdateWind(const msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  this->windVector = gz::msgs::Convert(_msg);
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
void AddWorldInertial(
  const gz::sim::Entity &_entity,
  gz::sim::EntityComponentManager &_ecm)
{
  if (!_ecm.Component<gz::sim::components::Inertial>(_entity))
  {
    _ecm.CreateComponent(_entity, gz::sim::components::Inertial());
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

math::Matrix3d LighterThanAirDynamics::SkewSymmetricMatrix(math::Vector3d mat)
{
  math::Matrix3d skewSymmetric(0, -1.0*mat[2], mat[1],
                                mat[2], 0, -1.0*mat[0],
                                -1.0*mat[1], mat[0], 0);


  return skewSymmetric;
}


math::Vector3d LighterThanAirDynamics::AddedMassForce(
    math::Vector3d lin_vel, math::Vector3d ang_vel,
    math::Matrix3d m11, math::Matrix3d m12)
{
        auto skewAngVel = this->SkewSymmetricMatrix(ang_vel);
        math::Vector3d forces = skewAngVel * (m11 * lin_vel + m12 * ang_vel);

        return forces;
}

math::Vector3d LighterThanAirDynamics::AddedMassTorque(
    math::Vector3d lin_vel, math::Vector3d ang_vel,
    math::Matrix3d m11, math::Matrix3d m12,  math::Matrix3d m21,
    math::Matrix3d m22)
{
        auto skewAngVel = this->SkewSymmetricMatrix(ang_vel);
        auto skewLinVel = this->SkewSymmetricMatrix(lin_vel);

        // note: the m11*lin_vel term: it is already accounted in the
        // inviscous term see [2], and thus removed by the zero multiplication,
        // but is added here for visibility.
        math::Vector3d torque = skewLinVel * (m11 * lin_vel*0 + m12 * ang_vel)
                              + skewAngVel * (m21 * lin_vel + m22 * ang_vel);

        return torque;
}

math::Vector3d LighterThanAirDynamics::LocalVelocity(math::Vector3d lin_vel,
                                  math::Vector3d ang_vel, math::Vector3d dist)
{
  return lin_vel + ang_vel.Cross(dist);
}

double LighterThanAirDynamics::DynamicPressure(
    math::Vector3d vec, double air_density)
{
  return 0.5 * air_density * vec.SquaredLength();
}

/////////////////////////////////////////////////
LighterThanAirDynamics::LighterThanAirDynamics()
{
  this->dataPtr = std::make_unique<LighterThanAirDynamicsPrivateData>();
}

/////////////////////////////////////////////////
LighterThanAirDynamics::~LighterThanAirDynamics()
{
  // Do nothing
}

/////////////////////////////////////////////////
void LighterThanAirDynamics::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/
)
{
  if (_sdf->HasElement("air_density"))
  {
    this->dataPtr->airDensity = SdfParamDouble(_sdf, "air_density", 1.225);
  }

  // Create model object, to access convenient functions
  auto model = gz::sim::Model(_entity);

  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "You must specify a <link_name> for the lighter than air"
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

  if(_sdf->HasElement("moment_inviscid_coeff"))
  {
    this->dataPtr->momentInviscCoeff =
                _sdf->Get<double>("moment_inviscid_coeff");

    gzdbg << "moment_inviscid_coeff: "
          << this->dataPtr->momentInviscCoeff << "\n";
  }else{
    gzerr << "moment_inviscid_coeff not found \n";
  }

  if(_sdf->HasElement("moment_viscous_coeff"))
  {
    this->dataPtr->momentViscCoeff = _sdf->Get<double>("moment_viscous_coeff");
    gzdbg << "moment_viscous_coeff: "
          << this->dataPtr->momentViscCoeff << "\n";
  }else{
    gzerr << "moment_viscous_coeff not found \n";
    return;
  }

  if(_sdf->HasElement("force_inviscid_coeff"))
  {
    this->dataPtr->forceInviscCoeff =
                  _sdf->Get<double>("force_inviscid_coeff");

    gzdbg << "force_inviscid_coeff: "
          << this->dataPtr->forceInviscCoeff << "\n";
  }else{
    gzerr << "force_inviscid_coeff not found \n";
    return;
  }

  if(_sdf->HasElement("force_viscous_coeff"))
  {
    this->dataPtr->forceViscCoeff = _sdf->Get<double>("force_viscous_coeff");

    gzdbg << "force_inviscous_coeff: " << this->dataPtr->forceViscCoeff << "\n";
  }else{
    gzerr << "force_inviscous_coeff not found \n";
    return;
  }

  if(_sdf->HasElement("eps_v"))
  {
    this->dataPtr->epsV = _sdf->Get<double>("eps_v");
    gzdbg << "eps_v: " << this->dataPtr->epsV << "\n";
  }else{
    gzerr << "eps_v not found \n";
    return;
  }

  if(_sdf->HasElement("axial_drag_coeff"))
  {
    this->dataPtr->axialDragCoeff = _sdf->Get<double>("axial_drag_coeff");
    gzdbg << "axial_drag_coeff: " << this->dataPtr->axialDragCoeff << "\n";
  }else{
    gzerr << "axial_drag_coeff not found \n";
    return;
  }

  AddWorldPose(this->dataPtr->linkEntity, _ecm);
  AddWorldInertial(this->dataPtr->linkEntity, _ecm);
  AddAngularVelocityComponent(this->dataPtr->linkEntity, _ecm);
  AddWorldLinearVelocity(this->dataPtr->linkEntity, _ecm);

  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  math::Matrix6d added_mass_matrix =
                          (baseLink.WorldFluidAddedMassMatrix(_ecm)).value();

  this->dataPtr->m11 = added_mass_matrix.Submatrix(
                                  math::Matrix6d::Matrix6Corner::TOP_LEFT);
  this->dataPtr->m12 = added_mass_matrix.Submatrix(
                                  math::Matrix6d::Matrix6Corner::TOP_RIGHT);
  this->dataPtr->m21 = added_mass_matrix.Submatrix(
                                  math::Matrix6d::Matrix6Corner::BOTTOM_LEFT);
  this->dataPtr->m22 = added_mass_matrix.Submatrix(
                                  math::Matrix6d::Matrix6Corner::BOTTOM_RIGHT);
}

/////////////////////////////////////////////////
void LighterThanAirDynamics::PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Get vehicle state
  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto linearVelocity = _ecm.Component<components::WorldLinearVelocity>(
                                              this->dataPtr->linkEntity);
  auto rotationalVelocity = baseLink.WorldAngularVelocity(_ecm);

  if (!linearVelocity)
  {
    gzerr << "no linear vel" <<"\n";
    return;
  }

  // Transform state to local frame
  auto pose = baseLink.WorldPose(_ecm);
  // Since we are transforming angular and linear velocity we only care about
  // rotation. Also this is where we apply the effects of current to the link
  auto linearVel = pose->Rot().Inverse() * (linearVelocity->Data());
  auto angularVelocity = pose->Rot().Inverse() * *rotationalVelocity;

  // Calculate viscous forces
  math::Vector3d velEpsV = LocalVelocity(linearVel, angularVelocity,
                                    math::Vector3d(-this->dataPtr->epsV, 0, 0));

  double q0EpsV = DynamicPressure(velEpsV, this->dataPtr->airDensity);
  double gammaEpsV = 0.0f;

  gammaEpsV = std::atan2(std::sqrt(velEpsV[1]*velEpsV[1] +
                                     velEpsV[2]*velEpsV[2]), velEpsV[0]);

  double forceInviscCoeff = this->dataPtr->forceInviscCoeff;
  double forceViscCoeff = this->dataPtr->forceViscCoeff;
  double momentInviscCoeff = this->dataPtr->momentInviscCoeff;
  double momentViscCoeff = this->dataPtr->momentViscCoeff;

  double forceViscMag_ = q0EpsV*(-forceInviscCoeff*std::sin(2*gammaEpsV)
                + forceViscCoeff*std::sin(gammaEpsV)*std::sin(gammaEpsV));

  double momentViscMag_ = q0EpsV*(-momentInviscCoeff*std::sin(2*gammaEpsV)
                + momentViscCoeff*std::sin(gammaEpsV)*std::sin(gammaEpsV));

  double viscNormalMag_ = std::sqrt(velEpsV[1]*velEpsV[1] +
                                      velEpsV[2]*velEpsV[2]);

  double viscNormalY_ = 0.0;
  double viscNormalZ_ = 0.0;

  if(viscNormalMag_ > std::numeric_limits<double>::epsilon()){

    viscNormalY_ = velEpsV[1]/viscNormalMag_;
    viscNormalZ_ = velEpsV[2]/viscNormalMag_;
  }

  auto forceVisc = forceViscMag_  * math::Vector3d(0,
                                                      -viscNormalY_,
                                                      -viscNormalZ_);

  auto momentVisc = momentViscMag_ * math::Vector3d(0,
                                                      viscNormalZ_,
                                                      -viscNormalY_);

  // Added Mass forces & Torques
  auto forceAddedMass = -1.0 * this->AddedMassForce(linearVel,
                                            angularVelocity,
                                            this->dataPtr->m11,
                                            this->dataPtr->m12);

  auto momentAddedMass = -1.0 * this->AddedMassTorque(linearVel,
                                            angularVelocity,
                                            this->dataPtr->m11,
                                            this->dataPtr->m12,
                                            this->dataPtr->m21,
                                            this->dataPtr->m22);

  // Axial drag
  double q0 = DynamicPressure(linearVel, this->dataPtr->airDensity);
  double angleOfAttack = std::atan2(linearVel[2], linearVel[0]);
  double axialDragCoeff = this->dataPtr->axialDragCoeff;

  auto forceAxialDrag = math::Vector3d(-q0 * axialDragCoeff *
              std::cos(angleOfAttack) * std::cos(angleOfAttack), 0, 0);

  math::Vector3d totalForce = forceAddedMass + forceVisc + forceAxialDrag;
  math::Vector3d totalTorque = momentAddedMass + momentVisc;

  baseLink.AddWorldWrench(_ecm,
                          pose->Rot()*(totalForce),
                          pose->Rot()*totalTorque);
}

GZ_ADD_PLUGIN(
  LighterThanAirDynamics, System,
  LighterThanAirDynamics::ISystemConfigure,
  LighterThanAirDynamics::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(
  LighterThanAirDynamics,
  "gz::sim::systems::LighterThanAirDynamics")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(
  LighterThanAirDynamics,
  "ignition::gazebo::systems::LighterThanAirDynamics")
