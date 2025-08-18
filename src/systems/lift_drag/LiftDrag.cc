/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "LiftDrag.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LiftDragPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Coefficient of Lift / alpha slope.
  /// Lift = C_L * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cla = 1.0;

  /// \brief Coefficient of Drag / alpha slope.
  /// Drag = C_D * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cda = 0.01;

  /// \brief Coefficient of Moment / alpha slope.
  /// Moment = C_M * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cma = 0.0;

  /// \brief angle of attach when airfoil stalls
  public: double alphaStall = GZ_PI_2;

  /// \brief Cl-alpha rate after stall
  public: double claStall = 0.0;

  /// \brief Cd-alpha rate after stall
  /// \todo(anyone): what's flat plate drag?
  public: double cdaStall = 1.0;

  /// \brief Cm-alpha rate after stall
  public: double cmaStall = 0.0;

  /// \brief How much Cm changes with a change in control
  /// surface deflection angle
  public: double cm_delta = 0.0;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  public: double rho = 1.2041;

  /// \brief if the shape is aerodynamically radially symmetric about
  /// the forward direction. Defaults to false for wing shapes.
  /// If set to true, the upward direction is determined by the
  /// angle of attack.
  public: bool radialSymmetry = false;

  /// \brief effective planeform surface area
  public: double area = 1.0;

  /// \brief initial angle of attack
  public: double alpha0 = 0.0;

  /// \brief center of pressure in link local coordinates with respect to the
  /// link's center of mass
  public: gz::math::Vector3d cp = math::Vector3d::Zero;

  /// \brief Normally, this is taken as a direction parallel to the chord
  /// of the airfoil in zero angle of attack forward flight.
  public: math::Vector3d forward = math::Vector3d::UnitX;

  /// \brief A vector in the lift/drag plane, perpendicular to the forward
  /// vector. Inflow velocity orthogonal to forward and upward vectors
  /// is considered flow in the wing sweep direction.
  public: math::Vector3d upward = math::Vector3d::UnitZ;

  /// \brief how much to change CL per radian of control surface joint
  /// value.
  public: double controlJointRadToCL = 4.0;

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;

  /// \brief Joint entity that actuates a control surface for this lifting body
  public: Entity controlJointEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};
};

//////////////////////////////////////////////////
void LiftDragPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  if (!this->model.Valid(_ecm))
  {
    gzerr << "The LiftDrag system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->cla = _sdf->Get<double>("cla", this->cla).first;
  this->cda = _sdf->Get<double>("cda", this->cda).first;
  this->cma = _sdf->Get<double>("cma", this->cma).first;
  this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
  this->claStall = _sdf->Get<double>("cla_stall", this->claStall).first;
  this->cdaStall = _sdf->Get<double>("cda_stall", this->cdaStall).first;
  this->cmaStall = _sdf->Get<double>("cma_stall", this->cmaStall).first;
  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->radialSymmetry = _sdf->Get<bool>("radial_symmetry",
      this->radialSymmetry).first;
  this->area = _sdf->Get<double>("area", this->area).first;
  this->alpha0 = _sdf->Get<double>("a0", this->alpha0).first;
  this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
  this->cm_delta = _sdf->Get<double>("cm_delta", this->cm_delta).first;

  // blade forward (-drag) direction in link frame
  this->forward =
      _sdf->Get<math::Vector3d>("forward", this->forward).first;
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  this->upward = _sdf->Get<math::Vector3d>(
      "upward", this->upward) .first;
  this->upward.Normalize();

  this->controlJointRadToCL = _sdf->Get<double>(
      "control_joint_rad_to_cl", this->controlJointRadToCL).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
             << "The LiftDrag will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The LiftDrag system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    auto controlJointName = _sdf->Get<std::string>("control_joint_name");
    auto entities =
        entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Joint with name[" << controlJointName << "] not found. "
             << "The LiftDrag will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple joint entities with name[" << controlJointName
              << "] found. Using the first one.\n";
    }

    this->controlJointEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->controlJointEntity,
                                     components::Joint::typeId))
    {
      this->controlJointEntity = kNullEntity;
      gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
      this->validConfig = false;
      return;
    }
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
LiftDrag::LiftDrag()
    : System(), dataPtr(std::make_unique<LiftDragPrivate>())
{
}

//////////////////////////////////////////////////
void LiftDragPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDragPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  // get wind as a component from the _ecm
  components::WorldLinearVelocity *windLinearVel = nullptr;
  if(_ecm.EntityByComponents(components::Wind()) != kNullEntity){
    Entity windEntity = _ecm.EntityByComponents(components::Wind());
    windLinearVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
  }
  components::JointPosition *controlJointPosition = nullptr;
  if (this->controlJointEntity != kNullEntity)
  {
    controlJointPosition =
        _ecm.Component<components::JointPosition>(this->controlJointEntity);
  }

  if (!worldLinVel || !worldAngVel || !worldPose)
  {
    return;
  }

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->cp);
  auto vel = worldLinVel->Data() + worldAngVel->Data().Cross(
  cpWorld);
  if (windLinearVel != nullptr){
    vel = worldLinVel->Data() + worldAngVel->Data().Cross(
    cpWorld) - windLinearVel->Data();
  }

  if (vel.Length() <= 0.01)
  {
    return;
  }

  const auto velI = vel.Normalized();

  // rotate forward and upward vectors into world frame
  const auto forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity
    // is in the same direction
    return;
  }

  math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in world frame
  const auto spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // The sweep adjustment depends on the velocity component normal to
  // the wing leading edge which appears quadratically in the
  // dynamic pressure, so scale by cos^2 .
  double cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  double sweep = std::asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (std::fabs(sweep) > 0.5 * GZ_PI)
  {
    sweep = sweep > 0 ? sweep - GZ_PI : sweep + GZ_PI;
  }

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  // Note: Original code had:
  //    const auto velInLDPlane = vel - vel.Dot(spanwiseI)*velI;
  // I believe the projection should be onto spanwiseI which then gets removed
  // from vel
  const auto velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  const auto dragDirection = -velInLDPlane.Normalized();

  // get direction of lift
  const auto liftI = spanwiseI.Cross(velInLDPlane).Normalized();

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Length())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  const double cosAlpha =
      math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  double alpha = this->alpha0 - std::acos(cosAlpha);
  if (liftI.Dot(forwardI) >= 0.0)
    alpha = this->alpha0 + std::acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(alpha) > 0.5 * GZ_PI)
  {
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;
  }

  // compute dynamic pressure
  const double speedInLDPlane = velInLDPlane.Length();
  const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall +
          this->claStall * (alpha - this->alphaStall)) *
         cos2SweepAngle;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall +
          this->claStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = this->cla * alpha * cos2SweepAngle;

  // modify cl per control joint value
  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cl = cl + this->controlJointRadToCL * controlJointPosition->Data()[0];
    /// \todo(anyone): also change cm and cd
  }

  // compute lift force at cp
  math::Vector3d lift = cl * q * this->area * liftI;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall +
          this->cdaStall * (alpha - this->alphaStall))
         * cos2SweepAngle;
  }
  else if (alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall +
          this->cdaStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
  }
  else
    cd = (this->cda * alpha) * cos2SweepAngle;

  // make sure drag is positive
  cd = std::fabs(cd);

  // drag at cp
  math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (alpha - this->alphaStall))
         * cos2SweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * alpha * cos2SweepAngle;

  // Take into account the effect of control surface deflection angle to cm
  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cm += this->cm_delta * controlJointPosition->Data()[0];
  }

  // compute moment (torque) at cp
  // spanwiseI used to be momentDirection
  math::Vector3d moment = cm * q * this->area * spanwiseI;

  // force and torque about cg in world frame
  math::Vector3d force = lift + drag;
  math::Vector3d torque = moment;
  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // We want to apply the force at cp. The old LiftDrag plugin did the
  // following:
  //     this->link->AddForceAtRelativePosition(force, this->cp);
  // The documentation of AddForceAtRelativePosition says:
  //> Add a force (in world frame coordinates) to the body at a
  //> position relative to the center of mass which is expressed in the
  //> link's own frame of reference.
  // But it appears that 'cp' is specified in the link frame so it probably
  // should have been
  //     this->link->AddForceAtRelativePosition(
  //         force, this->cp - this->link->GetInertial()->CoG());
  //
  // \todo(addisu) Create a convenient API for applying forces at offset
  // positions
  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);

  // Debug
  // auto linkName = _ecm.Component<components::Name>(this->linkEntity)->Data();
  // gzdbg << "=============================\n";
  // gzdbg << "Link: [" << linkName << "] pose: [" << pose
  //        << "] dynamic pressure: [" << q << "]\n";
  // gzdbg << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
  // gzdbg << "LD plane spd: [" << velInLDPlane.Length() << "] vel : ["
  //        << velInLDPlane << "]\n";
  // gzdbg << "forward (inertial): " << forwardI << "\n";
  // gzdbg << "upward (inertial): " << upwardI << "\n";
  // gzdbg << "q: " << q << "\n";
  // gzdbg << "cl: " << cl << "\n";
  // gzdbg << "lift dir (inertial): " << liftI << "\n";
  // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
  // gzdbg << "sweep: " << sweep << "\n";
  // gzdbg << "alpha: " << alpha << "\n";
  // gzdbg << "lift: " << lift << "\n";
  // gzdbg << "drag: " << drag << " cd: " << cd << " cda: "
  //        << this->cda << "\n";
  // gzdbg << "moment: " << moment << "\n";
  // gzdbg << "force: " << force << "\n";
  // gzdbg << "torque: " << torque << "\n";
  // gzdbg << "totalTorque: " << totalTorque << "\n";
}

//////////////////////////////////////////////////
void LiftDrag::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void LiftDrag::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDrag::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);

      if ((this->dataPtr->controlJointEntity != kNullEntity) &&
          !_ecm.Component<components::JointPosition>(
              this->dataPtr->controlJointEntity))
      {
        _ecm.CreateComponent(this->dataPtr->controlJointEntity,
            components::JointPosition());
      }
    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(LiftDrag,
                    System,
                    LiftDrag::ISystemConfigure,
                    LiftDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftDrag, "gz::sim::systems::LiftDrag")
