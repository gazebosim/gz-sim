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
 * @author: Karthik Srivatsan, Frederik Markus
 * @version: 1.1
 *
 * @brief: this plugin models the lift and drag of an aircraft
 * as a single body, using stability, aerodynamic, and control derivatives.
 * It takes in a specified number of control surfaces and control
 * derivatives are defined/specified with respect to the deflection
 * of individual control surfaces. Coefficients for this plugin can be
 * obtained using Athena Vortex Lattice (AVL) by Mark Drela
 * https://nps.edu/web/adsc/aircraft-aerodynamics2
 * The sign conventions used in this plugin are therefore written
 * in a way to be compatible with AVL.
 * Force equations are computed in the body, while
 * moment equations are computed in the stability frame.
 * Has been adapted for Gazebo (Ignition) using the ECS.
 *
 *
 */

#include "AdvancedLiftDrag.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/physics/Joint.hh>

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

class gz::sim::systems::AdvancedLiftDragPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Initializes lift and drag forces in order to later
  /// update the corresponding components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  /// \brief Compute Control Surface effects
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Comp_Ctrl_surf_eff(EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Lift Coefficient at zero angle of attack
  public: double CL0 = 0.0;

  /// \brief Drag coefficient at zero angle of attack
  public: double CD0 = 0.0;

  /// \brief Pitching moment coefficient at zero angle of attack
  public: double Cem0 = 0.0;

  // Get angle-of-attack (alpha) derivatives
  /// \brief dCL/da (slope of CL-alpha curve)
  public: double CLa = 0.0;

  /// \brief dCy/da (sideforce slope wrt alpha)
  public: double CYa = 0.0;

  /// \brief dCl/da (roll moment slope wrt alpha)
  public: double Cella = 0.0;

  /// \brief dCm/da (pitching moment slope wrt alpha - before stall)
  public: double Cema = 0.0;

  /// \brief dCn/da (yaw moment slope wrt alpha)
  public: double Cena = 0.0;

  // Get sideslip angle (beta) derivatives
  /// \brief dCL/dbeta (lift coefficient slope wrt beta)
  public: double CLb = 0.0;

  /// \brief dCY/dbeta (side force slope wrt beta)
  public: double CYb = 0.0;

  /// \brief dCl/dbeta (roll moment slope wrt beta)
  public: double Cellb = 0.0;

  /// \brief dCm/dbeta (pitching moment slope wrt beta)
  public: double Cemb = 0.0;

  /// \brief dCn/dbeta (yaw moment slope wrt beta)
  public: double Cenb = 0.0;

  /// \brief angle of attack when airfoil stalls
  public: double alphaStall = GZ_PI_2;

  /// \brief The angle of attack
  public: double alpha = 0.0;

  /// \brief The sideslip angle
  public: double beta = 0.0;

  /// \brief Slope of the Cm-alpha curve after stall
  public: double CemaStall = 0.0;

  /// \brief AVL reference point (it replaces the center of pressure
  /// in the original LiftDragPlugin)
  public: gz::math::Vector3d cp = math::Vector3d::Zero;

  // Get the derivatives with respect to non-dimensional rates.
  // In the next few lines, if you see "roll/pitch/yaw rate", remember it is in
  // a non-dimensional form- it is not the actual body rate.
  // Also, keep in mind that this CDp is not parasitic drag: that is CD0.

  /// \brief dCD/dp (drag coefficient slope wrt roll rate)
  public: double CDp = 0.0;

  /// \brief dCY/dp (sideforce slope wrt roll rate)
  public: double CYp = 0.0;

  /// \brief dCL/dp (lift coefficient slope wrt roll rate)
  public: double CLp = 0.0;

  /// \brief dCl/dp (roll moment slope wrt roll rate)
  public: double Cellp = 0.0;

  /// \brief dCm/dp (pitching moment slope wrt roll rate)
  public: double Cemp = 0.0;

  /// \brief dCn/dp (yaw moment slope wrt roll rate)
  public: double Cenp = 0.0;

  /// \brief dCD/dq (drag coefficient slope wrt pitching rate)
  public: double CDq = 0.0;

  /// \brief dCY/dq (side force slope wrt pitching rate)
  public: double CYq = 0.0;

  /// \brief dCL/dq (lift coefficient slope wrt pitching rate)
  public: double CLq = 0.0;

  /// \brief dCl/dq (roll moment slope wrt pitching rate)
  public: double Cellq = 0.0;

  /// \brief dCm/dq (pitching moment slope wrt pitching rate)
  public: double Cemq = 0.0;

  /// \brief dCn/dq (yaw moment slope wrt pitching rate)
  public: double Cenq = 0.0;

  /// \brief dCD/dr (drag coefficient slope wrt yaw rate)
  public: double CDr = 0.0;

  /// \brief dCY/dr (side force slope wrt yaw rate)
  public: double CYr = 0.0;

  /// \brief dCL/dr (lift coefficient slope wrt yaw rate)
  public: double CLr = 0.0;

  /// \brief dCl/dr (roll moment slope wrt yaw rate)
  public: double Cellr = 0.0;

  /// \brief dCm/dr (pitching moment slope wrt yaw rate)
  public: double Cemr = 0.0;

  /// \brief dCn/dr (yaw moment slope wrt yaw rate)
  public: double Cenr = 0.0;

  /// \brief Number of present control surfaces
  public: int num_ctrl_surfaces = 0;

  /// Initialize storage of control surface properties
  /// \brief Joint that the control surface connects to
  public: std::vector<Entity> controlJoints;

  /// \brief Direction the control surface deflects to
  public: std::vector<double> ctrl_surface_direction;

  /// \brief Effect of the control surface's deflection on drag
  public: std::vector<double> CD_ctrl;

  /// \brief Effect of the control surface's deflection on side force
  public: std::vector<double> CY_ctrl;

  /// \brief Effect of the control surface's deflection on lift
  public: std::vector<double> CL_ctrl;

  /// \brief Effect of the control surface's deflection on roll moment
  public: std::vector<double> Cell_ctrl;

  /// \brief Effect of the control surface's deflection on pitching moment
  public: std::vector<double> Cem_ctrl;

  /// \brief Effect of the control surface's deflection on yaw moment
  public: std::vector<double> Cen_ctrl;

  /// \brief Add aspect ratio (should that be computed?)
  public: double AR = 0.0;

  /// \brief Add mean-aerodynamic chord
  public: double mac = 0.0;

  /// \brief Add wing efficiency (Oswald efficiency factor for a 3D wing)
  public: double eff = 0.0;

  /// \brief The sigmoid blending parameter
  public: double M = 15.0;

  /// \brief coefficients for the flat plate drag model
  public: double CD_fp_k1 = -0.224;
  public: double CD_fp_k2 = -0.115;


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
void AdvancedLiftDragPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  if (!this->model.Valid(_ecm))
  {
    gzerr << "Advanced LiftDrag system should be attached to a model entity."
           << "Failed to initialize." << std::endl;
    return;
  }
  this->CL0 = _sdf->Get<double>("CL0", this->CL0).first;
  this->CD0 = _sdf->Get<double>("CD0", this->CD0).first;
  this->Cem0 = _sdf->Get<double>("Cem0", this->Cem0).first;
  this->CLa = _sdf->Get<double>("CLa", this->CLa).first;
  this->CYa = _sdf->Get<double>("CYa", this->CYa).first;
  this->Cella = _sdf->Get<double>("Cella", this->Cella).first;
  this->Cema = _sdf->Get<double>("Cema", this->Cema).first;
  this->Cena = _sdf->Get<double>("Cena", this->Cena).first;
  this->CLb = _sdf->Get<double>("CLb", this->CLb).first;
  this->CYb = _sdf->Get<double>("CYb", this->CYb).first;
  this->Cellb = _sdf->Get<double>("Cellb", this->Cellb).first;
  this->Cemb = _sdf->Get<double>("Cemb", this->Cemb).first;
  this->Cenb = _sdf->Get<double>("Cenb", this->Cenb).first;
  this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
  this->CemaStall = _sdf->Get<double>("Cema_stall", this->CemaStall).first;
  this->cp = _sdf->Get<gz::math::Vector3d>("cp", this->cp).first;
  this->CDp = _sdf->Get<double>("CDp", this->CDp).first;
  this->CYp = _sdf->Get<double>("CYp", this->CYp).first;
  this->CLp = _sdf->Get<double>("CLp", this->CLp).first;
  this->Cellp = _sdf->Get<double>("Cellp", this->Cellp).first;
  this->Cemp = _sdf->Get<double>("Cemp", this->Cemp).first;
  this->Cenp = _sdf->Get<double>("Cenp", this->Cenp).first;

  this->CDq = _sdf->Get<double>("CDq", this->CDq).first;
  this->CYq = _sdf->Get<double>("CYq", this->CYq).first;
  this->CLq = _sdf->Get<double>("CLq", this->CLq).first;
  this->Cellq = _sdf->Get<double>("Cellq", this->Cellq).first;
  this->Cemq = _sdf->Get<double>("Cemq", this->Cemq).first;
  this->Cenq = _sdf->Get<double>("Cenq", this->Cenq).first;
  this->CDr = _sdf->Get<double>("CDr", this->CDr).first;
  this->CYr = _sdf->Get<double>("CYr", this->CYr).first;
  this->CLr = _sdf->Get<double>("CLr", this->CLr).first;
  this->Cellr = _sdf->Get<double>("Cellr", this->Cellr).first;
  this->Cemr = _sdf->Get<double>("Cemr", this->Cemr).first;
  this->Cenr = _sdf->Get<double>("Cenr", this->Cenr).first;
  this->num_ctrl_surfaces = _sdf->Get<int>(
    "num_ctrl_surfaces", this->num_ctrl_surfaces).first;

  /*
  Next, get the properties of each control surface: which joint it connects to,
  which direction it deflects in, and the effect of its deflection on the
  coefficient of drag, side force, lift, roll moment, pitching moment, and yaw moment.
  */

  while( _sdf->HasElement("control_surface") )
  {
    auto curr_ctrl_surface = _sdf->GetElement("control_surface");
    auto ctrl_surface_name = curr_ctrl_surface->GetElement(
      "name")->Get<std::string>();
    auto entities =
        entitiesFromScopedName(ctrl_surface_name, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Joint with name[" << ctrl_surface_name << "] not found. "
             << "The LiftDrag will not generate forces.\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple joint entities with name[" << ctrl_surface_name
              << "] found. Using the first one.\n";
    }

    controlJointEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->controlJointEntity,
                                     components::Joint::typeId))
    {
      this->controlJointEntity = kNullEntity;
      gzerr << "Entity with name[" << ctrl_surface_name
            << "] is not a joint.\n";
      this->validConfig = false;
      return;
    }

    this->controlJoints.push_back(controlJointEntity);
    this->ctrl_surface_direction.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "direction"))->GetValue())->GetAsString()));
    this->CD_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "CD_ctrl"))->GetValue())->GetAsString()));
    this->CY_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "CY_ctrl"))->GetValue())->GetAsString()));
    this->CL_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "CL_ctrl"))->GetValue())->GetAsString()));
    this->Cell_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "Cell_ctrl"))->GetValue())->GetAsString()));
    this->Cem_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "Cem_ctrl"))->GetValue())->GetAsString()));
    this->Cen_ctrl.push_back(
      std::stod(((curr_ctrl_surface->GetElement(
        "Cen_ctrl"))->GetValue())->GetAsString()));
    _sdf->RemoveChild(curr_ctrl_surface);
  }

  this->AR = _sdf->Get<double>("AR", this->AR).first;
  this->mac = _sdf->Get<double>("mac", this->mac).first;
  this->eff = _sdf->Get<double>("eff", this->eff).first;
  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->radialSymmetry = _sdf->Get<bool>("radial_symmetry",
      this->radialSymmetry).first;
  this->area = _sdf->Get<double>("area", this->area).first;

  // blade forward (-drag) direction in link frame
  this->forward =
      _sdf->Get<math::Vector3d>("forward", this->forward).first;
  if(std::fabs(this->forward.Length()) >= 1e-9){
    this->forward.Normalize();
  }

  else
  {
    gzerr << "Forward vector length is zero. This is not valid.\n";
    this->validConfig = false;
    return;
  }

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
             << "The AdvancedLiftDrag will not generate forces.\n";
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
      gzerr << "Entity with name[" << linkName << "] is not a link.\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "AdvancedLiftDrag system requires the 'link_name' parameter.\n";
    this->validConfig = false;
    return;
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
AdvancedLiftDrag::AdvancedLiftDrag()
    : System(), dataPtr(std::make_unique<AdvancedLiftDragPrivate>())
{
}

//////////////////////////////////////////////////
void AdvancedLiftDragPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("AdvancedLiftDragPrivate::Update");
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

  std::vector<components::JointPosition*> controlJointPosition_vec(
    this->num_ctrl_surfaces);
  // Generate a new vector that contains the current positions for all joints
  for(int i = 0; i < this->num_ctrl_surfaces; i++){
    components::JointPosition *controlJointPosition = nullptr;
    if(this->controlJoints[i] != kNullEntity){
      controlJointPosition = _ecm.Component<components::JointPosition>(
        this->controlJoints[i]);
      controlJointPosition_vec[i] = controlJointPosition;
    }
  }

  if (!worldLinVel || !worldAngVel || !worldPose)
    return;

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->cp);
  auto air_velocity = worldLinVel->Data() + worldAngVel->Data().Cross(
    cpWorld);
  if (windLinearVel != nullptr){
    air_velocity = worldLinVel->Data() + worldAngVel->Data().Cross(
    cpWorld) - windLinearVel->Data();
  }

  // Define body frame: X forward, Z downward, Y out the right wing
  gz::math::Vector3d body_x_axis = pose.Rot().RotateVector(this->forward);
  gz::math::Vector3d body_z_axis = -1*(pose.Rot().RotateVector(this->upward));
  gz::math::Vector3d body_y_axis =  body_z_axis.Cross(body_x_axis);

  // Get the in-plane velocity (remove spanwise velocity from the velocity
  // vector air_velocity)
  gz::math::Vector3d velInLDPlane = air_velocity - air_velocity.Dot(
    body_y_axis)*body_y_axis;

  // Compute dynamic pressure
  const double speedInLDPlane = velInLDPlane.Length();

  // Define stability frame: X is in-plane velocity, Y is the same as body Y,
  // and Z perpendicular to both
  if(speedInLDPlane <= 1e-9){
    return;
  }
  gz::math::Vector3d stability_x_axis = velInLDPlane/speedInLDPlane;
  gz::math::Vector3d stability_y_axis = body_y_axis;
  gz::math::Vector3d stability_z_axis = stability_x_axis.Cross(
    stability_y_axis);

  double span = std::sqrt(this->area*this->AR);
  double epsilon = 1e-9;
  if (fabs(this->mac - 0.0) <= epsilon){
    // Computing the mean aerodynamic chord involves integrating the square of
    // the chord along the span. If this parameter has not been input, this
    // plugin will approximate MAC as mean chord. This works for rectangular
    // and trapezoidal wings, but for more complex wing shapes, doing the
    // integral is preferred.
    this->mac = this->area/span;
  }

  // Get non-dimensional body rates. Gazebo uses ENU, so some have to be flipped
  // gz::math::Vector3d body_rates = this->link->GetRelativeAngularVel();
  components::AngularVelocity *AngVel = nullptr;
  if (this->linkEntity != kNullEntity)
  {
    AngVel = _ecm.Component<components::AngularVelocity>(this->linkEntity);
  }
  if(AngVel == nullptr){
    gzerr << "Angular Velocity cannot be null.\n";
    this->validConfig = false;
    return;
  }

  double rr = AngVel->Data()[0];  // Roll rate
  double pr = -1*AngVel->Data()[1];  // Pitch rate
  double yr = -1*AngVel->Data()[2];  // Yaw rate

  // Compute angle of attack, alpha, using the stability and body axes
  // Project stability x onto body x and z, then take arctan to find alpha
  double stabx_proj_bodyx = stability_x_axis.Dot(body_x_axis);
  double stabx_proj_bodyz = stability_x_axis.Dot(body_z_axis);
  this->alpha = atan2(stabx_proj_bodyz, stabx_proj_bodyx);

  double sinAlpha = sin(this->alpha);
  double cosAlpha = cos(this->alpha);

  // Compute sideslip angle, beta
  double velSW = air_velocity.Dot(body_y_axis);
  double velFW = air_velocity.Dot(body_x_axis);
  this->beta = (atan2(velSW, velFW));

  // Compute dynamic pressure
  double dyn_pres = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;
  double half_rho_vel = 0.5 * this->rho * speedInLDPlane;

  // Compute CL at cp, check for stall
  double CL{0.0};

   // Use a sigmoid function to blend pre- and post-stall models
  double sigma = (1+exp(-1*this->M*(this->alpha-this->alphaStall))+exp(
    this->M*(this->alpha+this->alphaStall)))/((1+exp(
      -1*this->M*(this->alpha-this->alphaStall)))*(1+exp(
        this->M*(this->alpha+this->alphaStall))));  // blending function

  // The lift coefficient (as well as all the other coefficients) are
  // defined with the coefficient build-up method and using a quasi-steady
  // approach. The first thing that is calculated is the CL of the wing in
  // steady conditions (normal CL-alpha curve). Secondly, the effect of the
  // roll, pitch, and yaw is added through the AVL stability coefficients.
  // Finally, the effect of the control surfaces is added.

  // The lift coefficient of the wing is defined as a combination of a linear
  // function for the pre-stall regime and a combination of exponents a
  // trigonometric functions for the post-stall regime. Both functions are
  // blended in with the sigmoid function.
  // CL_prestall = this->CL0 + this->CLa*ths->alpha
  // CL_poststall = 2*(this->alpha/abs(this->alpha))*pow(sinAlpha,2.0)*cosAlpha

  CL = (1-sigma)*(this->CL0 + this->CLa*this->alpha) + sigma*(
    2*(this->alpha/abs(this->alpha))*sinAlpha*sinAlpha*cosAlpha);

  // Add sideslip effect, if any
  CL = CL + this->CLb*this->beta;

  // Compute control surface effects
  double CL_ctrl_tot = 0;
  double CD_ctrl_tot = 0;
  double CY_ctrl_tot = 0;
  double Cell_ctrl_tot = 0;
  double Cem_ctrl_tot = 0;
  double Cen_ctrl_tot = 0;

  for(int i = 0; i < this->num_ctrl_surfaces; i++){
    double controlAngle = 0.0;
    if (controlJointPosition_vec[i] && !controlJointPosition_vec[
      i]->Data().empty())
    {
      components::JointPosition *tmp_controlJointPosition =
      controlJointPosition_vec[i];
      controlAngle = tmp_controlJointPosition->Data()[0] * 180 / GZ_PI;
    }

    // AVL's and Gazebo's direction of "positive" deflection may be different.
    // Future users, keep an eye on this.
    CL_ctrl_tot += controlAngle*this->CL_ctrl[i]*
    this->ctrl_surface_direction[i];
    CD_ctrl_tot += controlAngle*this->CD_ctrl[i]*
    this->ctrl_surface_direction[i];
    CY_ctrl_tot += controlAngle*this->CY_ctrl[i]*
    this->ctrl_surface_direction[i];
    Cell_ctrl_tot += controlAngle*this->Cell_ctrl[i]*
    this->ctrl_surface_direction[i];
    Cem_ctrl_tot += controlAngle*this->Cem_ctrl[i]*
    this->ctrl_surface_direction[i];
    Cen_ctrl_tot += controlAngle*this->Cen_ctrl[i]*
    this->ctrl_surface_direction[i];
  }

  // AVL outputs a "CZ_elev", but the Z axis is down. This plugin
  // uses CL_elev, which is the negative of CZ_elev
  CL = CL+CL_ctrl_tot;

  // Compute lift force at cp
  gz::math::Vector3d lift = (CL * dyn_pres + (this->CLp * (
    rr*span/2) * half_rho_vel) + (this->CLq * (pr*this->mac/2) *
    half_rho_vel) + (this->CLr * (yr*span/2) * half_rho_vel)) *
    (this->area * (-1 * stability_z_axis));

  // Compute CD at cp, check for stall
  double CD{0.0};

  // Add in quadratic model and a high-angle-of-attack (Newtonian) model
  // The post stall model used below has two parts. The first part, the
  // (estimated) flat plate drag, comes from the data in Ostowari and Naik,
  // Post-Stall Wind Tunnel Data for NACA 44XX Series Airfoil Sections.
  // https://www.nrel.gov/docs/legosti/old/2559.pdf
  // The second part (0.5-0.5cos(2*alpha)) is fitted using data from
  // Stringer et al,
  // A new 360° airfoil model for predicting airfoil thrust potential in
  // vertical-axis wind turbine designs.
  // https://aip.scitation.org/doi/pdf/10.1063/1.5011207
  // I halved the drag numbers to make sure it would work with my
  // flat plate drag model.


  // To estimate the flat plate coefficient of drag, I fit a sigmoid function
  // to the data in Ostowari and Naik. The form I used was:
  // CD_FP = 2/(1+exp(k1+k2*AR)).
  // The coefficients k1 and k2 might need some tuning.
  // I chose a sigmoid because the CD would initially increase quickly
  // with aspect ratio, but that rate of increase would slow down as AR
  // goes to infinity.

  double CD_fp = 2 / (1 + exp(this->CD_fp_k1 + this->CD_fp_k2 * (
    std::max(this->AR, 1 / this->AR))));
  CD = (1 - sigma) * (this->CD0 + (CL*CL) / (GZ_PI * this->AR *
    this->eff)) + sigma * abs(
      CD_fp * (0.5 - 0.5 * cos(2 * this->alpha)));

  // Add in control surface terms
  CD = CD + CD_ctrl_tot;

  // Place drag at cp
  gz::math::Vector3d drag = (CD * dyn_pres + (this->CDp * (rr*span/2) *
    half_rho_vel) + (
      this->CDq * (pr*this->mac/2) * half_rho_vel) +
        (this->CDr * (yr*span/2) * half_rho_vel)) * (this->area *
          (-1*stability_x_axis));

  // Compute sideforce coefficient, CY
  // Start with angle of attack, sideslip, and control terms
  double CY = this->CYa * this->alpha + this->CYb * this->beta + CY_ctrl_tot;

  gz::math::Vector3d sideforce = (CY * dyn_pres + (this->CYp * (
    rr*span/2) * half_rho_vel) + (this->CYq * (pr*this->mac/2) *
    half_rho_vel) + (this->CYr * (yr*span/2) * half_rho_vel)) *
    (this->area * stability_y_axis);

  // The Cm is divided in three sections: alpha>stall angle, alpha<-stall
  // angle-stall angle<alpha<stall angle. The Cm is assumed to be linear in the
  // region between -stall angle and stall angle, with the slope given by
  // dCm/da. Once we get into the stall regions, the Cm is still linear, but
  // the slope changes to dCm_stall/da after stall (also provided as an input).
  // In the alpha>stall angle region the Cm is assumed to always be positive or
  // zero, in the alpha<-stall angle the Cm is assumed to always be
  // negative or zero.

  double Cem{0.0};

  if (alpha > this->alphaStall)
  {
    Cem = this->Cem0 + (this->Cema * this->alphaStall +
          this->CemaStall * (this->alpha - this->alphaStall));
  }
  else if (alpha < -this->alphaStall)
  {
    Cem = this->Cem0 + (-this->Cema * this->alphaStall +
          this->CemaStall * (this->alpha + this->alphaStall));
  }
  else
  {
    Cem = this->Cem0 + this->Cema * this->alpha;
  }
  // Add sideslip effect, if any
  Cem = this->Cemb * this->beta;

  Cem += Cem_ctrl_tot;

  gz::math::Vector3d pm = ((Cem * dyn_pres) + (this->Cemp * (
    rr*span/2) * half_rho_vel) + (this->Cemq * (pr*this->mac/2) *
      half_rho_vel) + (this->Cemr * (yr*span/2) * half_rho_vel)) *
        (this->area * this->mac * body_y_axis);

  // Compute roll moment coefficient, Cell
  // Start with angle of attack, sideslip, and control terms
  double Cell = this-> Cella * this->alpha + this->Cellb *
    this-> beta + Cell_ctrl_tot;
  gz::math::Vector3d rm = ((Cell * dyn_pres) + (this->Cellp * (
    rr*span/2) * half_rho_vel) + (this->Cellq * (pr*this->mac/2) *
      half_rho_vel) + (this->Cellr * (yr*span/2) * half_rho_vel)) *
        (this->area * span * body_x_axis);

  // Compute yaw moment coefficient, Cen
  // Start with angle of attack, sideslip, and control terms
  double Cen = this->Cena * this->alpha + this->Cenb * this->beta +
    Cen_ctrl_tot;
  gz::math::Vector3d ym = ((Cen * dyn_pres) + (this->Cenp * (
    rr*span/2) * half_rho_vel) + (this->Cenq * (pr*this->mac/2) *
      half_rho_vel) + (this->Cenr * (yr*span/2) * half_rho_vel)) *
        (this->area * span * body_z_axis);

  // Compute moment (torque)
  gz::math::Vector3d moment = pm+rm+ym;

  // Compute force about cg in inertial frame
  gz::math::Vector3d force = lift + drag + sideforce;
  gz::math::Vector3d torque = moment;

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // positions
  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);
}

//////////////////////////////////////////////////
void AdvancedLiftDrag::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void AdvancedLiftDrag::PreUpdate(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  GZ_PROFILE("AdvancedLiftDrag::PreUpdate");

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

      for(int i = 0; i < this->dataPtr->num_ctrl_surfaces; i++){

        if ((this->dataPtr->controlJoints[i]!= kNullEntity) &&
          !_ecm.Component<components::JointPosition>(this->dataPtr->
            controlJoints[i]))
          {
            _ecm.CreateComponent(this->dataPtr->controlJoints[i],
                components::JointPosition());
          }
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

GZ_ADD_PLUGIN(AdvancedLiftDrag,
                    System,
                    AdvancedLiftDrag::ISystemConfigure,
                    AdvancedLiftDrag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AdvancedLiftDrag,
                    "gz::sim::systems::AdvancedLiftDrag")
