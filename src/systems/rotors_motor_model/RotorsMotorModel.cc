/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>

#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"

#include "RotorsMotorModel.hh"

// from rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h
/// \brief      Obtains a parameter from sdf.
/// \param[in]  _sdf           Pointer to the sdf object.
/// \param[in]  _name          Name of the parameter.
/// \param[out] _param         Param Variable to write the parameter to.
/// \param[in]  _default_value Default value, if the parameter not available.
/// \param[in]  _verbose       If true, ignerr if parameter is not available.
template<class T>
bool getSdfParam(
    sdf::ElementPtr _sdf,
    const std::string& _name,
    T& _param,
    const T& _default_value,
    const bool& _verbose = false)
{
  if (_sdf->HasElement(_name)) {
    _param = _sdf->GetElement(_name)->Get<T>();
    return true;
  }
  else {
    _param = _default_value;
    if (_verbose)
      ignerr << "Please specify a value for parameter \"" << _name << "\".\n";
  }
  return false;
}

// from rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h
/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants.
/// \details
///           Short reveiw of discrete time implementation of first order system:
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
template <typename T>
class FirstOrderFilter {

 public:
  FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

  /// \brief    This method will apply a first order filter on the inputState.
  T updateFilter(T inputState, double samplingTime) {

    T outputState;
    if (inputState > previousState_) {
      // Calcuate the outputState if accelerating.
      double alphaUp = exp(-samplingTime / timeConstantUp_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

    }
    else {
      // Calculate the outputState if decelerating.
      double alphaDown = exp(-samplingTime / timeConstantDown_);
      outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
    }
    previousState_ = outputState;
    return outputState;

  }

  ~FirstOrderFilter() {}

 protected:
  double timeConstantUp_;
  double timeConstantDown_;
  T previousState_;
};

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Constants for specifying clockwise (CW) and counter-clockwise (CCW)
/// directions of rotation.
namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction

/// \brief Type of input command to motor.
enum class MotorType {
  kVelocity,
  kPosition,
  kForce
};

class ignition::gazebo::systems::RotorsMotorModelPrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnCmdForce(const ignition::msgs::Double &_msg);

  /// \brief Apply link forces and moments based on propeller state.
  public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Parent link Entity
  public: Entity parentLinkEntity;

  /// \brief Parent link name
  public: std::string parentLinkName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Sampling time (from motor_model.hpp).
  public: double sampling_time_ = 0.01;

  /// \brief Index of motor on multirotor_base.
  public: int motor_number_ = 0;

  /// \brief Turning direction of the motor.
  public: int turning_direction_ = turning_direction::CW;

  /// \brief Type of input command to motor.
  public: MotorType motor_type_ = MotorType::kVelocity;

  /// \brief Moment constant for computing drag torque based on thrust
  /// with units of length (m).
  /// The default value is taken from gazebo_motor_model.h
  double moment_constant_ = 0.016;

  /// \brief Thrust coefficient for propeller with units of N / (rad/s)^2.
  /// The default value is taken from gazebo_motor_model.h
  double motor_constant_ = 8.54858e-06;

  /// \brief Reference input to motor. For MotorType kVelocity, this
  /// is the reference angular velocity in rad/s.
  double ref_motor_input_ = 0.0;

  /// \brief Rolling moment coefficient with units of N*m / (m/s^2).
  /// The default value is taken from gazebo_motor_model.h
  double rolling_moment_coefficient_ = 1.0e-6;

  /// \brief Rotor drag coefficient for propeller with units of N / (m/s^2).
  /// The default value is taken from gazebo_motor_model.h
  double rotor_drag_coefficient_ = 1.0e-4;

  /// \brief Large joint velocities can cause problems with aliasing,
  /// so the joint velocity used by the physics engine is reduced
  /// this factor, while the larger value is used for computing
  /// propeller thrust.
  /// The default value is taken from gazebo_motor_model.h
  double rotor_velocity_slowdown_sim_ = 10.0;

  /// \brief Time constant for rotor deceleration.
  /// The default value is taken from gazebo_motor_model.h
  double time_constant_down_ = 1.0 / 40.0;

  /// \brief Time constant for rotor acceleration.
  /// The default value is taken from gazebo_motor_model.h
  double time_constant_up_ = 1.0 / 80.0;

  /// \brief Filter on rotor velocity that has different time constants
  /// for increasing and decreasing values.
  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
};

//////////////////////////////////////////////////
RotorsMotorModel::RotorsMotorModel()
  : dataPtr(std::make_unique<RotorsMotorModelPrivate>())
{
}

//////////////////////////////////////////////////
void RotorsMotorModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "RotorsMotorModel plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  // Get params from SDF
  if (sdfClone->HasElement("jointName"))
  {
    this->dataPtr->jointName = sdfClone->Get<std::string>("jointName");
  }

  if (this->dataPtr->jointName.empty())
  {
    ignerr << "RotorsMotorModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("linkName"))
  {
    this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");
  }

  if (this->dataPtr->linkName.empty())
  {
    ignerr << "RotorsMotorModel found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("motorNumber"))
    this->dataPtr->motor_number_ =
      sdfClone->GetElement("motorNumber")->Get<int>();
  else
    ignerr << "Please specify a motorNumber.\n";

  if (sdfClone->HasElement("turningDirection")) {
    std::string turning_direction =
        sdfClone->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      this->dataPtr->turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      this->dataPtr->turning_direction_ = turning_direction::CCW;
    else
      ignerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";
  } else
    ignerr << "Please specify a turning direction ('cw' or 'ccw').\n";

  if (sdfClone->HasElement("motorType")) {
    std::string motor_type =
      sdfClone->GetElement("motorType")->Get<std::string>();
    if (motor_type == "velocity")
      this->dataPtr->motor_type_ = MotorType::kVelocity;
    else if (motor_type == "position")
    {
      this->dataPtr->motor_type_ = MotorType::kPosition;
      ignerr << "motorType 'position' not supported" << std::endl;
    }
    else if (motor_type == "force") {
      this->dataPtr->motor_type_ = MotorType::kForce;
      ignerr << "motorType 'force' not supported" << std::endl;
    } else
      ignerr << "Please only use 'velocity', 'position' or "
               "'force' as motorType.\n";
  } else {
    ignwarn << "motorType not specified, using velocity.\n";
    this->dataPtr->motor_type_ = MotorType::kVelocity;
  }

  getSdfParam<double>(
      sdfClone, "rotorDragCoefficient", this->dataPtr->rotor_drag_coefficient_,
      this->dataPtr->rotor_drag_coefficient_);
  getSdfParam<double>(
      sdfClone, "rollingMomentCoefficient",
      this->dataPtr->rolling_moment_coefficient_,
      this->dataPtr->rolling_moment_coefficient_);
  getSdfParam<double>(sdfClone, "motorConstant",
      this->dataPtr->motor_constant_, this->dataPtr->motor_constant_);
  getSdfParam<double>(sdfClone, "momentConstant",
      this->dataPtr->moment_constant_, this->dataPtr->moment_constant_);

  getSdfParam<double>(
      sdfClone, "timeConstantUp",
      this->dataPtr->time_constant_up_, this->dataPtr->time_constant_up_);
  getSdfParam<double>(
      sdfClone, "timeConstantDown",
      this->dataPtr->time_constant_down_, this->dataPtr->time_constant_down_);
  getSdfParam<double>(
      sdfClone, "rotorVelocitySlowdownSim",
      this->dataPtr->rotor_velocity_slowdown_sim_, 10);

  // Create the first order filter.
  this->dataPtr->rotor_velocity_filter_.reset(
      new FirstOrderFilter<double>(
          this->dataPtr->time_constant_up_, this->dataPtr->time_constant_down_,
          this->dataPtr->ref_motor_input_));

  // Subscribe to commands
  std::string topic{"/model/" + this->dataPtr->model.Name(_ecm) + "/joint/" +
                    this->dataPtr->jointName + "/cmd_force"};
  this->dataPtr->node.Subscribe(topic, &RotorsMotorModelPrivate::OnCmdForce,
                                this->dataPtr.get());

  ignmsg << "RotorsMotorModel subscribing to Double messages on [" << topic
         << "]" << std::endl;
}

//////////////////////////////////////////////////
void RotorsMotorModel::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  // If the joint or links haven't been identified yet, look for them
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);

    const auto parentLinkName = _ecm.Component<components::ParentLinkName>(
        this->dataPtr->jointEntity);
    this->dataPtr->parentLinkName = parentLinkName->Data();
  }

  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->parentLinkEntity == kNullEntity)
  {
    this->dataPtr->parentLinkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->parentLinkName);
  }

  if (this->dataPtr->jointEntity == kNullEntity ||
      this->dataPtr->linkEntity == kNullEntity ||
      this->dataPtr->parentLinkEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->sampling_time_ =
    std::chrono::duration<double>(_info.dt).count();

  // Update joint force
  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);

  std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

  if (force == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        components::JointForceCmd({this->dataPtr->jointForceCmd}));
  }
  else
  {
    force->Data()[0] += this->dataPtr->jointForceCmd;
  }
}

//////////////////////////////////////////////////
void RotorsMotorModelPrivate::OnCmdForce(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
  this->jointForceCmd = _msg.data();
}

//////////////////////////////////////////////////
void RotorsMotorModelPrivate::UpdateForcesAndMoments(
    EntityComponentManager &_ecm)
{
  switch (motor_type_) {
    case (MotorType::kPosition): {
      // double err = joint_->GetAngle(0).Radian() - ref_motor_input_;
      // double force = pids_.Update(err, sampling_time_);
      // joint_->SetForce(0, force);
      break;
    }
    case (MotorType::kForce): {
      // joint_->SetForce(0, ref_motor_input_);
      break;
    }
    default:  // MotorType::kVelocity
    {
      const auto jointVelocity = _ecm.Component<components::JointVelocity>(
          this->jointEntity);
      double motor_rot_vel_ = jointVelocity->Data()[0];
      if (motor_rot_vel_ / (2 * IGN_PI) > 1 / (2 * sampling_time_)) {
        ignerr << "Aliasing on motor [" << motor_number_
              << "] might occur. Consider making smaller simulation time "
                 "steps or raising the rotor_velocity_slowdown_sim_ param.\n";
      }
      double real_motor_velocity =
          motor_rot_vel_ * rotor_velocity_slowdown_sim_;
      // Get the direction of the rotor rotation.
      int real_motor_velocity_sign =
          (real_motor_velocity > 0) - (real_motor_velocity < 0);
      // Assuming symmetric propellers (or rotors) for the thrust calculation.
      double thrust = turning_direction_ * real_motor_velocity_sign *
                      real_motor_velocity * real_motor_velocity *
                      motor_constant_;

      using Pose = ignition::math::Pose3d;
      using Vector3 = ignition::math::Vector3d;

      Link link(this->linkEntity);
      const auto worldPose = link.WorldPose(_ecm);
      if (!worldPose)
      {
        ignerr << "link " << this->linkName << " has no WorldPose component"
               << std::endl;
      }

      // Apply a force to the link.
      link.AddWorldForce(_ecm, worldPose->Rot() * Vector3(0, 0, thrust));

      // Forces from Philppe Martin's and Erwan Salaün's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      const auto jointPose = _ecm.Component<components::WorldPose>(
          this->jointEntity);
      const auto jointAxis = _ecm.Component<components::JointAxis>(
          this->jointEntity);
      Vector3 joint_axis = jointPose->Data().Rot() * jointAxis->Data().Xyz();
      const auto worldLinearVel = link.WorldLinearVelocity(_ecm);
      if (!worldLinearVel)
      {
        ignerr << "link " << this->linkName << " has no WorldLinearVelocity "
               << "component" << std::endl;
      }
      Vector3 body_velocity_W = *worldLinearVel;
      Entity windEntity = _ecm.EntityByComponents(components::Wind());
      auto windLinearVel =
          _ecm.Component<components::WorldLinearVelocity>(windEntity);
      Vector3 wind_speed_W = windLinearVel->Data();
      Vector3 relative_wind_velocity_W = body_velocity_W - wind_speed_W;
      Vector3 body_velocity_perpendicular =
          relative_wind_velocity_W -
          (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      Vector3 air_drag = -std::abs(real_motor_velocity) *
                               rotor_drag_coefficient_ *
                               body_velocity_perpendicular;

      // Apply air_drag to link.
      link.AddWorldForce(_ecm, air_drag);
      // Moments get the parent link, such that the resulting torques can be
      // applied.
      msgs::Wrench wrench;
      Vector3 parentWorldTorque;
      auto parentWrenchComp =
        _ecm.Component<components::ExternalWorldWrenchCmd>(
          this->parentLinkEntity);
      // gazebo_motor_model.cpp subtracts the GetWorldCoGPose() of the
      // child link from the parent but only uses the rotation component.
      // Since GetWorldCoGPose() uses the link frame orientation, it
      // is equivalent to use WorldPose().Rot().
      Link parentLink(this->parentLinkEntity);
      const auto parentWorldPose = parentLink.WorldPose(_ecm);
      if (!parentWorldPose)
      {
        ignerr << "link " << this->parentLinkName << " has no WorldPose "
               << "component" << std::endl;
      }
      // The tansformation from the parent_link to the link_.
      // Pose pose_difference =
      //  link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();
      Pose pose_difference = *worldPose - *parentWorldPose;
      Vector3 drag_torque(
          0, 0, -turning_direction_ * thrust * moment_constant_);
      // Transforming the drag torque into the parent frame to handle
      // arbitrary rotor orientations.
      Vector3 drag_torque_parent_frame =
          pose_difference.Rot().RotateVector(drag_torque);
      parentWorldTorque = parentWorldPose->Rot() * drag_torque_parent_frame;

      Vector3 rolling_moment;
      // - \omega * \mu_1 * V_A^{\perp}
      rolling_moment = -std::abs(real_motor_velocity) *
                       rolling_moment_coefficient_ *
                       body_velocity_perpendicular;
      parentWorldTorque += rolling_moment;
      msgs::Set(wrench.mutable_torque(), parentWorldTorque);
      components::ExternalWorldWrenchCmd newWrenchComp(wrench);
      if (parentWrenchComp)
      {
        *parentWrenchComp = newWrenchComp;
      }
      else
      {
        _ecm.CreateComponent(this->parentLinkEntity, newWrenchComp);
      }
      // Apply the filter on the motor's velocity.
      double ref_motor_rot_vel;
      ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(
          ref_motor_input_, sampling_time_);

      const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
          this->jointEntity);
      jointVelCmd->Data()[0] = (turning_direction_ * ref_motor_rot_vel /
                       rotor_velocity_slowdown_sim_);
    }
  }
}

IGNITION_ADD_PLUGIN(RotorsMotorModel,
                    ignition::gazebo::System,
                    RotorsMotorModel::ISystemConfigure,
                    RotorsMotorModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RotorsMotorModel,
                          "ignition::gazebo::systems::RotorsMotorModel")
