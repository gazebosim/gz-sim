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

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/Model.hh"

#include "RotorsMotorModel.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RotorsMotorModelPrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnCmdForce(const ignition::msgs::Double &_msg);

  /// \brief Apply link forces and moments based on propeller state.
  public: void UpdateForcesAndMoments();

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};
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
  auto sdfElem = sdfClone->GetElement("joint_name");
  if (sdfElem)
  {
    this->dataPtr->jointName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->jointName == "")
  {
    ignerr << "RotorsMotorModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

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
  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

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
void RotorsMotorModelPrivate::UpdateForcesAndMoments() {
  switch (motor_type_) {
    case (MotorType::kPosition): {
      double err = joint_->GetAngle(0).Radian() - ref_motor_input_;
      double force = pids_.Update(err, sampling_time_);
      joint_->SetForce(0, force);
      break;
    }
    case (MotorType::kForce): {
      joint_->SetForce(0, ref_motor_input_);
      break;
    }
    default:  // MotorType::kVelocity
    {
      motor_rot_vel_ = joint_->GetVelocity(0);
      if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
        gzerr << "Aliasing on motor [" << motor_number_
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

      // Apply a force to the link.
      link_->AddRelativeForce(math::Vector3(0, 0, thrust));

      // Forces from Philppe Martin's and Erwan Sala<FC>n's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
      math::Vector3 body_velocity_W = link_->GetWorldLinearVel();
      math::Vector3 relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
      math::Vector3 body_velocity_perpendicular =
          relative_wind_velocity_W -
          (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
      math::Vector3 air_drag = -std::abs(real_motor_velocity) *
                               rotor_drag_coefficient_ *
                               body_velocity_perpendicular;

      // Apply air_drag to link.
      link_->AddForce(air_drag);
      // Moments get the parent link, such that the resulting torques can be
      // applied.
      physics::Link_V parent_links = link_->GetParentJointsLinks();
      // The tansformation from the parent_link to the link_.
      math::Pose pose_difference =
          link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();
      math::Vector3 drag_torque(
          0, 0, -turning_direction_ * thrust * moment_constant_);
      // Transforming the drag torque into the parent frame to handle
      // arbitrary rotor orientations.
      math::Vector3 drag_torque_parent_frame =
          pose_difference.rot.RotateVector(drag_torque);
      parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

      math::Vector3 rolling_moment;
      // - \omega * \mu_1 * V_A^{\perp}
      rolling_moment = -std::abs(real_motor_velocity) *
                       rolling_moment_coefficient_ *
                       body_velocity_perpendicular;
      parent_links.at(0)->AddTorque(rolling_moment);
      // Apply the filter on the motor's velocity.
      double ref_motor_rot_vel;
      ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(
          ref_motor_input_, sampling_time_);

      // Make sure max force is set, as it may be reset to 0 by a world reset any
      // time. (This cannot be done during Reset() because the change will be undone
      // by the Joint's reset function afterwards.)
      #if GAZEBO_MAJOR_VERSION < 5
            joint_->SetMaxForce(0, max_force_);
      #endif
            joint_->SetVelocity(
                0, turning_direction_ * ref_motor_rot_vel /
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
