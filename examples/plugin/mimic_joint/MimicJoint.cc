/**
Copyright 2014 Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

/**

Modified by: Adwait Naik on March 31, 2021

**/


///////////////////////////////////////////////////////////////////////////////////

/* VERSION NUMBER */

#define IGNITION_GAZEBO_MAJOR_VERSION "Ignition Gazebo, version 3.8.0\nCopyright (C) 2018 Open Source Robotics Foundation.\n
Released under the Apache 2.0 License.\n\n"

#include <string>
#include <fstream>
#include <queue>
#include <vector>
#include <chrono>

#include "gazebo/physics/physics.hh"

#include <ignition/math/Quaternion.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/plugin/Register.hh>


#include "ignition/gazebo/components/JointPosition.hh"
#include "JointPositionController.hh"


#include "MimicJoint.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::MimicJointPrivate
{
	// update the child link
    private: void UpdateChild(const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm);

    // Joint attributes
    private: std::string joint_name_, mimic_joint_name_, robot_namespace_;
          
    private: double multiplier_, offset_, sensitiveness_, max_effort_;
          
    private: bool has_pid_;

    // PID controller if needed
    private: control_toolbox::Pid pid_;

    // Pointer to joints
    private: ignition::gazebo::Entity joint_, mimic_joint_;

    /// \brief Model interface
    public: Model model{kNullEntity};

    //storing references to the model
    //private: ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
    //private: ignition::gazebo::Entity modelLink{ignition::gazebo::kNullEntity};
    //private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

    // Pointer to world
    private: std::vector<dart::simulation::WorldPtr> world_;

};

//////////////////////////////////////////////////
MimicJoint::MimicJoint() 
   : dataPtr(std::make_unique<MimicJointPrivate>())

{
	joint_.reset();
    mimic_joint_.reset();
} 

//////////////////////////////////////////////////
void MimicJoint::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)

{   
    this->dataPtr->model = Model(_entity);
    world_ = this->dataPtr->model->GetWorld(); 

    // Error message if the model couldn't be found
    if (!model) {
        ROS_ERROR("Parent model is NULL! MimicJoint
         could not be loaded.");
        return;
    }


    if (!this->dataPtr->model.Valid(_ecm))
    {
    
    ignerr << "MimicJoint plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
    }

    
	// Check that ROS has been initialized
    if (!ros::isInitialized()) {
        ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;

    }

    // Check for robot namespace

    if (_sdf->HasElement("robotNamespace"))
    {
    	this->robotNamespace = transport::TopicUtils::AsValidTopic(
            _sdf->Get<std::string>("robotNamespace"));
        if (this->robotNamespace.empty())
        {
           ignerr << "Robot namespace ["
            << _sdf->Get<std::string>("robotNamespace") <<"] is invalid."
            << std::endl;
           return;
        }
    }
    else
    {
       ignerr << "Please specify a robotNamespace.\n";
       return;
    }

    // Check for joint element

    if (!_sdf->HasElement("joint")) {
        ROS_ERROR("No joint element present. MimicJoint could not be loaded.");
        return;
    }
    

    joint_name_ = _sdf->GetElement("joint")->Get<std::string>();

    // Check for mimic joint element

    if (!_sdf->HasElement("mimicJoint")) {
        ROS_ERROR("No mimicJoint element present. MimicJointPlugin could not be loaded.");
        return;
        
    }

    mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();
   
    // Check if PID controller wanted
        
    this->dataPtr->has_pid_ = _sdf->HasElement("hasPID");

    if (has_pid_) {
        std::string name = _sdf->GetElement("hasPID")->Get<std::string>();
        if (name.empty()) {
            name = "gazebo_ros_control/pid_gains/" + mimic_joint_name_;
        }
        const ros::NodeHandle nh(model_nh, name);
        pid_.init(nh);
    }


    // Check for multiplier element

    this->dataPtr->multiplier_ = 1.0;

    if (_sdf->HasElement("multiplier"))
        multiplier_ = _sdf->GetElement("multiplier")->Get<double>();



    // Check for offset element

    this->dataPtr->offset_ = 0.0;

    if (_sdf->HasElement("offset"))
        offset_ = _sdf->GetElement("offset")->Get<double>();

    // Check for sensitiveness element

    this->dataPtr->sensitiveness_ = 0.0;
    
    if (_sdf->zasElement("sensitiveness"))
        sensitiveness_ = _sdf->GetElement("sensitiveness")->Get<double>();

    // Get pointers to joints
    
    joint_ = model_->GetJoint(joint_name_);
    if (!joint_) {
        ROS_ERROR_STREAM("No joint named \"" << joint_name_ << "\". MimicJoint  could not be loaded.");
        return;
        
    }
        
    mimic_joint_ = model_->GetJoint(mimic_joint_name_);
    if (!mimic_joint_) {
        ROS_ERROR_STREAM("No (mimic) joint named \"" << mimic_joint_name_ << "\". MimicJoint could not be loaded.");
        return;
    }

    
    // Check for max effort

#if IGNITION_GAZEBO_MAJOR_VERSION > 2

        max_effort_ = mimic_joint_->GetEffortLimit(0);
#else
        max_effort_ = mimic_joint_->GetMaxForce(0);
#endif
        
    if (_sdf->HasElement("maxEffort")) {
        max_effort_ = _sdf->GetElement("maxEffort")->Get<double>();
    }


    // Set max effort
    if (!has_pid_) {

#if IGNITION_GAZEBO_MAJOR_VERSION > 2

        mimic_joint_->SetParam("fmax", 0, max_effort_);
#else
        mimic_joint_->SetMaxForce(0, max_effort_);
#endif
        
    }
    
    // Output some confirmation
    ROS_INFO_STREAM("MimicJoint loaded! Joint: \"" << joint_name_ << "\", Mimic joint: \"" << mimic_joint_name_ << "\""
                                                             << ", Multiplier: " << multiplier_ << ", Offset: " << offset_
                                                             << ", MaxEffort: " << max_effort_ << ", Sensitiveness: " << sensitiveness_);

    
}

//////////////////////////////////////////////////
void MimicJoint::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MimicJoint::PreUpdate");


  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }
  // Nothing left to do if paused.
  if (_info.paused)
    return;

}

//////////////////////////////////////////////////
void MimicJoint::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)

{
  IGN_PROFILE("MimicJoint::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateChild(_info, _ecm)

}

//////////////////////////////////////////////////
void MimicJointPrivate::UpdateChild(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MimicJoint::UpdateChild");


#if IGNITION_GAZEBO_MAJOR_VERSION >= 3.80
        static ros::Duration period(world_->Physics()->GetMaxStepSize());
#else
        static ros::Duration period(world_->GetPhysicsEngine()->GetMaxStepSize());
#endif


        // Set mimic joint's angle based on joint's angle
#if IGNITION_GAZEBO_MAJOR_VERSION >= 3.80

        double angle = joint_->Position(0) * multiplier_ + offset_;
        double a = mimic_joint_->Position(0);
#else
        double angle = joint_->GetAngle(0).Radian() * multiplier_ + offset_;
        double a = mimic_joint_->GetAngle(0).Radian();
#endif

        if (std::fabs(angle - a) >= sensitiveness_) {

        	if (has_pid_) {

        		a = angle;
            double error = angle - a;
            double effort = math::clamp(pid_.computeCommand(error, period), -max_effort_, max_effort_);
            mimic_joint_->SetForce(0, effort);
        	}
        }

        else{

#if IGNITION_GAZEBO_MAJOR_VERSION >= 4
        	mimic_joint_->SetPosition(0, angle, true);

#elif IGNITION_GAZEBO_MAJOR_VERSION > 3.80

        	ROS_WARN_ONCE("The mimic_joint plugin is using the Joint::SetPosition method without preserving the link velocity.");
            ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
            ROS_WARN_ONCE("Please set gazebo_pid parameters or upgrade to Gazebo 9.");
            ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
            mimic_joint_->SetPosition(0, angle);
#else
            mimic_joint_->SetAngle(0, math::Angle(angle));
#endif
            }
        }
    
/// \Register the plugin
IGNITION_ADD_PLUGIN(MimicJoint,
                    ignition::gazebo::System,
                    MimicJoint::ISystemConfigure,
                    MimicJoint::ISystemPreUpdate,
                    MimicJoint::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MimicJoint, "ignition::gazebo::systems::MimicJoint")
