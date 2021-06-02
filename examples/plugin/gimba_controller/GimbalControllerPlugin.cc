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
#include <vector>
#include <mutex>
#include <chrono>

#include <ignition/transport/Node.hh>

#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/PID.hh>

#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include "GimbalControllerPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::GimbalControllerPluginPrivate
{    
  /// \brief function for Pitch joint message
  public: void OnPitchStringMsg(const msgs::StringMsg &_msg);

  /// \brief function for Roll joint message
	public: void OnRollStringMsg(const msgs::StringMsg &_msg);

  /// \brief function for Yaw joint message
	public: void OnYawStringMsg(const msgs::StringMsg &_msg);

  /// \brief returns _angle1 normalized about
  /// (_reference - IGN_PI, _reference + IGN_PI]
  /// \param[in] _angle1 input angle
  /// \param[in] _reference reference input angle for normalization
  /// \return normalized _angle1 about _reference
  public: double NormalizeAbout(double _angle, double _reference);

  /// \brief returns shortest angular distance from _from to _to
  /// \param[in] _from starting anglular position
  /// \param[in] _to end angular position
  /// \return distance traveled from starting to end angular positions
  public: double ShortestAngularDistance(double _from, double _to);

  /// \brief function for 3-Axis rotation
  public: ignition::math::Vector3d ThreeAxisRot(
      double r11, double r12, double r21, double r31, double r32);

  /// \brief function for conversion of Quaternion to ZYX matrix
  public: ignition::math::Vector3d QtoZXY(
      const ignition::math::Quaterniond &_q);

  /// \brief function to initialize parameters
  public: void Init(const EntityComponentManager &_ecm);

  /// \brief callback for imu sensor
  public: void imuCb(const ignition::msgs::IMU &_msg);

  /// \brief publisher node for pitch
  public: transport::Node::Publisher pitchPub;

  /// \brief publisher node for roll
  public: transport::Node::Publisher rollPub;

  /// \breif publisher node for yaw
  public: transport::Node::Publisher yawPub;

  /// \brief interface to the Model
  public: Model model{kNullEntity};

  /// \brief yaw joint entity
  public: Entity yawJoint;

  /// \brief pitch joint entity
  public: Entity pitchJoint;

  /// \brief roll joint entity
  public: Entity rollJoint;

  /// \brief status 
  public: std::string status;

  /// IMU sensor
  public: 
    std::string imuName;
    bool imuInitialized;
    bool imuMsgValid;
    std::mutex imuMsgMutex;

  /// \brief model link entity
  public: Entity modelLink{ignition::gazebo::kNullEntity};

  /// \brief commands for pitch, yaw, and roll
  public: double pitchCommand;
  public: double yawCommand;
  public: double rollCommand;

  /// \brief node for communication
  public: transport::Node node;

  /// \brief Update time for the controller
  public: math::clock::time_point lastUpdateTime;

  /// \brief last update time
  public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief PID
  public: ignition::math::PID pitchPid;
  public: ignition::math::PID yawPid;
  public: ignition::math::PID rollPid;
};

////////////////////////////////////////////////////////////////////////
GimbalControllerPlugin::GimbalControllerPlugin()
  :dataPtr(std::make_unique<GimbalControllerPluginPrivate>())
{
  /// TODO: make these gains part of sdf xml
  this->dataPtr->pitchPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->dataPtr->rollPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->dataPtr->yawPid.Init(1.0, 0, 0, 0, 0, 1.0, -1.0);
  this->dataPtr->pitchCommand = 0.5* IGN_PI;
  this->dataPtr->rollCommand = 0;
  this->dataPtr->yawCommand = 0;
}

////////////////////////////////////////////////////////////////////////////
void GimbalControllerPluginPrivate::imuCb(const ignition::msgs::IMU &_msg)
{
  std::lock_guard<std::mutex> lock(this->imuMsgMutex);
  ignition::msgs::IMU imuMsg = _msg;
  this->imuMsgValid = true;
}

/////////////////////////////////////////////////////////////////////////////
void GimbalControllerPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
	this->dataPtr->model = ignition::gazebo::Model(_entity);

	std::string yawJointName = "cgo3_vertical_arm_joint";
	this->dataPtr->yawJoint = this->dataPtr->model.JointByName(_ecm, "yawJointName");

	if(_sdf->HasElement("joint_yaw"))
	{
		// Add names to map
		yawJointName = _sdf->Get<std::string>("joint_yaw");

		if(this->dataPtr->model.JointByName(_ecm, "yawJointName"))
		{
			this->dataPtr->yawJoint = this->dataPtr->model.JointByName(_ecm, "yawJointName");
		}

		else
		{
			ignwarn << "joint_yaw ["<< yawJointName <<"] does not exist?\n"<< std::endl;
		}
	}

	if(!this->dataPtr->yawJoint)
	{
		ignerr << "GimbalControllerPlugin::Configure ERROR! Can't get yaw joint. " <<yawJointName <<"' " <<std::endl;
	}

	// for roll joint
	std::string rollJointName = "cgo3_horizontal_arm_joint";
	this->dataPtr->rollJoint = this->dataPtr->model.JointByName(_ecm, "rollJointName");

	if(_sdf->HasElement("joint_roll"))
	{
		// Add names to map
		rollJointName = _sdf->Get<std::string>("joint_roll");

		if(this->dataPtr->model.JointByName(_ecm, "rollJointName"))
		{
			this->dataPtr->rollJoint = this->dataPtr->model.JointByName(_ecm, "rollJointName");
		}

		else
		{
			ignwarn << "joint_roll ["<< rollJointName <<"] does not exist?\n"<< std::endl;
		}
	}

	if(!this->dataPtr->rollJoint)
	{
		ignerr << "GimbalControllerPlugin::Configure ERROR! Can't get roll joint. " <<rollJointName <<"' " <<std::endl;
	}

	// for pitch joint

	std::string pitchJointName = "cgo3_camera_joint";
	this->dataPtr->pitchJoint = this->dataPtr->model.JointByName(_ecm, "pitchJointName");

	if(_sdf->HasElement("joint_pitch"))
	{
		// Add names to map
		pitchJointName = _sdf->Get<std::string>("joint_pitch");
		if(this->dataPtr->model.JointByName(_ecm, "pitchJointName"))
		{
			this->dataPtr->pitchJoint = this->dataPtr->model.JointByName(_ecm, "picthJointName");
		}

		else
		{
			ignwarn << "joint_pitch ["<< pitchJointName <<"] does not exist?\n"<< std::endl;
		}
	}

	if(!this->dataPtr->pitchJoint)
	{
		ignerr << "GimbalControllerPlugin::Configure ERROR! Can't get pitch joint. " <<pitchJointName <<"' " <<std::endl;
	}
	// get imu sensor
	this->dataPtr->imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
}

///////////////////////////////////////////////////////////////////////////
void GimbalControllerPluginPrivate::Init(const EntityComponentManager &_ecm)
{    
  // receive pitch command via ignition transport 
	std::string pitchTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_pitch_cmd";
  this->node.Subscribe(pitchTopic, &GimbalControllerPluginPrivate::OnPitchStringMsg, this);

  // receive roll command via gz transport
  std::string rollTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_roll_cmd";
  this->node.Subscribe(rollTopic, &GimbalControllerPluginPrivate::OnRollStringMsg, this);

  // receive yaw command via gz transport
  std::string yawTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_yaw_cmd";
  this->node.Subscribe(yawTopic, &GimbalControllerPluginPrivate::OnYawStringMsg, this);

  // publish pitch status via gz transport
  pitchTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_pitch_status";
  this->pitchPub= this->node.Advertise<ignition::msgs::StringMsg>(pitchTopic);

  // publish roll status via gz transport
  rollTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_roll_status";
  this->rollPub= this->node.Advertise<ignition::msgs::StringMsg>(rollTopic);

  // publish yaw status via gz transport
  yawTopic = std::string("~/") +  this->model.Name(_ecm) + "/gimbal_yaw_status";
  this->yawPub= this->node.Advertise<ignition::msgs::StringMsg>(yawTopic);

  ignmsg<< "GimbalControllerPluginPrivate::Init" <<std::endl;
}

////////////////////////////////////////////////////////////////////////////
void GimbalControllerPluginPrivate::OnPitchStringMsg(const msgs::StringMsg &_msg)
{
	this->pitchCommand = atof(_msg.data().c_str());
}

////////////////////////////////////////////////////////////////////////////
void GimbalControllerPluginPrivate::OnYawStringMsg(const msgs::StringMsg &_msg)
{
	this->yawCommand = atof(_msg.data().c_str());
}

////////////////////////////////////////////////////////////////////////////
void GimbalControllerPluginPrivate::OnRollStringMsg(const msgs::StringMsg &_msg)
{
	this->rollCommand = atof(_msg.data().c_str());
}

////////////////////////////////////////////////////////////////////////////
ignition::math::Vector3d GimbalControllerPluginPrivate::ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::math::Vector3d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

////////////////////////////////////////////////////////////////////////////
ignition::math::Vector3d GimbalControllerPluginPrivate::QtoZXY(
  const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = this->ThreeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}

////////////////////////////////////////////////////////////////////////////
void GimbalControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{  if(!this->dataPtr->imuInitialized)
   {
      // Set unconditionally because we're only going to try this once.
      this->dataPtr->imuInitialized = true;

      auto imuEntity = _ecm.EntityByComponents(
          components::Name(this->dataPtr->imuName),
          components::Imu(),
          components::ParentEntity(this->dataPtr->modelLink));

      //Imu sensor topic 
      std::string topic = scopedName(imuEntity, _ecm) + "/imu";

      if(topic.empty())
      {
        ignerr << "imu_sensor [" << this->dataPtr->imuName
               << "] not found, abort ArduPilot plugin." << "\n";
        return;
      }

      this->dataPtr->node.Subscribe(topic, &GimbalControllerPluginPrivate::imuCb, this->dataPtr.get());
    }
    
    if(!this->dataPtr->pitchJoint || !this->dataPtr->rollJoint || !this->dataPtr->yawJoint)
      return;

    if(_info.simTime < this->dataPtr->lastControllerUpdateTime)
    {    
       ignerr << "time reset event\n";
       this->dataPtr->lastControllerUpdateTime = _info.simTime;
        return;
    }
    else if (_info.simTime > this->dataPtr->lastControllerUpdateTime)
    { 
      // Time delta
      std::chrono::duration<double> dt  = (this->dataPtr->lastControllerUpdateTime - _info.simTime);
  
      // apply forces to move gimbal
      // for pitch 
      auto pitchJointPos = _ecm.Component<components::JointPosition>(this->dataPtr->pitchJoint);
      double pitchError = pitchJointPos->Data().at(0) - this->dataPtr->pitchCommand;
      double pitchForce = this->dataPtr->pitchPid.Update(pitchError, dt);

      ignition::gazebo::components::JointForceCmd* pjfcComp = nullptr;
      pjfcComp = _ecm.Component<components::JointForceCmd>(this->dataPtr->pitchJoint);

      if (pjfcComp == nullptr)
      {
        pjfcComp = _ecm.Component<components::JointForceCmd>(_ecm.CreateComponent(this->dataPtr->pitchJoint, components::JointForceCmd({0})));
      }

      //_ecm.SetComponentData(this->dataPtr->pitchJoint, components::JointForceCmd({pitchForce}));
      // for yaw
      auto yawJointPos = _ecm.Component<components::JointPosition>(this->dataPtr->yawJoint);
      double yawError = yawJointPos->Data().at(0) - this->dataPtr->yawCommand;
      double yawForce = this->dataPtr->yawPid.Update(yawError, dt);

      ignition::gazebo::components::JointForceCmd* yjfcComp = nullptr;
      yjfcComp = _ecm.Component<components::JointForceCmd>(this->dataPtr->yawJoint);

      if (yjfcComp == nullptr)
      {
        yjfcComp = _ecm.Component<components::JointForceCmd>(_ecm.CreateComponent(this->dataPtr->yawJoint, components::JointForceCmd({0})));
      }

      // for roll
      auto rollJointPos = _ecm.Component<components::JointPosition>(this->dataPtr->rollJoint);
      double rollError = rollJointPos->Data().at(0) - this->dataPtr->rollCommand;
      double rollForce = this->dataPtr->rollPid.Update(rollError, dt);

      ignition::gazebo::components::JointForceCmd* rjfcComp = nullptr;
      rjfcComp = _ecm.Component<components::JointForceCmd>(this->dataPtr->rollJoint);

      if (rjfcComp == nullptr)
      {
        rjfcComp = _ecm.Component<components::JointForceCmd>(_ecm.CreateComponent(this->dataPtr->rollJoint, components::JointForceCmd({0})));
      }

      static int i =1000;
      if (++i>100)
      {
        i = 0;
        std::stringstream ss;
        ignition::msgs::StringMsg m;

        ss << pitchJointPos;
        m.set_data(ss.str());
        this->dataPtr->pitchPub.Publish(m);

        ss << rollJointPos;
        m.set_data(ss.str());
        this->dataPtr->rollPub.Publish(m);

        ss << yawJointPos;
        m.set_data(ss.str());
        this->dataPtr->yawPub.Publish(m);
      }
  }
}

////////////////////////////////////////////////////////////////////////////
double GimbalControllerPluginPrivate::NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -IGN_PI)
  {
    diff += 2.0*IGN_PI;
  }
  while (diff > IGN_PI)
  {
    diff -= 2.0*IGN_PI;
  }
  return diff + reference;
}

///////////////////////////////////////////////////////////////////////////
double GimbalControllerPluginPrivate::ShortestAngularDistance(double _from, double _to)
{
  return this->NormalizeAbout(_to, _from) - _from;
}

IGNITION_ADD_PLUGIN(GimbalControllerPlugin,
  ignition::gazebo::System,
  GimbalControllerPlugin::ISystemConfigure,
  GimbalControllerPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(GimbalControllerPlugin,"ignition::gazebo::systems::GimbalControllerPlugin")