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

#include <mutex>
#include <string>

#include <ignition/transport/Node.hh>
#include <ignition/transport/TransportTypes.hh>

#include <ignition/gazebo/Model.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/msgs.hh>
#include <ignition/msgs/hydra.pb.h>

#include <ignition/math/Vector3.hh>

#include "HydraDemo.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

#define HYDRA_RIGHT_BUMPER 7
#define HYDRA_RIGHT_1 8
#define HYDRA_RIGHT_2 9
#define HYDRA_RIGHT_3 10
#define HYDRA_RIGHT_4 11
#define HYDRA_RIGHT_CENTER 12
#define HYDRA_RIGHT_JOY 13

#define HYDRA_LEFT_LB 0
#define HYDRA_LEFT_1 1
#define HYDRA_LEFT_2 2
#define HYDRA_LEFT_3 3
#define HYDRA_LEFT_4 4
#define HYDRA_LEFT_CENTER 5
#define HYDRA_LEFT_JOY 6

class ignition::gazebo::systems::HydraDemoPluginPrivate
{

	/// \brief Callback executed every time a new hydra message is received.
  /// \param[in] _msg The hydra message
  public: void OnHydra(const ignition::msgs::Hydra &_msg);


  /// \brief model interface
  public: Model model{kNullEntity};

   
  /// \brief Node used for using ignition gazebo communications.
  public: transport::Node node;


  /// \brief Mutex to protect hydraMsgPtr.
  public: std::mutex msgMutex;


  /// \brief Store the last message from hydra.
  public: msgs::Hydra hydraMsgPtr;


  /// \brief Resets all of the relevant data for this plugin.  Called when
  /// the user clicks the reset button and when the user starts a new
  /// measurement.
  public: void Reset();
  

  /// \publisher node
  public: transport::Node::Publisher pub;


  /// \brief Analog joysticks
  public: float analog[6];

  /// \brief Buttons that have been pressed.
  public: uint8_t buttons[14];

  /// \brief Left and right controller positions.
  public: ignition::math::Vector3d pos[2];

  /// \brief Left and right controller orientations.
  public: ignition::math::Quaterniond quat[2];

};
//////////////////////////////////////////////////////////////////

HydraDemoPlugin::HydraDemoPlugin()
  :dataPtr(std::make_unique<HydraDemoPluginPrivate>())
{
}

/////////////////////////////////////////////////////////////////
HydraDemoPlugin::~HydraDemoPlugin()
{
}
/////////////////////////////////////////////////////////////////

void HydraDemoPluginPrivate::OnHydra(const ignition::msgs::Hydra &_msg)
{

	IGN_PROFILE("HydraDemoPluginPrivate::OnHydra");

	std::lock_guard<std::mutex> lock(this->msgMutex);

  	this->hydraMsgPtr = _msg;

}
////////////////////////////////////////////////////////////////

void HydraDemoPluginPrivate::Reset()
{
  IGN_PROFILE("HydraDemoPluginPrivate::Reset");

  this->hydraMsgPtr;
}
/////////////////////////////////////////////////////////////////

void HydraDemoPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
    this->dataPtr->model = Model(_entity);

  	if (!this->dataPtr->model.Valid(_ecm))
  	{
    	ignerr << "HydraDemo plugin should be attached to a model entity. "
           	   << "Failed to initialize." << std::endl;
    	return;

  	}

    std::string topic = transport::TopicUtils::AsValidTopic("~/hydra");

    if (topic.empty())
    {
      ignerr << "Failed to create topic" << std::endl;
     
      return;
    }

    this->dataPtr->node.Subscribe(topic, &HydraDemoPluginPrivate::OnHydra, this->dataPtr.get());
}
/////////////////////////////////////////////////////////////////

void HydraDemoPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{

	IGN_PROFILE("HydraDemoPlugin::PreUpdate");

	std::lock_guard<std::mutex> lock(this->dataPtr->msgMutex);

	/// \This part of the code is borrowed from the original HydraPlugin

	ignition::math::Pose3d origRight(this->dataPtr->pos[1], this->dataPtr->quat[1]);

  	ignition::math::Pose3d pivotRight = origRight;
  	ignition::math::Pose3d grabRight = origRight;

  	pivotRight.Pos() +=
      origRight.Rot() * ignition::math::Vector3d(-0.04, 0, 0);
  	grabRight.Pos() +=
      origRight.Rot() * ignition::math::Vector3d(-0.12, 0, 0);

  	ignition::math::Pose3d origLeft(this->dataPtr->pos[0], this->dataPtr->quat[0]);

  	ignition::math::Pose3d pivotLeft = origLeft;
  	ignition::math::Pose3d grabLeft = origLeft;

  	pivotLeft.Pos() +=
      origLeft.Rot().RotateVector(ignition::math::Vector3d(-0.04, 0, 0));
  	grabLeft.Pos() +=
      origLeft.Rot().RotateVector(ignition::math::Vector3d(-0.12, 0, 0));

	msgs::Hydra msg;
	msgs::Hydra::Paddle *right_paddle = msg.mutable_right();   
	msgs::Hydra::Paddle *left_paddle = msg.mutable_left();

	right_paddle->set_joy_y(this->dataPtr->analog[3]);
	right_paddle->set_joy_x(this->dataPtr->analog[4]);
	right_paddle->set_trigger(this->dataPtr->analog[5]);

	left_paddle->set_joy_y(this->dataPtr->analog[0]);
	left_paddle->set_joy_x(this->dataPtr->analog[1]);
	left_paddle->set_trigger(this->dataPtr->analog[2]);

	left_paddle->set_button_bumper(this->dataPtr->buttons[0]);
	left_paddle->set_button_1(this->dataPtr->buttons[1]);
	left_paddle->set_button_2(this->dataPtr->buttons[2]);
	left_paddle->set_button_3(this->dataPtr->buttons[3]);
	left_paddle->set_button_4(this->dataPtr->buttons[4]);

	left_paddle->set_button_center(this->dataPtr->buttons[5]);
	left_paddle->set_button_joy(this->dataPtr->buttons[6]);

	right_paddle->set_button_bumper(this->dataPtr->buttons[7]);
	right_paddle->set_button_1(this->dataPtr->buttons[8]);
	right_paddle->set_button_2(this->dataPtr->buttons[9]);
	right_paddle->set_button_3(this->dataPtr->buttons[10]);
	right_paddle->set_button_4(this->dataPtr->buttons[11]);
	right_paddle->set_button_center(this->dataPtr->buttons[12]);
	right_paddle->set_button_joy(this->dataPtr->buttons[13]);

	msgs::Set(right_paddle->mutable_pose(), grabRight);
	msgs::Set(left_paddle->mutable_pose(), grabLeft);

	this->dataPtr->pub.Publish(msg);
  
    this->dataPtr->Reset();

}
IGNITION_ADD_PLUGIN(HydraDemoPlugin,

                    HydraDemoPlugin::ISystemConfigure,

                    HydraDemoPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(HydraDemoPlugin,"ignition::gazebo::systems::HydraDemoPlugin")