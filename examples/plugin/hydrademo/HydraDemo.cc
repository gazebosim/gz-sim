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


  /// \brief linear velocity for the model
  public: math::Vector3d linearVel{0.5, 0.3, 0.5};

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

  //double joyX = this->dataPtr->hydraMsgPtr.right().joy_x();
  //double joyY = this->dataPtr->hydraMsgPtr.right().joy_y();

  msgs::Twist msg;

  msgs::Set(msg.mutable_linear(), this->dataPtr->linearVel);

  this->dataPtr->Reset();

}
IGNITION_ADD_PLUGIN(HydraDemoPlugin,

                    HydraDemoPlugin::ISystemConfigure,

                    HydraDemoPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(HydraDemoPlugin,"ignition::gazebo::systems::HydraDemoPlugin")