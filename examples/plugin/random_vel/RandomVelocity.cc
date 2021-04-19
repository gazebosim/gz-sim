/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "RandomVelocity.hh"

#include <chrono>
#include <string>
#include <vector>


#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <ignition/gazebo/Link.hh>


#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/math/Rand.hh>

#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include "ignition/gazebo/components/LinearVelocity.hh"


using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RandomVelocityPluginPrivate
{
	  
    /// \brief Velocity scaling factor.
    public: double velocityFactor = 1.0;


    /// \brief Time between recomputing a new velocity vector
    public: std::chrono::steady_clock::duration lastUpdatePeriod;


    /// \brief Time the of the last update.
    public: std::chrono::steady_clock::duration prevUpdate;


    /// \brief Time reset value
    public: std::chrono::steady_clock::duration resetTime;


    /// \brief Velocity to apply.
    public: ignition::math::Vector3d velocity;


    /// \brief X velocity clamping values
    public: ignition::math::Vector2d xRange{-ignition::math::MAX_D, ignition::math::MAX_D};

    /// \brief Y velocity clamping values
    public: ignition::math::Vector2d yRange{-ignition::math::MAX_D, ignition::math::MAX_D};

    /// \brief Z velocity clamping values
    public: ignition::math::Vector2d zRange{-ignition::math::MAX_D, ignition::math::MAX_D};


    /// \brief link pointer
    public: Entity link;
 
    
    /// \brief reset function for last udpate time
    public: void Reset();

};
///////////////////////////////////////////////////////////////////////

RandomVelocityPlugin::RandomVelocityPlugin()
  : dataPtr(new RandomVelocityPluginPrivate)
{
}

////////////////////////////////////////////////////////////////////////

RandomVelocityPlugin::~RandomVelocityPlugin()
{
}

///////////////////////////////////////////////////////////////////////

void RandomVelocityPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{   
    
	auto model = Model(_entity);


  	// Make sure the link has been specified
  	if (!_sdf->HasElement("link"))
  	{
    	ignerr << "<link> element missing from RandomVelocity plugin. "
      	       << "The plugin will not function.\n";
    	return;
  	}

  	// Get the link;
  	this->dataPtr->link = model.LinkByName(_ecm, "link");

  	if (!this->dataPtr->link)
    {
    	ignerr << "Unable to find link[" << _sdf->Get<std::string>("link") << "] "
      		   << "in model[" << model.Name(_ecm) << "]. The RandomVelocity plugin "
      		   << "will not function.\n";
        return;
    }

    // Get x clamping values
  	if (_sdf->HasElement("min_x"))
    	this->dataPtr->xRange.X(_sdf->Get<double>("min_x"));

  	if (_sdf->HasElement("max_x"))
    	this->dataPtr->xRange.Y(_sdf->Get<double>("max_x"));

      // Make sure min <= max
	  //
	  // Analysis:
	  //     min_x   max_x  | result_X  result_Y
	  //    ----------------------------------
	  // 0:   0       0     |   0         0
	  // 1:   1       0     |   0         1
	  // 2:   0       1     |   0         1
	  //
	  // Line 1 above is the error case, and can be handled different ways:
	  //   a: Throw an exception
	  //   b: Set both min and max values to the same value (either min_x or
	  //   max_x)
	  //   c: Fix it by swapping the values
	  //   d: Do nothing, which would:
	  //        i. First clamp the value to <= max_x
	  //        ii. Second clamp the value to >= min_x
	  //        iii. The result would always be the value of min_x
	  //
	  // We've opted for option c, which seems the most resonable.


    ignition::math::Vector2d tmp = this->dataPtr->xRange;
  	this->dataPtr->xRange.X(std::min(tmp.X(), tmp.Y()));
  	this->dataPtr->xRange.Y(std::max(tmp.X(), tmp.Y()));

  	// Get y clamping values
  	if (_sdf->HasElement("min_y"))
   		this->dataPtr->yRange.X(_sdf->Get<double>("min_y"));
  	if (_sdf->HasElement("max_y"))
    	this->dataPtr->yRange.Y(_sdf->Get<double>("max_y"));


    // Make sure min <= max
  	tmp = this->dataPtr->yRange;
  	this->dataPtr->yRange.X(std::min(tmp.X(), tmp.Y()));
  	this->dataPtr->yRange.Y(std::max(tmp.X(), tmp.Y()));


  	// Get z clamping values
	if (_sdf->HasElement("min_z"))
	    this->dataPtr->zRange.X(_sdf->Get<double>("min_z"));
	if (_sdf->HasElement("max_z"))
	    this->dataPtr->zRange.Y(_sdf->Get<double>("max_z"));

	// Make sure min <= max
	tmp = this->dataPtr->zRange;
	this->dataPtr->zRange.X(std::min(tmp.X(), tmp.Y()));
	this->dataPtr->zRange.Y(std::max(tmp.X(), tmp.Y()));


	// Set the velocity factor
  	if (_sdf->HasElement("velocity_factor"))
    	this->dataPtr->velocityFactor = _sdf->Get<double>("velocity_factor");

  	// Set the update period
  	// update_period requires fix 
  	if (!_sdf->HasElement("update_period"))
  		//the line given below(commented) gave error, so changed
  		//this->dataPtr->lastUpdatePeriod = _sdf->Get<double>("update_period")
    	this->dataPtr->lastUpdatePeriod = std::chrono::steady_clock::duration::zero();

}

//////////////////////////////////////////////////////////////////////////////

void RandomVelocityPluginPrivate::Reset()
{
	IGN_PROFILE("RandomVelocityPluginPrivate::Reset");
	this->prevUpdate = this->resetTime;
}

/////////////////////////////////////////////////////////////////////////////

void RandomVelocityPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)

{
	IGN_PROFILE("RandomVelocityPlugin::PreUpdate");
	IGN_PROFILE_BEGIN("PreUpdate");

	IGN_ASSERT(this->dataPtr->link, "<link> in RandomVelocity plugin is null");

	// Short-circuit in case the link is invalid.
	if (!this->dataPtr->link)
	    return;

	// Change direction when enough time has elapsed
	if (_info.simTime - this->dataPtr->prevUpdate > this->dataPtr->lastUpdatePeriod)
	{

	    // Get a random velocity value.
	    this->dataPtr->velocity.Set(
	        ignition::math::Rand::DblUniform(-1, 1),
	        ignition::math::Rand::DblUniform(-1, 1),
	        ignition::math::Rand::DblUniform(-1, 1));

	    // Apply scaling factor
	    this->dataPtr->velocity.Normalize();
	    this->dataPtr->velocity = this->dataPtr->velocityFactor;

	    // Clamp X value
	    this->dataPtr->velocity.X(ignition::math::clamp(this->dataPtr->velocity.X(),
	    this->dataPtr->xRange.X(), this->dataPtr->xRange.Y()));

	    // Clamp Y value
	    this->dataPtr->velocity.Y(ignition::math::clamp(this->dataPtr->velocity.Y(),
	    this->dataPtr->yRange.X(), this->dataPtr->yRange.Y()));

	    // Clamp Z value
	    this->dataPtr->velocity.Z(ignition::math::clamp(this->dataPtr->velocity.Z(),
	    this->dataPtr->zRange.X(), this->dataPtr->zRange.Y()));

	    this->dataPtr->prevUpdate = _info.simTime;
	}

	  // Apply velocity
	  Link link_(this->dataPtr->link);
	  link_.WorldLinearVelocity(_ecm);
	  IGN_PROFILE_END();

}
//////////////////////////////////////////////////////////////////////////////

IGNITION_ADD_PLUGIN(RandomVelocityPlugin,
                    ignition::gazebo::System,
                    RandomVelocityPlugin::ISystemConfigure,
                    RandomVelocityPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RandomVelocityPlugin, "ignition::gazebo::systems::RandomVelocityPlugin")