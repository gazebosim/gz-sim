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

/*
* Desc: Modified for ignition gazebo
* by Adwait Naik
*/

#ifndef IGNITION_GAZEBO_PLUGINS_RANDOMVELOCITY_PLUGIN_HH_
#define IGNITION_GAZEBO_PLUGINS_RANDOMVELOCITY_PLUGIN_HH_

#include <memory>

#include <ignition/gazebo/System.hh>
#include <sdf/sdf.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

  // Forward declaration

  class RandomVelocityPluginPrivate;

  /// \a brief class implementation 
  class RandomVelocityPlugin:
	    public System,
	    public ISystemConfigure,
	    public ISystemPreUpdate
	    

  {

  	/// \brief constructor
    public: RandomVelocityPlugin();

    /// \brief Destructor
    public: ~RandomVelocityPlugin();


    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm);

    /// \brief Private data pointer
    private: std::unique_ptr<RandomVelocityPluginPrivate> dataPtr;

  };

}
}
}
}
#endif //IGNITION_GAZEBO_PLUGINS_RANDOMVELOCITY_PLUGIN_HH_