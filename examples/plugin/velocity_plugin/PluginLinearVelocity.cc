/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#include <iostream>
#include <memory>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>

// This header is required to register plugins.
// The good practice suggests to recall it in the ".cc" file

#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>

// The next row recalls the plugin's header.

#include "PluginLinearVelocity.hh"

// This is required to register the plugin.
// NOTE: The interfaces shall match what's in the plugin's header (see line 42 to 44).

GZ_ADD_PLUGIN(
    plugin_velocity::PluginLinearVelocity,
    gz::sim::System,
    plugin_velocity::PluginLinearVelocity::ISystemConfigure,
    plugin_velocity::PluginLinearVelocity::ISystemPreUpdate)


using namespace plugin_velocity;

// In the next section, all the functions that we need are recalled.
// Since the plugin's header (see PluginLinearVelocity.hh file) is used as "function declaration",
// in the following section the core purpose of our plugin will be defined.

// For this plugin I decided to use the "Configure function" to read the field in the .sdf file
// as e.g. the link_name field. Let's dive into the details, analyzing step-by-step the following code.


void plugin_velocity::PluginLinearVelocity::Configure(const gz::sim::Entity &_identity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr)
{
    // STEP1) Read property from SDF.
    // In this case, I call it "link_name" (see line 71 in the velocity_world_plugin.sdf file)

    auto linkName = _sdf->Get<std::string>("link_name");

    // STEP 2) Store the lName variable the value of the "link_name"

    this -> lName = linkName; // Do not forget to declare the lName variable in the plugin's header file (see line 61 in the PluginLinearVelocity.hh file))

    // Store the _identity value using the "this ->".

    this -> entity = _identity;

    // Create the model object to access all the function using as attibute the "entity" variable.
    // The function "Model()" enables to get some important details 

    auto model = gz::sim::Model(entity);

    // Look for link entity using "LinkByName" into "model" defined above. 
    
    this -> linkEntity = model.LinkByName(_ecm, linkName);
}


void PluginLinearVelocity::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{

    // Define the linear velocity vector using.

    gz::math::Vector3d lVel(1,0,0);

    // Create an object using the "Link" class.

    gz::sim::Link link_n(linkEntity);

    // The next row is the main command wich applies the linear velocity vector "lVel" on the link entity.
    // For this example we are applying the linear velocity on a simple box that we defined in the sdf file (see line 34 in the velocity_world_plugin.sdf file)

    link_n.SetLinearVelocity(_ecm,lVel);
}

// end
