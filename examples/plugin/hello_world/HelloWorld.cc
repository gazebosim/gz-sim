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

// We'll use a string and the ignmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <ignition/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <ignition/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "HelloWorld.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    ignition::gazebo::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void HelloWorld::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  // Messages printed with ignmsg only show when running with verbosity 3 or
  // higher (i.e. ign gazebo -v 3)
  ignmsg << msg << std::endl;
}



