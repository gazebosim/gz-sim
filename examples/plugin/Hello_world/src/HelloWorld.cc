/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>


namespace ignition;
namespace gazebo;
namespace systems;

// A simple Hello world plugin
class ignition::gazebo::systems::HelloWorld

{   
};

/// \ brief Hello World data class
HelloWorld::Hello_world() : System(), dataPtr(std::make_unique<HelloWorld>())


HelloWworld::~HelloWorld() = default;

// Implement Configure callback, provided by ISystemConfigure
// and called once at startup.

void MyPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)

{       
   printf("Hello World!\n");
}

// Register the plugin

IGNITION_ADD_PLUGIN(HelloWorld, System, HelloWorld::ISystemConfigure)


IGNITION_ADD_PLUGIN_ALIAS(HelloWorld,"ignition::gazebo::systems::HelloWorld")
