/*
Copyright 2025 Open Source Robotics Foundation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef SYSTEM_PLUGIN_HELLOWORLD_HH_
#define SYSTEM_PLUGIN_HELLOWORLD_HH_


#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Vector3.hh>


// When a plugin's header file is coded, it is a good practice to define a customized namespace.
// In this case we call it "plugin_plugin_velocity".

namespace plugin_velocity
{

  // Under our customized namespace we define and recall all the functions and variables that we need.

  // Since our objective is to:
  // - Read property from the .sdf file
  // - Send the LinearVelovity command to a link entity

  // Here we have to use "ISystemConfigure" and "ISystemPreUpdate".

  class PluginLinearVelocity:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    // The PreUpdate callback must be imported by the plugins that are inheriting ISystemPreUpdate.
    // Since its nature, the PreUpdate callback is called every simulation iteration before the physics updates the world.
    // On the other hand the Configure callback is called only once.
    
    public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr)override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    private: 
      gz::sim::Entity linkEntity;
      gz::sim::Entity entity;
      std::string lName;
  };
}

#endif
