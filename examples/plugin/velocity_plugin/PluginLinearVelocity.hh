#ifndef SYSTEM_PLUGIN_HELLOWORLD_HH_
#define SYSTEM_PLUGIN_HELLOWORLD_HH_


#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Vector3.hh>


using namespace gz;
using namespace sim;
using namespace systems;

// When a plugin's header file is coded, it is a good practice to define a customized namespace.
// In this case we call it "plugin_test"

namespace plugin_test
{

  // Under our customized namespace we define and recall all the functions and variables that we need

  // Since our objective is to:
  // - Read property from the .sdf file
  // - Send the LinearVelovity command to a link entity

  // Here we have to use "ISystemConfigure" and "ISystemPreUpdate"

  class PluginLinearVelocity:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    // The PreUpdate callback must be imported by the plugins that are inheriting ISystemPreUpdate.
    // Since its nature, the PreUpdate callback is called every
    // simulation iteration before the physics updates the world.
    // On the other hand the Configure callback is called only once.
    
    public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    private: 
      gz::sim::Entity linkEntity;
      gz::sim::Entity entity;
      std::string lName;
  };
}

#endif
