#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>


namespace ignition;
namespace gazebo;
namespace systems;

// Inherit from System and 2 extra interfaces
// ISystemConfigure and ISystemPostUpdate

class ignition::gazebo::systems::Hello_world

{   
};
/// \ brief Hello_world data class
Hello_world::Hello_world() : System(), dataPtr(std::make_unique<Hello_world>())

//////////////////////////////////////////////////////////////

Hello_world::~Hello_world() = default;

// Implement Configure callback, provided by ISystemConfigure
// and called once at startup.
void MyPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)

{
	printf("Hello World!\n");
}

// Register plugin

IGNITION_ADD_PLUGIN(Hello_world, System, Hello_world::ISystemConfigure)


IGNITION_ADD_PLUGIN_ALIAS(Hello_world,"ignition::gazebo::systems::Hello_world")