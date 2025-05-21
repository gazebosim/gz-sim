#include <string>
#include <iostream>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>

// This header is required to register plugins.
// The good practice suggests to recall it in the ".cc" file

#include <gz/plugin/Register.hh>


#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>


// The next row recalls the plugin's header.

#include "PluginLinearVelocity.hh"

using namespace gz;
using namespace sim;
using namespace systems;


// This is required to register the plugin.
// NOTE: The interfaces shall match what's in the plugin's header (see line 26).

GZ_ADD_PLUGIN(
    plugin_test::PluginLinearVelocity,
    gz::sim::System,
    plugin_test::PluginLinearVelocity::ISystemConfigure,
    plugin_test::PluginLinearVelocity::ISystemPreUpdate)


using namespace plugin_test;

// In the next section, all the functions that we need are recalled.
// Since the plugin's header (see PluginLinearVelocity.hh file) is used as "function declaration",
// in the following section the core purpose of our plugin will be defined.

// For this plugin I decided to use the "Configure function" to read the field in the .sdf file
// as e.g. the link_name field. Let's dive into the details, analyzing step-by-step the following code.


void plugin_test::PluginLinearVelocity::Configure(const gz::sim::Entity &_identity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr)
  {
    // STEP1) Read property from SDF.
    // In this case, I call it "link_name" (see line 84 in the velocity_world_plugin.sdf file)

    auto linkName = _sdf->Get<std::string>("link_name");

    // STEP 2) Store the lName variable the value of the "link_name"

    this -> lName = linkName; // Do not forget to declare the lName variable in the plugin's header file (see line 50 in the PluginLinearVelocity.hh file))

    // Store the _identity value using the "this ->"
    this -> entity=_identity;

    // Create the model object to access all the function using as attibute the "entity" variable.
    // The function "Model()" enables to get some important details 
    
    auto model = Model(entity);

    // Look for link entity using "LinkByName" into "model" defined above 
    
    this-> linkEntity = model.LinkByName(_ecm, linkName);

  }


void PluginLinearVelocity::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{

    // Define the linear velocity vector using.

    math::Vector3d lVel(1,0,0);

    // Create an object using the "Link" class.

    Link link_n(linkEntity);

    // The next row is the main command wich applies the linear velocity vector "lVel" on the link entity.
    // For this example we are applying the linear velocity on a simple box that we defined in the sdf file (see line 39 in the velocity_world_plugin.sdf file)

    link_n.SetLinearVelocity(_ecm,lVel);
    

}

// end