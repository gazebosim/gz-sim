\page migrationplugins

# Migration from Gazebo Classic: Plugins

Gazebo Classic supports
[6 different C++ plugin types](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin),
each providing access to different parts of the API, like physics, rendering,
sensors, GUI, etc. Due to Ignition Gazebo's architecture based on an
[ECS](https://en.wikipedia.org/wiki/Entity_component_system)
, plugin interfaces are somewhat different, but more varied and in many cases much
more powerful. Some plugins in Ignition are systems within Ignition Gazebo,
while others are specific plugin types from other Ignition libraries.

\note Plugin types other than systems may be added to Ignition Gazebo in the future.

For example, plugins which get and set properties of simulation entities would be
Ignition Gazebo systems. On the other hand, there are now plugin interfaces which didn't
exist in Gazebo Classic, such as integrating a new physics or rendering engine, and these can
be also used in projects outside of Ignition Gazebo.

Take a look at the comparison below:

---

Classic plugin | Features | Ignition equivalent | Differences
------------- | -------- | ------------------- | -----------
World | Get/set properties of the world and its children | Gazebo system | Get/set components.
Model | Get/set properties of the model and its children | Gazebo system | Get/set components.
Visual | Get/set properties of the visual and its children | Gazebo system | Get/set components.
Sensor | Get/set sensor properties and readings | Gazebo system | Get/set components.
World / Model | Access physics-engine-specific features | Physics plugin | Loaded by ign-physics
Visual | Access rendering-engine-specific features | Rendering plugin | Loaded by ign-rendering
Sensor | Connect to callbacks | Standalone program | Subscribe to Ignition Transport messages.
All | Connect to simulation events | Gazebo system | Use `PreUpdate`, `Update` and `PostUpdate` callbacks for the update loop, and the event manager for other events.
GUI | Add an overlay UI | GUI plugin | Support for overlays and docked widgets
GUI / System | Change the default UI | GUI plugin / SDF | All GUI elements can be removed or configured through SDF
System | Access command line arguments | TBD | -

---

Another key difference is that systems will be able to access all entity
properties at once, despite the entity type. So while in Gazebo Classic you may
need 3 different plugins to interact with physics, rendering and sensors, on
Ignition you could, for example, set a camera's visual color, its velocity and
frame rate, all from a single plugin.

## Plugin interfaces

Let's take a look at a typical Gazebo Classic plugin which accesses a property from an
entity and does something with it. In this case, it will print the current pose
of a link.

In general, the plugin will need to:

1. Get the link and store it.
2. Register a callback that is called at every simulation update.
3. At the callback, query the link's pose and print it.

On classic Gazebo, that would look something like this:

```cpp
#include <gazebo/common/Plugin.hh>

// Inherit from ModelPlugin
class MyPlugin : public ModelPlugin
{
  // Implement Load callback, provided by ModelPlugin
  // and called once at startup.
  virtual void MyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Read property from SDF
    auto linkName = _sdf->Get<std::string>("link_name");

    // Store link pointer
    this->link = _model->GetLink(linkName);

    // Register callback to be called at every iteration
    this->connection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MyPlugin::OnUpdate, this));
  }

  // Custom callback called at every iteration
  void MyPlugin::OnUpdate()
  {
    // Get link pose and print it
    std::cout << this->link->WorldPose() << std::endl;
  }

  // Must keep the connection
  private: event::ConnectionPtr updateConnection;

  // Keep pointer to link
  private: physics::LinkPtr link;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(MyPlugin)
```

On Ignition Gazebo, that would be implemented as follows:

```cpp
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  // Implement Configure callback, provided by ISystemConfigure
  // and called once at startup.
  void MyPlugin::Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &/*_eventMgr*/)
  {
    // Read property from SDF
    auto linkName = _sdf->Get<std::string>("link_name");

    // Create model object, to access convenient functions
    auto model = Model(_entity);

    // Get link entity
    auto linkEntity = model->LinkByName(_ecm, linkName);
  }

  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration, after physics is done
  void MyPlugin::PostUpdate(const UpdateInfo &/*_info*/,
      const EntityComponentManager &_ecm)
  {
    // Get link pose and print it
    std::cout << worldPose(this->linkEntity, _ecm) << std::endl;
  }

  // ID of link entity
  private: Entity linkEntity;
};

// Register plugin
IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gazebo::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"ignition::gazebo::systems::MyPlugin")
```

The example above uses headers like `Model.hh` and `Util.hh`, which offer
convenient APIs for some tasks that are common during simulation. However,
the real power of the ECS architecture is the direct access to all
entities and components. Let's take a look at how to do the same just using
the ECM's API:

```cpp
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>

// Inherit from System and 2 extra interfaces:
// ISystemConfigure and ISystemPostUpdate
class MyPlugin
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
{
  void MyPlugin::Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &/*_eventMgr*/)
  {
    auto linkName = _sdf->Get<std::string>("link_name");

    // Create link entity by querying for an entity that has a specific set of
    // components
    this->linkEntity = _ecm.EntityByComponents(
        components::ParentEntity(this->dataPtr->id),
        components::Name(_name),
        components::Link());
  }

  void MyPlugin::PostUpdate(const UpdateInfo &/*_info*/,
      const EntityComponentManager &_ecm)
  {
    // Get link's local pose
    auto pose = _ecm.Component<components::Pose>(this->linkEntity)->Data();

    // Get link's parent entity
    auto p = _ecm.Component<components::ParentEntity>(this->linkEntity);

    // Iterate over parents until world is reached
    while (p)
    {
      // Get parent entity's pose
      auto parentPose = _ecm.Component<components::Pose>(p->Data());
      if (!parentPose)
        break;

      // Add pose
      pose = pose + parentPose->Data();

      // keep going up the tree
      p = _ecm.Component<components::ParentEntity>(p->Data());
    }

    std::cout << pose << std::endl;
  }

  // ID of link entity
  private: Entity linkEntity;
};

IGNITION_ADD_PLUGIN(MyPlugin,
                    ignition::gazebo::System,
                    MyPlugin::ISystemConfigure,
                    MyPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MyPlugin,"ignition::gazebo::systems::MyPlugin")
```

In summary, the key differences between Gazebo Classic and Ignition Gazebo are:

* Plugins must inherit from the `ISystemConfigure` class to be able to override
  the `Configure` callback that gives access to many things, such as the SDF
  pointer. This used to be done through the `Load` callback.

* Plugins no longer need to manage connections to loop-related events. Instead,
  they implement an interface such as `ISystemPostUpdate`.

* Plugins don't have direct access to physics objects such as `physics::Model`.
  Instead, they can either deal directly with entities and their components by
  calling functions in the ECM, or using convenient objects such as
  `ignition::gazebo::Model` which wrap the ECM interface.

All these changes are meant to give plugin developers more flexibility to
only use the features they need, and several layers of abstraction which
can be chosen according to the developer's experience and specific use-case.

