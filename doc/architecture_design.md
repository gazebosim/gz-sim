# Ignition Gazebo design

> This is an evolving document as design is discussed and iterated on. As features are released,
  the relevant parts of this document should be updated and moved to a tutorial.

## High level behavior

The `Server` loads an SDF file, which may contain multiple worlds, and each
world may be divided into multiple levels.

Ideally, the server should only load the high-level information from the file
such as **worlds**, **levels** and **performers**, but not all entities
immediately, and only load entities according to how simulation is distributed
or which levels are enabled.

> **TODO**: It may be good for performance if the SDFormat library provided a
    way to incrementally load the world, or at least not to create objects in
    memory for everything at once.

### Multiple worlds

> **TODO**: Decide whether to, and how to, support multiple `<world>` tags.

There are use cases for running multiple different worlds in parallel, either in
the same process or different ones.

For these cases, each world could be simulated in a single runner, which will
run all systems and handle all the entities in the world.

In this setup, there's no concept of primary / secondary runners. However, they
will run in lock-step and their data can be collected in a unified manner.

### ![](https://66.media.tumblr.com/c95e2b7e9bd698f0ef7b968faaed23f7/tumblr_inline_mkn8tqM06r1roozkr.gif) Multiple performers, multiple servers

> **This whole section is ![](https://66.media.tumblr.com/c95e2b7e9bd698f0ef7b968faaed23f7/tumblr_inline_mkn8tqM06r1roozkr.gif)**

In case there are multiple performers, the simulation will be broken down into:

* 2 or more **secondary simulation runners**, each simulating 1 or more levels;
* 1 **primary simulation** runner, which is responsible for keeping the
  secondaries in sync.

The total number of runners will be predefined through SDF, as well as the
affinity of levels and performers to each runner. Depending on the world
configuration at a given time, some runners may be in stand-by, not performing
simulation, for example, when all performers are physically interacting with
each other.

Let's take a look at the following example.

![](architecture_design/06.png)

* There are now 3 performers: `R1`~`R3`

* `R1` and `R2` are both in level `L1`, while `R3` is in `L2`

* The server spins up 3 runners, as described in SDF:
    * The primary runner
    * A secondary runner (`SR1`) with `L1` loaded, together with `R1` and `R2` -
      represented by the bright green outline.
    * A secondary runner (`SR2`) with `L2` loaded, together with `R3` -
      represented by the bright pink outline.

* Note that `M6` is loaded by both secondaries, since it is a global entity.

* During simulation, the primary keeps track of whether performers are
  entering any buffer zones.

Let's say that `R1` does the same movement it did in the example above,
from `L1` to `L3`. In this case, since there are no secondary runners in
stand-by, `SR1` will be simulating both `L1` and `L3`, while `SR2` keeps
simulating just `L2`.

> If, however, the simulation had been started with 3 secondaries and one
of them (`SR3`) was in stand-by, that runner would become responsible for `L3`.

In case `R1` moves towards `L2` however, the following happens:

1. `R1` enters `L2`'s buffer zone

    ![](architecture_design/07.png)

1. The primary detects it and forwards `R1`'s current state to `SR2`. At this
time, both `SR1` and `SR2` have `R1` loaded, so `R3`'s sensors can detect `R1`,
but only `SR1`'s physics is acting on `R1`.
1. Once `R1` moves into `L2`, `SR2` takes over its physics simulation, but `SR1`
still keeps track of its state.

    ![](architecture_design/08.png)

1. Only once `R1` exits `L1`'s buffer zone it is that `SR1` unloads its
entities.

> **TODO**: How to handle a performer that interacts with other performers in
two different levels at the same time?

> **TODO**: How to decide whether `R1` should be moved to `SR2` or if `L2`
should be loaded into `SR1` together with `L1`? There could be a situation
where `SR2` is already simulating too many levels.

### ![](https://66.media.tumblr.com/c95e2b7e9bd698f0ef7b968faaed23f7/tumblr_inline_mkn8tqM06r1roozkr.gif) Keeping runners in sync

Each secondary runner has its own ECS, with detailed entities loaded. The
primary runner keeps a high-level ECM, which keeps track of which performers are
in which levels and whether they've reached the buffer zone. The server uses
this information to keep performers synced across runners and to make sure each
level and performer is only simulated at one runner at a time.

> **TODO**: Explain how the primary does this

## ![](https://66.media.tumblr.com/c95e2b7e9bd698f0ef7b968faaed23f7/tumblr_inline_mkn8tqM06r1roozkr.gif) Component serialization

> **TODO**: Describe how components will be serialized to be sent across runners
> so their state is synced.


## Plugins

Classic Gazebo supported 6 different C++ plugin types, which provided access to
different parts of the API, like physics, rendering, sensors, GUI, etc. Due to Ignition
Gazebo's architecture based on an ECS, plugin interfaces will be somewhat different,
but more varied and in many cases much more powerful. Some plugins will be systems
within Ignition Gazebo, while others will be specific plugin types from other Ignition
libraries.

> **NOTE**: We may add other plugins to Ignition Gazebo which are not systems in the
  future.

For example, plugins which get and set properties of simulation entities would be
Ignition Gazebo systems. On the other hand, there are now plugin interfaces which didn't
exist in Gazebo, such as integrating a new physics or rendering engine, and these can
exist outside of Ignition Gazebo.

Take a look at the comparison below:

Gazebo plugin | Features | Ignition equivalent | Differences
------------- | -------- | ------------------- | -----------
World | Get/set properties of the world and its children | Gazebo system | Will be done through components.
Model | Get/set properties of the model and its children | Gazebo system | Will be done through components.
Visual | Get/set properties of the visual and its children | Gazebo system | Will be done through components.
Sensor | Get/set sensor properties and readings | Gazebo system | Will be done through components.
World / Model | Access physics-engine-specific features | Physics plugin | Specified on SDF and passed to physics
Visual | Access rendering-engine-specific features | Rendering plugin | Specified on SDF and passed to rendering
Sensor | Connect to callbacks | Standalone program | Subscribe to Ignition Transport messages.
All | Connect to simulation events | Gazebo system | Use `PreUpdate`, `Update` and `PostUpdate` callbacks for the update loop, and the event manager for other events.
GUI | Add an overlay UI | GUI plugin | More customization available by default
GUI / System | Change the default UI | GUI plugin / SDF | All GUI elements can be removed and added through SDF
System | Access command line arguments | TBD |

Another key difference is that systems will be able to access all entity
properties at once, despite the entity type. So while in Gazebo you may
need 3 plugins to interact with physics, rendering and sensors, on
Ignition you could, for example, set a camera's visual color, its velocity and
frame rate, all from a single plugin.

> **TODO**: Mention specific concerns plugin devs need to have when writing
  plugins for distributed simulation.

## Plugin interfaces

Let's take a look at a typical Gazebo plugin which accesses a property from an
entity and does something with it. In this case, it will print the current pose
of a link.

In general, the plugin will need to:

1. Get the link and store it.
1. Register a callback that is called at every simulation update.
1. At the callback, query the link's pose and print it.

On Gazebo, that would look something like this:

---

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

---

In Ignition Gazebo, that would be implemented as follows:

---

```cpp
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
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
    auto linkId = model->LinkByName(_ecm, linkName);

    // Create link object and store it so we can easily access its API later
    this->link = Link(linkId);
  }

  // Implement PostUpdate callback, provided by ISystemPostUpdate
  // and called at every iteration
  void MyPlugin::PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm)
  {
    // Get link pose and print it
    std::cout << this->link->Pose(_ecm) << std::endl;
  }

  // Object providing a convenient interface for link entities
  private: Link link;
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

---

Some of the key differences are:

* Plugins must inherit from the `ISystemConfigure` class to be able to override
  the `Configure` callback that gives access to many things, such as the SDF
  pointer.

* Plugins no longer need to manage connections to loop-related events. Instead,
  they implement an interface such as `ISystemUpdate`.

* Plugins don't have direct access to physics objects such as `physics::Link`.
  Instead, they can either deal directly with entities and their components by
  calling functions in the ECM, or use convenient objects such as
  `ignition::gazebo::Link` which wraps around the ECM interface.

All these changes are meant to give plugin developers more flexibility to
only use the features they need, and several layers of abstraction which
can be chosen according to the developer's experience and specific use-case.

## Standalone programs

It would be convenient to be able to specify standalone programs in the SDF
file so they're loaded at the same time as the simulation. For example,
Gazebo's
[JoyPlugin](https://bitbucket.org/osrf/gazebo/src/default/plugins/JoyPlugin.hh?fileviewer=file-view-default)
is a `WorldPlugin`, but it doesn't need to access any world API, or to run
in the physics thread, or even to run in the gzserver process. However,
it was implemented as a plugin because that makes it easier to specify in
SDF.

> **TODO**: Describe what this would look like.



