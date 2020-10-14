\page terminology Terminology

This is a list of definitions used throughout Ignition Gazebo. Some of them
are important for downstream users, while some of them are only interesting
to developers touching the source code.

* **World**: The complete description of a simulation, including all robots,
    movable and static objects, plugins, scenes and GUIs. It corresponds to
    an SDF `<world>` tag.

* **Entity**: Every "object" in the world, such as models, links,
    collisions, visuals, lights, joints, etc.
    An entity [is just a numeric ID](namespaceignition_1_1gazebo.html#ad83694d867b0e3a9446b535b5dfd208d),
    and may have several components attached to it. Entity IDs are assigned
    at runtime.

* **Component**: Adds a certain functionality or characteristic (e.g., pose,
    name, material, etc.) to an entity.
    Ignition Gazebo comes with various
    [components](namespaceignition_1_1gazebo_1_1components.html)
    ready to be used, such as `Pose` and `Inertial`, and downstream developers
    can also create their own by inheriting from the
    [BaseComponent](classignition_1_1gazebo_1_1components_1_1BaseComponent.html)
    class or instantiating a template of
    [Component](classignition_1_1gazebo_1_1components_1_1Component.html).

* **System**: Logic that operates on all entities that have a given set of
    components. Systems are plugins that can be loaded at runtime.
    Ignition Gazebo ships with various systems, and downstream develpers can
    [create their own systems](createsystemplugins.html).

* **Entity-component manager** (**ECM**): Provides functions for
    querying, creating, removing and updating entities and components.
    See the whole API
    [here](classignition_1_1gazebo_1_1EntityComponentManager.html).

* **Level**: Part of a world, defined by a box volume and the static entities
    inside it. An entity can be present in more than one level, or in none of
    them. Levels may overlap in their volumes and may be far from each other.

* **Buffer zone**: Each level has a buffer zone, which is an inflation of the
    level's volume outside its boundaries used to detect when a performer
    is about to come into the level, or has left and is far enough away to
    exclude the entity from the level.

* **Performer**: All simulation entities which may change levels during the
    simulation, such as robots, actors and dynamic models. A performer only
    has meaning if there are levels.

* **Global entities**: Entities which are present on all levels, such as the
    sun, ground plane, heightmaps, etc. These entities will be duplicated
    across all simulation runners.

* **Default level**: Level which handles all entities that are not within
    any other levels.

* **Network manager**: Controls the flow of information in a simulation
    distributed across processes.

* **Primary / secondary network manager**: For each world that is split
    across multiple managers, there is exactly one primary network and one or more
    secondary runners. The **secondary** runners are running a set of levels of
    the world, while the **primary** runner is keeping all secondaries in sync.
    Worlds that are not split across runners don't have a primary runner.

* **[Event manager](classignition_1_1gazebo_1_1EventManager.html)**:
    Manages events that can be sent across systems and the server. Plugins can
    create and emit custom
    [Event](https://ignitionrobotics.org/api/common/3.0/classignition_1_1common_1_1Event.html)s
    and / or emit / listen to events from Ignition Gazebo.

* **Simulation runner**: Runs a whole world or some levels of a world, but no
    more than 1 world.
    * It has a single ECM with all the entities and components
      relevant to the levels / world / performer being simulated.
        * It's still TBD how to support multiple `<worlds>` in parallel.
    * It has an event manager.
    * It has a network manager, if simulation is distributed.
    * It loads up a set of systems.

* **Server**: Ignition Gazebo's entry point. It's responsible for loading an
    SDF file and instantiating a simulation runner per world.

