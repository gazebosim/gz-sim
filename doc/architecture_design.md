# Design document for distrbuted simulation

## Goals

* Simulation can be distributed among 1 or more processes
* These processes can be running on the same machine or different ones
* These processes must be kept in sync
* Results can't be interpolated or missed
* We should reduce the amount of duplicate effort across processes

## Terminology

* **World**: The complete description of a simulation, including all robots,
    movable and static objects, plugins, scene and GUIs.

* **Level**: Part of a world, defined by a box volume and the entities inside
    it. Each level has buffer zones, which are areas close to its boundaries
    from which a performer may cross to a different level.

* **Entity-component manager**: **ECM** for short. Manages a set of entities
    and their components.

* **Simulation runner**: Runs a whole world or levels of a world, but no more
    than 1 world.
    * It has a single ECM with all the entities and components relevant to the
      level / world that is being simulated.
    * It has an event manager
    * Each simulation runner runs in a separate process.

* **Server**: Ignition Gazebo's entry point. It's responsible for loading an
    SDF file, spinning up simulation runners accordingly, and keeping track of
    the global simulation state .

* **Performer**: All simulation entities which may change levels during the
    simulation, such as robots, actors and dynamic models. A performer only
    has meaning if there are levels.

* **Global entities**: Entities which are present on all levels, such as the
    sun, ground plane, heightmaps, etc.

## Spreading the work

The `Server` loads an SDF file, which may contain multiple worlds, and each
world may be divided into multiple levels.

Ideally, the server should only load the high-level information from the file
such as **worlds**, **levels** and **performers**, since it only needs enough
information to decide how to spin up simulation runners and split up the work
among them. The server shouldn't be concerned about individual entities like
links, visuals, collisions, etc.

> **TODO**: It may be good for performance if the SDFormat library provided a
    way to incrementally load the world, or at least not to create objects in
    memory for everything in a file.

### Worlds without levels

The simplest case is for worlds without levels or performers, which can be considered the same as a world with a single level.

For these cases, the whole world is simulated in a single runner. If there are
multiple worlds in the SDF file, each world will get a runner.

> TODO: explain the server's role in keeping these worlds running in sync and
collecting data in a unified manner.

### Worlds with multiple levels and one performer

If a world has a single performer, who may be moving across several levels,
the server spins up a **single simulation runner**, which will initially load
only the level which contains the performer. As the performer moves in the
world, the runner will load / unload levels as needed.

* To "load" a level is to create entities and components for the models in that
level.
* To "unload" a level is to delete entities and components for the models in
that level.

> TODO: how does the server initially divide the work across runners, and keeps
them in sync afterwards? Is there a predefined number of runners or can they be
spinned / killed at runtime?

    Consider a few cases:

    1. Simulation starts with all performers in the same level, and as
       simulation evolves performers may split across multiple levels.

    1. Simulation starts with performers spread around, and performers may
       get into the same level and interact.


### Keeping runners in sync

Each runner has its own ECS, with detailed entities loaded. The server keeps a
high-level ECM, which keeps track of which performers are in which levels and
whether they've reached the buffer zone. The server uses this information to
keep performers synced across runners and to make sure each level is only
simulated at a runner at a time.


