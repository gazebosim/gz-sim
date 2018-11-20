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
    it.

* **Buffer zone**: Each level has a buffer zone, which is an inflation of the
    level's volume outside its boundaries used to detect when a performer
    is about to come into the level, or has left and is far enough.

* **Entity-component manager**: **ECM** for short. Manages a set of entities
    and their components.

* **Simulation runner**: Runs a whole world or some levels of a world, but no
    more than 1 world.
    * It has a single ECM with all the entities and components relevant to the
      levels / world being simulated.
    * It has an event manager.
    * Each simulation runner may run in a separate process, or share a process
      with other runners - this is decided at runtime.

* **Primary / secondary runner**: For each world, there is exactly one primary
    simulation runner and one or more secondary runners. The **secondary**
    runners are running levels of the world, while the **primary** runner is
    keeping all secondaries in sync.

* **Server**: Ignition Gazebo's entry point. It's responsible for loading an
    SDF file and spinning up simulation runners accordingly.

* **Performer**: All simulation entities which may change levels during the
    simulation, such as robots, actors and dynamic models. A performer only
    has meaning if there are levels.

    > An alternative would be that actors and other simple dynamic models are
      handled by the primary runner.

* **Global entities**: Entities which are present on all levels, such as the
    sun, ground plane, heightmaps, etc. These entities will be duplicated
    across all simulation runners.

## High level behavior

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

### No levels

The simplest case is for worlds without levels or performers. This is
essentially the same as a world with a single level.

For these cases, each world is simulated in a single runner, which will
run all systems and handle all the entities in the world.

In this setup, there's no concept of primary / secondary runners. However, since
worlds are being run in the same server, they will run in lock-step and their
data can be collected in a unified manner.

### Multiple levels, one performer

If a world has a single performer, who may be moving across several levels,
the server spins up a **single runner**, which will initially load only the
level which contains the performer. As the performer moves in the world, the
runner will load / unload levels as needed.

* To "load" a level is to create entities and components for the models in that
level.
* To "unload" a level is to delete entities and components for the models in
that level.
* Systems that keep internal state will need to react appropriately to entities
being added and removed.
* A level is loaded when the performer enters its external buffer zone.
* A level is unloaded when the performer exits its external buffer zone.

Take a look at the 2D example below.

![](architecture_design/01.png)

* All the **green area** represents the area of the world which this simulation
  is expected to take place in.

* Each of the areas delimited by **dashed purple lines** is a level. There are 3
levels (`L1`~`L3`).

* The **light blue area** represents the buffer zone for level `L1`. Zones
  for `L2` and `L3` have been ommitted.

* Each **orange shape** represents a static model in the world (`M1`~`M6`)

* The **red shape** represents the robot performer (`R1`)

Entities are divided into those that belong to a level, and global entities:

* `M1` belongs to `L1`, so when `L1` is loaded / unloaded, entities will be
  created / destroyed for that model. Likewise, `M2` belongs to `L2`, and `M4`
  and `M5` belong to `L3`.

* `M3` is present in more than one level, so it is treated as a
    **global entity**.

* `M6` is not in any level, so it is also treated as a **global entity** and is
    always loaded. Ideally, this kind of entity should be avoided unless there's
   a need for it.

Let's take a look at how levels are loaded / unloaded as the performer moves:

1. Performer `R1` starts inside level `L1`. This means that the simulator runner
    will initially have loaded the following, which is represented by bright
    green lines:

    * `R1`, which is the performer.
    * `M1`, because it belongs to the level.
    * `M3` and `M6`, because they are global.

1. The performer moves south towards `L3` and enters its buffer zone, triggering
  a load of that level's models, `M4` and `M5`. Note that at this moment, both
  `L1` and `L3` are loaded.

1. The performer moves further south, exiting `L1` and entering `L3`. However,
  `L1` is still loaded, since `R1` is still within its buffer zone.

1. Eventually `R1` moves beyond `L1`'s buffer, triggering an unload of `L1`. The
  main effect is unloading `M1`.

### Multiple performers

In case there are multiple performers, the simulation will be broken down into:

* 2 or more secondary simulation runners, each simulating 1 or more levels;
* 1 primary simulation runner, which is responsible for keeping the secondaries
  in sync.

> TODO: how does the server initially divide the work across runners, and keeps
> them in sync afterwards? Is there a predefined number of runners or can they
> be spinned / killed at runtime?
>
>    Consider a few cases:
>
>    1. Simulation starts with all performers in the same level, and as
>       simulation evolves performers may split across multiple levels.
>
>    1. Simulation starts with performers spread around, and performers may
>       get into the same level and interact.

Let's take a look at the following example.

* There are now 3 performers: `R1`~`R3`

* `R1` and `R2` are both in level `L1`, while `R3` is in `L2`

* The server spins up 3 runners:
    * The primary runner
    * A secondary runner (`SR1`) with `L1` loaded, together with `R1` and `R2` -
      represented by the bright green outline.
    * A secondary runner (`SR2`) with `L2` loaded, together with `R3` -
      represented by the bright pink outline.

* Note that `M3` and `M6` are loaded by both secondaries, since they are global
  entities.

* During simulation, the primary keeps track of whether performers are
  entering any buffer zones.

Let's say that `R1` does the same movement it did in the example above,
from `L1` to `L3`. In this case, the server can decide to either:

* Keep simulating both `L1` (which still contains `R2`) and `L3` (which contains
`R1`) within `SR1`;
* Or to spin up a new secondary when `R1` is fully within `L3`.

> **TODO**: Based on what? Does the user provide a number of maximum runners at
> startup? A non-strict target number of total runners?

In case `R1` moves towards `L2` however, the following happens:

1. `R1` enters `L2`'s buffer zone
1. The primary detects it and forwards `R1`'s current state to `SR2`. At this
time, both `SR1` and `SR2` have `R1` loaded, but only `SR1`'s physics is acting
on `R1`.
1. Once `R1` moves into `L2`, `SR2` takes over its physics simulation, but `SR1`
still keeps track of its state.
1. Only once `R1` exits `L1`'s buffer zone it is that `SR1` unloads its
entities.

> **TODO**: How to handle a performer that interacts with other performers in
two different levels at the same time?

> **TODO**: How to decide whether `R1` should be moved to `SR2` or if `L2`
should be loaded into `SR1` together with `L1`? There could be a situation
where `SR2` is already simulating too many levels.

### Keeping runners in sync

Each secondary runner has its own ECS, with detailed entities loaded. The
primary runner keeps a high-level ECM, which keeps track of which performers are
in which levels and whether they've reached the buffer zone. The server uses
this information to keep performers synced across runners and to make sure each
level and performer is only simulated at one runner at a time.

> **TODO**: Explain how the primary does this

## SDF elements

> **TODO**: Describe the new SDF tags for levels and performers, whether they're
> part of the SDF spec or if they're loaded within plugins, where they're
> loaded, etc.

## Component serialization

> **TODO**: Describe how components will be serialized to be sent across runners
> so their state is synced.

