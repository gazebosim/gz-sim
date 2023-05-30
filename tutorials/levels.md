\page levels Levels

This tutorial gives an introduction to Gazebo Sim's levels feature.
This feature allows loading and unloading objects in simulation according
to their proximity to the robot, which improves performance in simulations
with large environments.

A level is a part of a world defined by a box volume and the static entities
inside it. An entity can be present in more than one level, or in none of
them. Levels may overlap in their volumes and may be far from each other.

Each level has a buffer zone, which is an inflation of the
level's volume outside its boundaries used to detect when a performer
is about to come into the level, or has left and is far enough away to
exclude the entity from the level.

All simulation entities which may change levels during the
simulation, such as robots, actors and dynamic models are called Performers.
A performer only has meaning in the context of levels or distributed simulation.

To enable levels, it is necessary to add appropriate tags in the SDF file
as described below, and to launch Gazebo with the `--levels` flag.

All the situations described here refer to simulation running in a single
server.

\note Take a look at the [terminology](terminology.html) tutorial to get familiar with concepts used in this tutorial.

## Try it out

Gazebo ships with an example world that demos the levels feature. Try it as follows:

1. Run the example world with the `--levels` flag:

    `gz sim levels.sdf --levels`

    Gazebo will open with a world that has 2 vehicles, one red and one blue.

2. Open a new terminal and publish the following commands for the vehicles to
    drive forward:

    `gz topic -t "/model/vehicle_blue/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 4.0}"`

    and

    `gz topic -t "/model/vehicle_red/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 2.0}"`

3. Press play on Gazebo. You'll see that the tunnels will be loaded as the
    vehicles move forward.


### No levels

The simplest case is a world without levels or performers. This is
essentially the same as a world with a single level.

This is the situation when the world file doesn't have `<level>` tags
or when Gazebo is launched without the `--levels` flag.

### Multiple levels, one server

If a world has a single performer, which may be moving across several levels,
only the level which contains the performer, and the default level, will be
initially loaded.
As the performer moves in the world, the runner will load / unload levels as
needed.

* To "load" a level is to create entities and components for the models in that
level.
* To "unload" a level is to remove entities and components for the models in
that level.
* Systems that keep internal state will need to react appropriately to entities
being added and removed.
* A level is loaded when the performer enters its buffer zone.
* A level is unloaded when the performer exits its buffer zone.

Take a look at the 2D example below. This example focuses on a single performer,
but the same logic can be extended to multiple performers.

<img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/01.png"/>

* The **green area** represents the area of the world which this simulation
  is expected to take place in.

* Each of the areas delimited by **dashed purple lines** is a level. There are 3
levels (`L1`~`L3`).

* The **light blue area** represents the buffer zone for level `L1`. Zones
  for `L2` and `L3` have been omitted.

* Each **orange shape** represents a static model in the world (`M1`~`M6`)

* The **red shape** represents the robot performer (`R1`)

Entities are divided into those that belong to levels, and global entities:

* `M1` belongs to `L1`, so when `L1` is loaded / unloaded, entities will be
    created / destroyed for that model. Likewise, `M2` belongs to `L2`, and `M4`
    and `M5` belong to `L3`.

* `M3` belongs to more than one level and will be loaded as long as one of those
  levels is loaded.

* `M6` is not in any level, so it is treated as a **global entity** and is
    always loaded. Ideally, this kind of entity should be avoided unless there's
    a need for it, such as an infinite ground plane.

Let's take a look at how levels are loaded / unloaded as the performer moves:

1. Performer `R1` starts inside level `L1`. This means that the simulation
    will initially have loaded the following, which is represented by bright
    green lines:

    * `R1`, which is the performer.
    * `M1` and `M3`, because they belong to the level.
    * `M6`, because it is global.

    <img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/02.png"/>

2. The performer moves south towards `L3` and enters its buffer zone, triggering
    a load of that level's models, `M4` and `M5`. Note that at this moment, both
    `L1` and `L3` are loaded.

    <img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/03.png"/>

3. The performer moves further south, exiting `L1` and entering `L3`. However,
  `L1` is still loaded, since `R1` is still within its buffer zone.

    <img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/04.png"/>

4. Eventually `R1` moves beyond `L1`'s buffer, triggering an unload of `L1`. The
  main effect is unloading `M1`.

    <img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/05.png"/>

## SDF elements

Two new SDF elements are introduced for distributed simulation:

* `<level>`
* `<performer>`

The concepts of levels and performers are specific to Gazebo, thus,
putting them directly under the `<world>` tag would diminish the generality of
SDF. A new tag, `<extension>`, has been proposed for such circumstances but has
not been implemented yet. Therefore, for now, the `<level>` and `<performer>`
tags will be added to a `<plugin name="gz::sim" filename="dummy">` tag.
The plugin name `gz::sim` will be fixed so that a simulation runner
would know to check for that name in each plugin tag.

### <level>

The `<level>` tag contains information about the volume occupied by the level
and the entities inside the level. The volume is given by a `<box>` geometry
(more shapes may be supported in the future) and it is used to determine whether
a performer is inside the level. Currently, the box shape is internally
converted into an axis aligned box to speed up intersection calculations. The
position of the volume is specified with respect to the world frame using the
`<pose>` tag. Although we are using the `<pose>` tag, the orientation part is
ignored. The `<buffer>` tag is used to express the **buffer zone** of the
volume.

Entities associated with the level are specified using the `<ref>` tag. The
value of this tag is the name of the entity. A level can contain one or more
`<ref>` tags. Note that it is this tag that determines whether an entity is
considered to be in the level or not. That is, an entity specified by a `<ref>`
would be considered part of a level even if its position is outside the level's
volume. It is up to the user to ensure that all entities specified by the
level's `<ref>` tags are contained within the level's volume.

Example snippet:

```xml
<level name="level1">
  <pose>0 0 5 0 0 0</pose>
  <geometry>
    <box>
      <size>10 10 10</size>
    </box>
  </geometry>
  <buffer>2</buffer>
  <ref>model1</ref>
  <ref>model2</ref>
</level>
```

### <performer>

\note See Runtime performers, the next section, for information about specifying performers without using the SDF `<performer>` tag.

The `<performer>` tag contains a reference to the performer entity (most likely
a model). The `<ref>` tag designates the name of the performer entity. It is
a required tag and there can only be one inside a `<performer>`. Multiple
`<performer>`s cannot point to the same entity.

In addition, the `<performer>` tag contains information about the volume
occupied by the performer. This volume is specified by the `<geometry>` tag.
Only the `<box>` tag is currently supported.

\note The volume for a performer may be automatically generated in future
versions of Gazebo.

Example snippet:

```xml
<performer name="perf1">
  <ref>robot1</ref>
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
</performer>
```

### Runtime performers

Performers can be specified at runtime using a Gazebo Transport service.
This functionality can be used when a performer is not known at load time. For
example, you may need to start simulation with an empty world and spawn
models (performers) into simulation at a later time.

The name of the add performer service is
`/world/<world_name>/level/set_performer`. Make sure to replace
`<world_name>` with the name of simulated world. The service request is an
gz:msgs::StringMsg message, and the response is an
gz::msgs::Boolean message. The response is true when the peformer was
successfuly added.

#### Example

1. Run the `levels_no_performer.sdf` world in a terminal.

```
gz sim levels_no_performers.sdf -v 4 --levels
```

Here you will see the two vehicles, which are regular models that do not trigger level loading. They are not performers until you call the service.

2. In another terminal call the add performer service for the blue vehicle.

```
gz service -s /world/levels/level/set_performer --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: "vehicle_blue"'
```

### Example

The following is a world file that could be an instance of the world shown in
the figure

<img src="https://raw.githubusercontent.com/gazebosim/gz-sim/d62080da95edbb172c47eac883ec4b707b59bb38/doc/architecture_design/06.png"/>

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
<world name="default">
  <model name="M1">
    <static>1</static>
    <pose>-8 8 0 0 0 0</pose>
    <!-- other links -->
  </model>
  <model name="M2">
    <static>1</static>
    <pose>8 5 0 0 0 0</pose>
    <!-- other links -->
  </model>
  <model name="M3">
    <static>1</static>
    <pose>0 0 0 0 0 0</pose>
    <!-- other links -->
  </model>
  <model name="M4">
    <static>1</static>
    <pose>-8 -8 0 0 0 0</pose>
    <!-- other links -->
  </model>
  <model name="M5">
    <static>1</static>
    <pose>-5 -5 0 0 0 0</pose>
    <!-- other links -->
  </model>
  <model name="M6">
    <static>1</static>
    <pose>-12 -8 0 0 0 0</pose>
    <!-- other links -->
  </model>

  <model name="R1">
    <pose>-5 5 0 0 0 0</pose>
    <!-- other links and joints-->
  </model>
  <model name="R2">
    <pose>-5 8 0 0 0 0</pose>
    <!-- other links and joints-->
  </model>
  <model name="R3">
    <pose>5 2 0 0 0 0</pose>
    <!-- other links and joints-->
  </model>

  <plugin name="gz::sim" filename="dummy">
    <performer name="perf1">
      <ref>R1</ref>
      <geometry>
        <box>
          <size>2 2 2</size>
        </box>
      </geometry>
    </performer>
    <performer name="perf2">
      <ref>R2</ref>
      <geometry>
        <box>
          <size>2 2 2</size>
        </box>
      </geometry>
    </performer>
    <performer name="perf3">
      <ref>R3</ref>
      <geometry>
        <box>
          <size>2 2 2</size>
        </box>
      </geometry>
    </performer>

    <level name="L1">
      <pose>-5 5 5 0 0 0</pose>
      <geometry>
        <box>
          <size>10 10 10</size>
        </box>
      </geometry>
      <buffer>2</buffer>
      <ref>M1</ref>
      <ref>M3</ref>
    </level>
    <level name="L2">
      <pose>5 5 5 0 0 0</pose>
      <geometry>
        <box>
          <size>10 10 10</size>
        </box>
      </geometry>
      <buffer>2</buffer>
      <ref>M2</ref>
      <ref>M3</ref>
    </level>
    <level name="L3">
      <pose>-5 -5 5 0 0 0</pose>
      <geometry>
        <box>
          <size>10 10 10</size>
        </box>
      </geometry>
      <buffer>2</buffer>
      <ref>M3</ref>
      <ref>M4</ref>
      <ref>M5</ref>
    </level>
  </plugin>
</world>
</sdf>
```
