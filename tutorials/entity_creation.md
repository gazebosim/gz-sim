\page entity_creation Entity creation

This tutorial gives an introduction to Gazebo Sim's service `/world/<world name>/create`.
This service allows creating entities in the scene such us spheres, lights, etc.

Gazebo creates many services depending on the plugins that are specified in the SDF.
In this case we need to load the `UserCommands` plugin, which will offer the `create` service.
You can include the `UserCommands` system plugin including these lines in your SDF:

```xml
<plugin
  filename="gz-sim-user-commands-system"
  name="gz::sim::systems::UserCommands">
</plugin>
```

You can check if this service is available typing:

In one terminal

```bash
gz sim -r -v 4 <your_world>.sdf
```

In another terminal, see if the create service is listed:

```bash
gz service --list
/gazebo/resource_paths/add
/gazebo/resource_paths/get
/gazebo/worlds
/gui/follow
/gui/move_to
/gui/move_to/pose
/gui/record_video
/gui/transform_mode
/gui/view_angle
/marker
/marker/list
/marker_array
/server_control
/world/world_name/control
/world/world_name/create
/world/world_name/create_multiple
/world/world_name/generate_world_sdf
/world/world_name/gui/info
/world/world_name/level/set_performer
/world/world_name/light_config
/world/world_name/playback/control
/world/world_name/remove
/world/world_name/scene/graph
/world/world_name/scene/info
/world/world_name/set_pose
/world/world_name/state
/world/world_name/state_async
```

# Factory message

To create new entities in the world we need to use the
[gz::msgs::EntityFactory](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1EntityFactory.html)
message to send a request to the create service.
This message allows us to create entities from strings, files,
[Models](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Model.html),
[Lights](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Light.html) or even clone models.
This tutorial introduces how to create entities from SDF strings and light messages.

## Insert an entity based on a string

We will open an empty Gazebo world, let's start creating a sphere in the world.
In the next snippet you can see how to create models based on strings.

\snippet examples/standalone/entity_creation/entity_creation.cc create sphere

The variable `sphereStr` contains the SDF of the model that we want to create in the world.
In this case we are creating a sphere of 1 meter of radius in the position: `x: 0 y: 0 z: 0.5 roll: 0 pitch: 0 yaw: 0`.

**NOTE**: You can insert here all kinds of models that can be described using an SDF string.

Then we need to call the service `/world/<world_name>/create`:

\snippet examples/standalone/entity_creation/entity_creation.cc call service create sphere

**NOTE**: By default, if the entity name does not exist then the entity will be created
in the world. On the other hand, if that entity name already exists, then nothing will
happen. You may see some traces in the console showing this information.
There is an option to create a new entity every time that the message is sent by setting
`allow_renaming` to true (you can use the method `set_allow_renaming()`).

## Insert a light

To insert a light in the world we have two options:

 - Fill the string inside the `gz::msgs::EntityFactory` message like in the section above.
 - Fill the field `light` inside the `gz::msgs::EntityFactory` message.

In the following snippet you can see how the light's field is filled.

\snippet examples/standalone/entity_creation/entity_creation.cc create light

Or we can create an SDF string:

\snippet examples/standalone/entity_creation/entity_creation.cc create light str

Please check the API to know which fields are available in the
[Light message](https://gazebosim.org/api/msgs/9/classgz_1_1msgs_1_1Light.html).
There are three types of lights: Point, Directional and Spot.

Finally we should call the same service `/world/<world_name>/create`:

\snippet examples/standalone/entity_creation/entity_creation.cc call service create
