\page model_creation Model creation

This tutorial gives an introduction to the Ignition Gazebo's service `/world/<world name>/create`.
This service will allow to create models in the scene such us spheres, lights, etc.

Ignition Gazebo creates many services. You can check this typing:

In one terminal
```bash
ign gazebo -r -v 4 empty.world
```

In other terminal:
```bash
ign service --list
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
/world/empty/control
/world/empty/create
/world/empty/create_multiple
/world/empty/generate_world_sdf
/world/empty/gui/info
/world/empty/level/set_performer
/world/empty/light_config
/world/empty/playback/control
/world/empty/remove
/world/empty/scene/graph
/world/empty/scene/info
/world/empty/set_pose
/world/empty/state
/world/empty/state_async
```
# Including models and Lights

To create new entities in the world we need to use the [ignition::msgs::EntityFactory](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1EntityFactory__V.html) message.
This message allow us to create entities from string, files, [Models](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Model.html),
[Lights](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Light.html) or even clone models.
This tutorial introduces how to create entities from strings and light msgs.

## Include a model based on a string

We will open an empty Ignition Gazebo world, let's start creating a sphere in the world.
In the next snippet you can see how to create models based on strings.

\snippet examples/standalone/model_creation/model_creation.cc create sphere

The variable `modelStr` contains the SDF of the model that we want to create in the world.
In this case we are creating a sphere of 1 meter of radius in the position: `x: 0 y:0 z:0.5 roll: 0 pitch: 0 yaw: 0`.

**NOTE**: You can insert here all kind of models that you can described using a SDF string.

Then we need to call the service `/world/<world_name>/create`:

\snippet examples/standalone/model_creation/model_creation.cc call service create sphere

**NOTE**: If the entity name does not exist then the entity will be created in the world or if the entity already
exist then nothing will happens. You may see some traces in the console showing this information.

## Include a light

To include a light in the world we have two options:

 - Filled the string in the inside the `ignition::msgs::EntityFactory` message like in the section above.
 - Filled the field `light` inside the `ignition::msgs::EntityFactory` message.

In the following snippet you can see how the light's field is filled.

\snippet examples/standalone/model_creation/model_creation.cc create light

Or we can create a SDF string:

\snippet examples/standalone/model_creation/model_creation.cc create light str

Please check the API to know which fields are available for each type of light. There are three types of lights:
 - Point [rendering API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1Light.html) [light msgs API](https://ignitionrobotics.org/api/msgs/6.2/classignition_1_1msgs_1_1Light.html)
 - Directional [rendering API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1DirectionalLight.html)[light msgs API](https://ignitionrobotics.org/api/msgs/6.2/classignition_1_1msgs_1_1Light.html)
 - Spot [rendering API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1SpotLight.html)[light msgs API](https://ignitionrobotics.org/api/msgs/6.2/classignition_1_1msgs_1_1Light.html)

Finally we should call the same service `/world/<world_name>/create`:

\snippet examples/standalone/model_creation/model_creation.cc call service create
