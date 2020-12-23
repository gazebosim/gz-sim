\page light_config Light config

This tutorial gives an introduction to Ignition Gazebo's services `/world/`<world name>`/create` and
`/world/`<world name>`/light_config`. These services will allow to create entities and lights and modify
existing lights in the scene.

Ignition Gazebo creates these two services amongst other. You can check this typing:

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

To create new entities in the world we need to use the [ignition::msgs::EntityFactory](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1EntityFactory__V.html) message. This message allow us to create entities from
string, files, [Models](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Model.html),
[Lights](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Light.html) or even clone models.
This tutorial introduce how to create entities from strings and Lights.

## Include a model based on a string

As we described at the beginning of this tutorial, we have an empty Ignition Gazebo world,
let's start creating a sphere in the world. In the next snippet you can learn how to create models based on strings.

\snippet examples/standalone/light_control/light_control.cc create sphere

The variable `modelStr` will contain the SDF of the model that we want to create in the world. In this
case we are creating a sphere of 1 meter os radius in the position: `x: 0 y:0 z:0.5 roll: 0 pitch: 0 yaw: 0`.
Then we need to call the service `/world/`<world_name>`/create`:

\snippet examples/standalone/light_control/light_control.cc call service create sphere

**NOTE**: If the entity name does not exist then the entity will not be created in the world or if the entity already
exist then nothing will happens. You may see some traces in the console showing this information.

## Include a light

To include a light in the world we should use the field `light` inside the `ignition::msgs::EntityFactory` message.
In the following snippet you can see how the light's fields are filled.

Please check the API to know which fields are available for each type of light. There are three types of lights:
 - Point [API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1Light.html)
 - Directional [API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1DirectionalLight.html)
 - Spot [API](https://ignitionrobotics.org/api/rendering/4.1/classignition_1_1rendering_1_1SpotLight.html)

\snippet examples/standalone/light_control/light_control.cc create light

Finally we should call the same service `/world/`<world_name>`/create`:

# Modifying lights

To modify lights inside the scene we need to use the service `/world/`<world name>`/light_config` and
fill the message [`ignition::msgs::Light`](https://ignitionrobotics.org/api/msgs/6.0/classignition_1_1msgs_1_1Light.html).
In particular this example modify the point light that we have introduced in the section above.
As you can see in the following snippet we need to fill of the fields that corresponds to type of light in the scene.

\snippet examples/standalone/light_control/light_control.cc modify light

In the case we are creating random numbers to fill the diffuse and specular.
