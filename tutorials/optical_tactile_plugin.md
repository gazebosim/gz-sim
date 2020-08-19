\page opticaltactileplugin Optical Tactile Plugin

This plugin simulates an optical tactile sensor from a depth camera and a contact sensor. 
Given the increasing number of this kind of sensors and their use cases, this 
plugin is the first step towards data collection and sanity check of algorithms
involving this type of sensors.

Currently, the plugin is required to work within models that have one link, one
contact sensor and one depth camera. In order to simulate the behaviour of the
contact surface, which returns the contact forces of the object being touched,
we merge the information coming from these two different sensors. Contact normals to 
the surfaces being touched are computed from the depth image. (TODO pending ExtraContactData fields from https://github.com/ignitionrobotics/ign-physics/pull/40 are exposed in ign-gazebo) Next, these
values are merged with the information returned by the contact sensor, i.e. force 
magnitudes, penetration and depth.

The plugin allows the user to visualize the contact normals of the objects in 
the proximity of contact. In the next image, we can see these normals 
when we touch the ring and the top surface of a can of coke:

<img src="https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/master/tutorials/files/optical_tactile_plugin/coke_can.png"/>

## Components layout

As described in the previous section, there are several components that make up 
the optical tactile sensor. The following graphical description aims to help the
user get a better understanding of the different components of the plugin:

```
                               ___
                             /|   |
                            / |   |
                           /  |   |
                        _ /   |   |
 Depth camera ---------|_|    | ·-|----- Model's origin
 placed behind the        \   |   |
 contact surface           \  |   |
 (see the next section)     \ |   |
                             \|___|---  Contact surface: This is the collision
                                        geometry of the contact sensor. Forces
                                        returned by the sensor are inside this
                                        volume (check the <extended_sensing>
                                        parameter)
```

## Assumptions

The following assumptions have been made when working on this plugin:

- Currently, the plugin's model should be made up of one link, one contact
sensor and one depth camera.
- The geometry of the sensor must be a box, because the interpolation of the 
contacts returned assumes 4 points of input. The visualization of contacts 
and the `<extended_sensing>` parameter currently make this assumption as well.
Ideally, the size should be around 20x20x5 mm, but this can be changed. A bigger
one will also work, as long as the contact surface fits inside the camera's FOV.
- The depth camera should be displaced from the contact surface as in the 
previous sketch, so it's not placed inside the contact surface. This can
be done by setting the depth camera's `<pose>` accordingly. Check out the [example below](#try-it-out).
If the camera was placed inside the contact surface, objects could pass through
the camera from in front of the camera to behind it when the sensor approaches them.
Thus, information would be missing.
- There should not be a `<visual>` element inside the link, so that the link of 
the sensor does not block the depth camera's view. The contact surface can still
be visualized by setting the `<visualize_sensor>` parameter to true. For more 
information about the parameters, check the [parameters section](#parameters).

## Try it out

You can run [the example world](https://github.com/ignitionrobotics/ign-gazebo/tree/master/examples/worlds/optical_tactile_sensor_plugin.sdf) or build your own model following the example below. If the model
structure were to be modified, please take a look at the [assumptions](#assumptions) taken. 

```{.xml}
<model name="tactile_sensor">
  <pose>0 0 1 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.005 0.02 0.02</size>
        </box>
      </geometry>
    </collision>
    <sensor name="depth_camera" type="depth_camera">
      <update_rate>1</update_rate>
      <topic>depth_camera</topic>
      <pose relative_to="tactile_sensor">-0.05 0 0 0 0 0</pose>
      <camera>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R_FLOAT32</format>
        </image>
        <clip>
          <near>0.030</near>
          <far>0.065</far>
        </clip>
      </camera>
    </sensor>
    <sensor name="contact_sensor" type="contact">
      <contact>
        <collision>collision</collision>
      </contact>
    </sensor>

  </link>
  <static>true</static>
  <plugin
    filename="libignition-gazebo-opticaltactileplugin-system.so"
    name="ignition::gazebo::systems::OpticalTactilePlugin">
    <enabled>true</enabled>
    <visualization_resolution>15</visualization_resolution>
    <visualize_forces>true</visualize_forces>
    <visualize_sensor>true</visualize_sensor>
    <force_length>0.01</force_length>
    <extended_sensing>0.001</extended_sensing>
  </plugin>
</model>
```
You should be able to do something like this:

<img src="https://raw.githubusercontent.com/ignitionrobotics/ign-gazebo/master/tutorials/files/optical_tactile_plugin/example_world.gif"/>

## Parameters

### Plugin
These parameters are optional but can be specified in order to customize the
desired behaviour:

- `<enabled>`: (todo) Set this to true so the plugin works from the
start and doesn't need to be enabled. This element is optional, and the default
value is true.

- `<visualization_resolution>`: Number of pixels to skip when visualizing
forces. One vector representing a normal force is computed for each of the camera
pixels. This element must be positive and it is optional. The default
value is 30.

- `<namespace>`: Namespace for transport topics/services. If there are more
than one optical tactile plugins, their namespaces should be different
This element is optional, and the default value is "optical_tactile_sensor".
Check the ['Topics & services'](#Topics-&-services) section.

- `<visualize_forces>`: Set this to true so the plugin visualizes the normal
forces in the 3D world. This element is optional, and the
default value is false.

- `<force_length>`: Length in meters of the forces visualized if
`<visualize_forces>` is set to true. This parameter is optional and the
default value is 0.01.

- `<extended_sensing>`: Extended sensing distance in meters. The sensor will
output data coming from its collision geometry plus this distance. This
element is optional, and the default value is 0.001.

- `<visualize_sensor>`: Whether to visualize the sensor or not. This element
is optional, and the default value is false.

### Depth camera
There are some parameters within the depth camera that influence the way the
plugin works:

- `<update_rate>`: The 3D vectors normal to the surfaces being touched are
computed each time the camera returns an image. If `<visualize_forces>`
is set to `true` and this value is too high, it may slow down the simulation.

- `<clip>`: These values should be set in a way that the contact surface stays
in between its far and near values. Otherwise, nothing will be visualized.

## Topics & services
In order to access the data computed by the plugin and control it during
simulation, the following topics and services have been advertised:

  `/<namespace>/enable` : Service used to enable and disable the plugin. If disabled,
  the plugin won't publish messages in the `/normal_forces` nor the `/contacts` topics.

  `/<namespace>/normal_forces` : Topic where a message is published each
  time the normal forces are computed.

## Performance
In order to give an idea about the resources consumed by the plugin, we carried 
out some simple experiments by setting the parameters to specific values.
To run the experiments, we've used a Xiaomi Mi Air 13.3 with an Intel Core
i5-8250U quad-core 1.6GHz.

The main parameter that can affect the computational cost of running
the plugin is the visualization resolution. The following parameters 
have been tested for a sensor with a size of 20x20x5 mm:

- `<visualization_resolution>`: A value of 20 allows us to see a sufficient
number of forces and check what the sensor is looking at. However, if we
set the value to less than 5, performance may decrease.

| `<visualization_resolution>`  | No. of forces computed (not the same as visualized) | Worst RTF (%) |
| ------------- | ------------- | ------------- |
| 20 | 768 | 90 |
| 10 | 1376 | 84 |
| 5 | 3072 | 48 |
| 1 | 304964 | 35 |    

It's important to know how many plugins could be loaded before the RTF becomes
unacceptable. Of course, this depends on the parameters of each individual plugin.

In the following evaluation, they have been set as follows:

```
  <visualization_resolution>15</visualization_resolution>
  <visualize_forces>false</visualize_forces>
  <visualize_sensor>true</visualize_sensor>
  <force_length>0.01</force_length>
  <extended_sensing>0.001</extended_sensing>
```

We found the following RTFs:


| No. of sensors  | Worst RTF (%) |
| ------------- | ------------- |
| 1 | 88 |
| 3 | 80 |
| 5 | 77 |
| 7 | 69 |
| 10 | 39 |

## Future work

A more realistic behaviour of the current sensor like noise, 
drift and hysteresis, as well as the extraction of higher level physical 
properties by image processing can be added.
