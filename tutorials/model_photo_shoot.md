\page model_photo_shoot Model Photo Shoot

## Using the Model Photo Shoot plugin

Gazebo offers a model photo taking tool that will take perspective,
top, front, and both sides pictures of a model. You can test the demo world
in Gazebo, located at `examples/worlds/model_photo_shoot.sdf`, by
running the following command:

```
gz sim  -s -r -v 4 --iterations 50 model_photo_shoot.sdf
```

This will start Gazebo server, load the model and the plugin, take the
pictures and shutdown after 50 iterations. The pictures can be found at the
same location where the command was issued.

## Model Photo Shoot configurations

SDF is used to load and configure the `Model Photo Shoot` plugin. The demo SDF
contains a good example of the different options and other related plugins:

1. The physics plugin:

```
<plugin
  filename="gz-sim-physics-system"
  name="gz::sim::systems::Physics">
</plugin>
```

A physics plugin is needed only if the `<random_joints_pose>` option is to
be used. This will allow the `Model Photo Shoot` plugin to set the joints
to random positions.

2. The render engine plugin:

```
<plugin
  filename="gz-sim-sensors-system"
  name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
  <background_color>1, 1, 1</background_color>
</plugin>
```

A render plugin is needed to render the image. If `ogre2` is used, as shown in
the example, the `<backgrond_color>` tag can be used to set the background
of the pictures taken by the plugin. Please note that lights added by the
plugin will also affect the final resulting background color on the images.

3. The model and the photo shoot plugin:

```
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Robonaut</uri>
  <plugin
    filename="gz-sim-model-photo-shoot-system"
    name="gz::sim::systems::ModelPhotoShoot">
    <translation_data_file>poses.txt</translation_data_file>
    <random_joints_pose>true</random_joints_pose>
  </plugin>
</include>
```

The model is loaded through the `<uri>` tag. Then the `model photo shoot`
plugin and its options are specified:

* `<translation_data_file>`: (optional) Location to store the camera
translation, scaling data and joints position (if using the
`<random_joints_pose>` option) that can be used to replicate the
pictures using other systems.
* `<random_joints_pose>`: (optional) When set to `true` the joints in the model
will be set to random positions prior to taking the pictures.

4. Camera sensor:

```
<model name="photo_shoot">
  <link name="link">
    <pose>0 0 0 0 0 0</pose>
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>960</width>
          <height>540</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <topic>camera</topic>
    </sensor>
  </link>
  <static>true</static>
</model>
```

A `camera sensor` must be added as it will be used by the plugin to take the
pictures. This allows plugin users to set the different parameters of the
camera to their desired values.
