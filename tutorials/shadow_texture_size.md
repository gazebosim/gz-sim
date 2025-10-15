\page shadow_texture_size Shadow Texture Size

This tutorial showcases how to change the texture size of shadows casted by lights.

<div style="text-align:center;">
  \image html files/shadow_texture_size/shadow_texsize_2k_to_16k.gif width=60%
</div>

## Texture size options

Supported texture sizes include: 512px, 1024px, 2048px, 4096px, 8192px, 16384px.

The default shadow texture size for all lights is 2K. The default max texture size is 16K, but for less powerful computers, it may be a lower value, like 8K.

## Changing shadow texture size

The shadow texture size can be changed for the GUI with the usage of the Minimal Scene GUI plugin in the SDF.

Currently, light type is restricted to only directional light. As a scene should typically have at most one directional light (representing the sun), this will ensure that an increase in shadow texture size does not increase VRAM too much.

### Example usage for the GUI

Let's change the shadow texture size for directional light with the SDF file below. (The finished SDF file can be viewed [here](https://github.com/gazebosim/gz-sim/tree/main/examples/worlds/shadow_texture_size.sdf).)

1) Save the below in an SDF file named `shadow_texture_size.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <scene>
      <background>0.1 0.1 0.1</background>
      <ambient>0.0 0.0 0.0</ambient>
    </scene>

    <!--GUI plugins-->
    <gui>
      <plugin filename="MinimalScene" name="3D View">
        <engine>ogre2</engine>
        <camera_pose>-10 0 7 0 0.5 0</camera_pose>
      </plugin>

      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
    </gui>

    <!--lighting-->
    <light type="directional" name="sun">
      <pose>0 0 8 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0 0 0 0</specular>
      <attenuation>
        <range>50</range>
        <constant>0</constant>
        <linear>0</linear>
        <quadratic>0</quadratic>
      </attenuation>
      <cast_shadows>true</cast_shadows>
      <direction>-2 2 -1.5</direction>
      <intensity>1.0</intensity>
    </light>

    <!--scene objects-->
    <include>
      <pose>0 0 0 0 0 -1.57</pose>
      <uri>
      https://fuel.gazebosim.org/1.0/OpenRobotics/models/Garden Mascot
      </uri>
    </include>

    <model name="floor">
    <pose>-5 0 -0.5 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <collision name="collision">
      <geometry>
        <box>
        <size>15 15 1</size>
        </box>
      </geometry>
      </collision>
      <visual name="visual">
      <geometry>
        <box>
        <size>15 15 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
        <specular>0.8 0.8 0.8 1</specular>
      </material>
      </visual>
    </link>
    </model>
  </world>
</sdf>
```

2) Add the following lines to the Minimal Scene GUI plugin.

```xml
<shadows>
  <texture_size light_type="directional">8192</texture_size>
</shadows>
```

The `<texture_size>` value can be changed as per the Texture size options section.

3) Open the `shadow_texsize_demo.sdf` world with

```bash
gz sim shadow_texture_size.sdf
```

## Impact on VRAM usage

Keep in mind that the larger the shadow texture size, the more VRAM is used. Thus, in a scene populated with many lights, changing the directional light's shadow texture size should be fine because there is typically only one directional light in a scene. But if the scene has many point lights and the point light's shadow texture size is increased, the VRAM consumption goes up significantly.
