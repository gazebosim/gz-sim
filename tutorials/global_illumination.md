\page global_illumination Global Illumination

This tutorial showcases how to enable real-time global illumination in Gazebo.

<div style="text-align:center;">
    \image html files/global_illumination/global_illumination.gif width=60%
</div>

## About global illumination

Global illumination (GI) techniques account for illumination from indirect light reflections. When GI is enabled, objects not only receive direct light from light sources, but also from indirect light bouncing off of other surfaces, ensuring nice reflections and a more natural appearance.

Gazebo supports two GI methods through the Ogre2 rendering engine, Voxel Cone Tracing (VCT) and Cascaded Image Voxel Cone Tracing (CI VCT).

### Voxel Cone Tracing

A scene rendered with VCT is reliable and good quality, and thus is the best choice for most scenes.

VCT parameters include:
* **Resolution**
* **Octant count**
* **Bounce count**
* **High quality**
* **Anisotropic**
* **Thin wall counter**
* **Conserve memory**
* **Debug visualization mode**

### Cascaded Image Voxel Cone Tracing

Compared to VCT, a scene rendered with CI VCT is slightly lower quality, but the scene can be re-voxelized every frame, so it’s much faster when dealing with very large scenes. However, it is more memory-intensive than VCT.

CI VCT parameters include:
* **Bounce count**
* **High quality**
* **Anisotropic**
* **Debug visualization mode**
* **Cascade**
    * **Resolution**
    * **Octant count**
    * **Thin wall counter**
    * **Area half size**

## Enabling global illumination in Gazebo

During simulation, the GUI and the camera sensor view are two different scenes. The GUI scene is rendered on the frontend client process, and the camera sensor scene is rendered on the backend server process. Thus, GI can be enabled on either or both of these processes.

> **Note:** GI solutions require hardware that uses OpenGL4. <br>
> **Note:** CI VCT must be run with Vulkan as the render engine API backend.

### How to enable global illumination for the GUI

GI can be enabled for the GUI through a GUI plugin. Both VCT and CI VCT are supported for the GUI.

#### Example usage for VCT

1) Open the [global_illumination.sdf](
https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds/global_illumination.sdf) world with

```bash
gz sim global_illumination.sdf
```
<div style="text-align:center;">
    \image html files/global_illumination/gui_demo_default.png width=60%
</div>
<br>

2) From the plugin dropdown, select the Global Illumination Vct plugin.

3) On the card, check the `Enabled` button to enable GI and change the parameter values as desired.

<div style="text-align:center;">
    \image html files/global_illumination/gui_demo_with_vct.png width=60%
</div>

#### Example usage for CI VCT

1) Open the [global_illumination.sdf](
https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds/global_illumination.sdf) world using Vulkan with

```bash
gz sim global_illumination.sdf --render-engine-api-backend vulkan
```

2) From the plugin dropdown, select the Global Illumination Ci Vct plugin.

3) On the card, check the `Enabled` button to enable GI and change the parameter values as desired.

### How to enable global illumination for the sensor

GI can be enabled for sensors through the `<global_illumination>` element in the Sensors System plugin. VCT is supported for the sensor.

#### Example usage with VCT

We will demonstrate how to enable VCT for the sensor with the SDF file below. (The finished SDF file can be viewed [here](
https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds/global_illumination.sdf).)

1) Save the below in an SDF file named `gi_demo.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
    <world name="gi_demo">
        <!--sensor plugins-->
        <plugin
        filename="gz-sim-physics-system"
        name="gz::sim::systems::Physics">
        </plugin>
        <plugin
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin
            filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands">
        </plugin>
        <plugin
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
        </plugin>

        <!--scene description for system view-->
        <scene>
            <ambient>0.4 0.4 0.4</ambient>
            <background>0 0 0</background>
        </scene>

        <!--GUI plugins-->
        <gui>
        <plugin filename="MinimalScene" name="3D View">
            <gz-gui>
                <title>Cornell box GI demo</title>
                <property type="bool" key="showTitleBar">true</property>
                <property type="string" key="state">docked</property>
            </gz-gui>
            <engine>ogre2</engine>
            <camera_pose>-12 0 4 0 0 0</camera_pose>
        </plugin>

        <plugin filename="ImageDisplay" name="Image Display">
            <topic>camera</topic>
        </plugin>

        <plugin filename="GzSceneManager" name="Scene Manager"/>
        <plugin filename="InteractiveViewControl" name="Interactive view control"/>

        <plugin filename="TransformControl" name="Transform control">
            <gz-gui>
                <property key="resizable" type="bool">false</property>
                <property key="x" type="double">0</property>
                <property key="y" type="double">50</property>
                <property key="width" type="double">250</property>
                <property key="height" type="double">50</property>
                <property key="state" type="string">docked</property>
                <property key="showTitleBar" type="bool">true</property>
                <property key="cardBackground" type="string">#777777</property>
            </gz-gui>
        </plugin>
        <plugin filename="SelectEntities" name="Select Entities">
            <gz-gui>
                <property key="resizable" type="bool">false</property>
                <property key="width" type="double">5</property>
                <property key="height" type="double">5</property>
                <property key="state" type="string">floating</property>
                <property key="showTitleBar" type="bool">false</property>
            </gz-gui>
        </plugin>
        </gui>

        <!--lighting-->
        <light type="point" name="ceiling_light">
        <pose>0 0 7.5 0 0 0</pose>
        <diffuse>1 1 1 1</diffuse>
        <specular>0.0 0.0 0.0 0</specular>
        <attenuation>
            <range>50</range>
            <linear>0</linear>
            <constant>0</constant>
            <quadratic>0.02</quadratic>
        </attenuation>
        <cast_shadows>true</cast_shadows>
        <visualize>false</visualize>
        <intensity>1.0</intensity>
        </light>

        <!--wall models-->
        <model name="wall_left">
        <pose>0 5 4 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <box>
                <size>10 1 10</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>10 1 10</size>
                </box>
            </geometry>
            <material>
                <ambient>1 0 0 1</ambient>
                <diffuse>1 0 0 1</diffuse>
                <specular>1 0 0 1</specular>
            </material>
            </visual>
        </link>
        </model>

        <model name="wall_right">
        <pose>0 -5 4 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <box>
                <size>10 1 10</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>10 1 10</size>
                </box>
            </geometry>
            <material>
                <ambient>0 1 0 1</ambient>
                <diffuse>0 1 0 1</diffuse>
                <specular>0 1 0 1</specular>
            </material>
            </visual>
        </link>
        </model>

        <model name="floor">
        <pose>0 0 -0.5 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <box>
                <size>10 10 1</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>10 10 1</size>
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

        <model name="ceiling">
        <pose>0 0 8.5 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <box>
                <size>10 10 1</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>10 10 1</size>
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

        <model name="wall_back">
        <pose>4.5 0 4 0 0 0</pose>
        <static>true</static>
        <link name="link">
            <collision name="collision">
            <geometry>
                <box>
                <size>1 10 10</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>1 10 10</size>
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

        <!--scene objects-->
        <model name="pump">
        <pose>-1.5 0 0 0 0 0</pose>
        <link name="pump_link">
            <collision name="pump_collision">
            <geometry>
                <mesh>
                    <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/meshes/pump.dae</uri>
                    <scale>3 3 3</scale>
                </mesh>
            </geometry>
            </collision>

            <visual name="pump_visual">
            <geometry>
                <mesh>
                    <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/meshes/pump.dae</uri>
                    <scale>3 3 3</scale>
                </mesh>
            </geometry>
            <material>
                <diffuse>1.0 1.0 1.0</diffuse>
                <specular>1.0 1.0 1.0</specular>
                <pbr>
                    <metal>
                    <albedo_map>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/materials/textures/pump_albedo.png</albedo_map>
                    <roughness_map>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/materials/textures/pump_roughness.png</roughness_map>
                    <metalness_map>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/materials/textures/pump_metallic.png</metalness_map>
                    <normal_map>https://fuel.gazebosim.org/1.0/openrobotics/models/pump/1/files/materials/textures/pump_normal.png</normal_map>
                    </metal>
                </pbr>
            </material>
            </visual>
        </link>
        </model>

        <!--camera sensor-->
        <model name="camera">
        <pose>-15 0 4 0 0.0 0</pose>
        <static>true</static>
        <link name="link">
            <pose>0.05 0.05 0.05 0 0 0</pose>
            <collision name="collision">
            <geometry>
                <box>
                <size>0.1 0.1 0.1</size>
                </box>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <box>
                <size>0.1 0.1 0.1</size>
                </box>
            </geometry>
            </visual>
            <sensor name="camera" type="camera">
            <camera>
                <horizontal_fov>1.047</horizontal_fov>
                <image>
                <width>2560</width>
                <height>1920</height>
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
        </model>
    </world>
</sdf>
```

2) Add the `<global_illumination>` element to the Sensors System plugin.

```xml
<global_illumination type="vct">
    <enabled>true</enabled>
    <resolution>16 16 16</resolution>
    <octant_count>1 1 1</octant_count>
    <bounce_count>6</bounce_count>
    <high_quality>true</high_quality>
    <anisotropic>true</anisotropic>
    <thin_wall_counter>1.0</thin_wall_counter>
    <conserve_memory>false</conserve_memory>
    <debug_vis_mode>none</debug_vis_mode>
</global_illumination>
```

The parameters can be changed from the default values.

3) Open the `gi_demo.sdf` world with

```bash
gz sim gi_demo.sdf
```
