\page server_config Server Configuration

Most functionality on Gazebo is provided by plugins, which means that
users can choose exactly what functionality is available to their simulations.
Even running the physics engine is optional. This gives users great control
and makes sure only what's crucial for a given simulation is loaded.

This tutorial will go over how to specify what system plugins to be loaded for
a simulation.

## How to load plugins

There are a few places where the plugins can be defined:

1. `<plugin>` elements inside an SDF file.
2. File path defined by the `GZ_SIM_SERVER_CONFIG_PATH` environment variable.
3. The default configuration file at `$HOME/.gz/sim/<#>/server.config` \*,
   where `<#>` is Gazebo Sim's major version.

Each of the items above takes precedence over the ones below it. For example,
if a the SDF file has any `<plugin>` elements, then the
`GZ_SIM_SERVER_CONFIG_PATH` variable is ignored. And the default configuration
file is only loaded if no plugins are passed through the SDF file or the
environment variable.

> \* For log-playback, the default file is
> `$HOME/.gz/sim/<#>/playback_server.config`

## Try it out

### Default configuration

Let's try this in practice. First, let's open Gazebo without passing
any arguments:

`gz sim`

You should see an empty world with several systems loaded by default, such as
physics, the scene broadcaster (which keeps the GUI updated), and the system that
handles user commands like translating models. Try for example inserting a simple
shape into simulation and pressing "play":

* the shape is inserted correctly because the user commands system is loaded;
* the shape falls due to gravity because the physics system is loaded;
* and you can see the shape falling through the GUI because the scene
broadcaster is loaded.

@image html files/server_config/default_server.gif

By default, you're loading this file:

`$HOME/.gz/sim/<#>/server.config`

That file is created the first time you load Gazebo. Once it is
created, Gazebo will never write to it again unless you delete it. This
means that you can customize it with your preferences and they will be applied
every time Gazebo is started!

Let's try customizing it:

1. Open this file with your favorite editor:

    `$HOME/.gz/sim/<#>/server.config`

2. Remove the `<plugin>` block for the physics system

3. Reload Gazebo:

    `gz sim`

Now insert a shape and press play: it shouldn't fall because physics wasn't
loaded.

@image html files/server_config/modified_default_config.gif

You'll often want to restore default settings or to use the latest default
provided by Gazebo (when you update to a newer version for example). In
that case, just delete that file, and the next time Gazebo is started a new file
will be created with default values:

`rm $HOME/.gz/sim/<#>/server.config`

### SDF

Let's try overriding the default configuration from an SDF file. Open your
favorite editor and save this file as `fuel_preview.sdf`:

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="fuel_preview">
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>

    <gui fullscreen="0">

      <!-- 3D scene -->
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>1.0 1.0 1.0</ambient_light>
        <background_color>0.4 0.6 1.0</background_color>
        <camera_pose>8.3 7 7.8 0 0.5 -2.4</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>

    </gui>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone</uri>
    </include>

  </world>
</sdf>
```

Now let's load this world:

`gz sim -r <path to>/fuel_preview.sdf`

Notice how the application has only one system plugin loaded, the scene
broadcaster, as defined on the SDF file above. Physics is not loaded, so even
though the simulation is running (started with `-r`), the cone doesn't fall
with gravity.

@image html files/server_config/from_sdf.png

If you delete the `<plugin>` element from the file above and reload it, you'll
see the same model loaded with the default plugins, so it will fall.

@image html files/server_config/from_sdf_no_plugins.gif

### Environment variable

It's often inconvenient to embed your plugins directly into every SDF file.
But you also don't want to be editing the default config file every time you
want to start with a different set of plugins. That's why Gazebo also supports
loading configuration files from an environment variable.

Let's start by saving this simple world with a camera sensor as
`simple_camera.sdf`:

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_camera">

    <gui fullscreen="0">

      <!-- 3D scene -->
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>

        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>1.0 1.0 1.0</ambient_light>
        <background_color>0.4 0.6 1.0</background_color>
        <camera_pose>8.3 7 7.8 0 0.5 -2.4</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz-gui>
      </plugin>

      <plugin filename="ImageDisplay" name="Image Display">
        <gz-gui>
          <property key="state" type="string">floating</property>
        </gz-gui>
      </plugin>

    </gui>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <include>
      <pose>0 0 1 0 0 0</pose>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Gazebo</uri>
    </include>

    <model name="camera">
      <static>true</static>
      <pose>20 0 1.0 0 0.0 3.14</pose>
      <link name="link">
        <pose>0.05 0.05 0.05 0 0 0</pose>
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
              <width>320</width>
              <height>240</height>
            </image>
          </camera>
          <update_rate>30</update_rate>
        </sensor>
      </link>
    </model>

  </world>
</sdf>
```

Then load the `simple_camera.sdf` world:

`gz sim -r <path to>/simple_camera.sdf`

You'll see a world with a camera and a cone. If you refresh the image display
plugin, it won't show any image topics. That's because the default server
configuration doesn't include the sensors system, which is necessary for
rendering-based sensors to generate data.

@image html files/server_config/camera_no_env.gif

Now let's create a custom configuration file in
`$HOME/.gz/sim/rendering_sensors_server.config` that has the sensors
system:

```
<server_config>
  <plugins>
    <plugin entity_name="*"
            entity_type="world"
            filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin entity_name="*"
            entity_type="world"
            filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
  </plugins>
</server_config>
```

And point the environment variable to that file:

`export GZ_SIM_SERVER_CONFIG_PATH=$HOME/.gz/sim/rendering_sensors_server.config`

Now when we launch the simulation again, refreshing the image display will
show the camera topic, and we can see the camera data.
One interesting thing to notice is that on the camera view, there's no grid and
the background color is the default grey, instead of the blue color set on the
GUI `GzScene` plugin.

@image html files/server_config/camera_env.gif
