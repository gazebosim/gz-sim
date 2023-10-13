\page gui_config GUI Configuration

Gazebo Sim's graphical user interface is powered by
[Gazebo GUI](https://gazebosim.org/libs/gui). Therefore, Gazebo Sim's
GUI layout can be defined in
[Gazebo GUI configuration files](https://gazebosim.org/api/gui/8/config.html).
These are XML files that describe what plugins to be loaded and with what
settings.

## How to load different GUI configurations

There are a few places where the GUI configuration can come from:

1. A file passed to the `--gui-config` command line argument
2. A `<gui>` element inside an SDF file
3. The default configuration file at `$HOME/.gz/sim/<#>/gui.config` \*,
   where `<#>` is Gazebo Sim's major version.

Each of the items above takes precedence over the ones below it. For example,
if a user chooses a `--gui-config`, the SDF's `<gui>` element is ignored. And
the default configuration file is only loaded if no configuration is passed
through the command line or the SDF file.

> \* For log-playback, the default file is
> `$HOME/.gz/sim/<#>/playback_gui.config`

## Try it out

### Default configuration

Let's try this in practice. First, let's open Gazebo without passing
any arguments:

`gz sim`

You should see an empty world with several plugins loaded by default, such as the
3D Scene, the play/pause button, etc.

@image html files/gui_config/default_gui.png

By default, you're loading this file:

`$HOME/.gz/sim/<#>/gui.config`

That file is created the first time you load Gazebo. Once it is
created, Gazebo will never write to it again unless you delete it. This
means that you can customize it with your preferences and they will be applied
every time Gazebo is started!

Let's try customizing it:

1. Open this file with your favorite editor:

    `$HOME/.gz/sim/<#>/gui.config`

2. Change `material_theme` from `Light` to `Dark`

3. Reload Gazebo:

    `gz sim`

Note how the UI is now in dark mode!

@image html files/gui_config/dark_gui.png

You'll often want to restore default settings or to use the latest default
provided by Gazebo (when you update to a newer version for example). In
that case, just delete that file, and the next time Gazebo is started a new file
will be created with default values:

`rm $HOME/.gz/sim/<#>/gui.config`

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
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Gazebo</uri>
    </include>

  </world>
</sdf>
```

Now let's load this world:

`gz sim <path to>/fuel_preview.sdf`

Notice how the application has only one GUI plugin loaded, the 3D scene, as defined
on the SDF file above.

@image html files/gui_config/fuel_preview.png

If you delete the `<gui>` element from the file above and reload it, you'll see
the same model loaded into the default GUI layout.

@image html files/gui_config/fuel_preview_no_gui.png

### Command line

It's often inconvenient to embed your GUI layout directly into every SDF file.
But you also don't want to be editing the default config file every time you
want to start with a different layout. That's why Gazebo also supports loading
configuration files from the command line.

Let's start by creating a custom configuration file, but instead of editing by
hand, we'll create it from the UI.

1. Let's start loading the SDF world we created above, with the `<gui>` element back:

    `gz sim <path to>/fuel_preview.sdf`

2. Now from the top-right menu, choose to add the "View Angle" plugin. This
   plugin has convenient buttons to change the camera angle, try them out!

3. Undock the inserted plugin by clicking on the □  button on top of it.

4. Now that the plugin is floating, drag it to any place you want.

5. Finally, let's save our configuration by going to the top-left menu
   and choosing "Save client configuration as".

6. Save the file at a path of your choice, and name it `saved.config`.

7. Close Gazebo

@image html files/gui_config/save_config.gif

1. Take a look at the saved file if you're curious, it will look a lot like
   the default file, but with more properties defined.

2. Finally, let's load the previous world, with our custom configuration:

    `gz sim <path to>/fuel_preview.sdf --gui-config <path to>saved.config`

3. Gazebo should open with your custom layout.

**Tip**: From the top-left menu, you can choose "Save client configuration" to
save directly to `$HOME/.gz/sim/<#>/gui.config`.

@image html files/gui_config/cmd_line.png
