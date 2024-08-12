\page migrationsdf Migration from Gazebo classic: SDF

Both Gazebo classic and Gazebo support [SDF](http://sdformat.org/)
files to describe the simulation to be loaded. An SDF file defines the world
environment, the robot's characteristics and what plugins to load.

Despite using the same description format, users will note that the same SDF
file may behave differently for each simulator. This tutorial will
explain how to write SDF files in a way that they're as reusable by both
simulators as possible. It will also explain when you'll need to use separate
files for each simulator.

The minimum required versions to use this guide are:

* Gazebo 11.2.0
* Igniton Citadel

## URIs

SDF files use URIs to refer to resources from other files, such as meshes and
nested models. These are some of the SDF tags that take URIs:

* `<include><uri>`
* `<mesh><uri>`
* `<material><pbr><...><*_map>` (only on Gazebo)
* `<actor><skin><filename>`
* `<actor><animation><filename>`

Here are the recommended ways to use URIs from most recommended to least:

### Gazebo Fuel URL

It's possible to use URLs of resources on
[Gazebo Fuel](https://app.gazebosim.org) within any of the tags
above and both simulators will be able to load it.

For example, this world can be loaded into both simulators:

```
<sdf version="1.7">
  <world name="demo">
    <!-- Included light -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- Included model -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <model name="Radio">
      <pose>3 -1.5 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <!-- Collision mesh -->
            <mesh>
              <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Radio/4/files/meshes/Radio.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <!-- Visual mesh -->
            <mesh>
              <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Radio/4/files/meshes/Radio.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <actor name="actor_talking">
      <skin>
        <filename>https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor - relative paths/tip/files/meshes/talk_b.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="talk_b">
        <filename>https://fuel.gazebosim.org/1.0/OpenRobotics/models/actor - relative paths/tip/files/meshes/talk_b.dae</filename>
        <scale>1.0</scale>
      </animation>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="talk_b">
          <waypoint>
            <time>0</time>
            <pose>2 -2 0.5 0 0 0</pose>
          </waypoint>
          <waypoint>
            <time>5</time>
            <pose>2 -2 0.5 0 0 0</pose>
          </waypoint>
        </trajectory>
      </script>
    </actor>

  </world>
</sdf>
```

\note The actor's vertical pose will be different on both simulators.
      That's because a hardcoded offset was removed on Gazebo and
      maintained on Gazebo classic for backwards compatibility.

### Path relative to the SDF file

It's possible to use relative paths within SDF files to refer to other files
in the same directory. This is recommended when creating files that work
together and need to be relocatable.

For example, consider the following directory structure:

```
/home/username/
├── world.sdf
└── models
    └── model1
        ├── model.sdf
        └── meshes
            └── mesh.dae
```

The world `world.sdf` can include `model1` as follows:

`<include>models/model1</include>`

And `model.sdf` can refer to `mesh.dae` as:

`<uri>meshes/mesh.dae</uri>`

### Path relative to an environment variable

This method is useful if you don't know where the files will be located
with respect to each other, and you have some control over the simulation's
runtime environment. On either simulator, you can refer to resources relative
to paths set on an environment variable.

Each simulator uses a different environment variable:

* Gazebo classic:
    * `GAZEBO_MODEL_PATH` for models
    * `GAZEBO_RESOURCE_PATH` for worlds and some rendering resources
* Gazebo:
    * `GZ_SIM_RESOURCE_PATH` for worlds, models and other resources

For example, if you have the file structure above, you can set the environment
variable to `/home/username/models`:

```
export GAZEBO_MODEL_PATH=/home/username/models
export GAZEBO_RESOURCE_PATH=/home/username/models
export GZ_SIM_RESOURCE_PATH=/home/username/models
```

And inside `world.sdf` include the model with:

`<include>model://model1</include>`

Also, `model.sdf` can refer to `mesh.dae` as:

`<uri>model://model1/meshes/mesh.dae</uri>`

On both situations, the `model://` prefix will be substituted by
`/home/username/models`.

You can also set several lookup paths separating them with `:`, for example:

`export GZ_SIM_RESOURCE_PATH=/home/username/models:/home/username/another_project/models`

### Absolute paths

Finally, both simulators will accept absolute paths, for example:

`<include>/home/username/models/model1</include>`

This method is not recommended, because the file most likely won't work if
copied to a different computer. It also becomes inconvenient to move files to
different directories.

## Plugins

Plugins are binary files compiled to use with a specific simulator. Plugins
for Gazebo classic and Gazebo aren't usually compatible, so plugins
will need to be specified for each simulator separately.

It's important to note that for both simulators, plugins compiled against
a major version of the simulator can't be loaded by other major versions.
The shared libraries will usually have the same name, so it's important to make
sure you're loading the correct plugins.

### Official plugins

Both simulators are installed with several built-in plugins.
[Gazebo classic's plugins](https://github.com/osrf/gazebo/tree/gazebo11/plugins)
and
[Gazebo Sim's plugins](https://github.com/gazebosim/gz-sim/tree/main/src/systems)
have different file names. For example, to use Gazebo classic's differential drive
plugin, the user can refer to it as follows:

```
<model ...>
  <plugin filename="libDiffDrivePlugin.so" name="any_custom_name">
     ...
  </plugin>
</model>
```

On Gazebo, that would be:

```
<model ...>
   <plugin filename="gz-sim-diff-drive-system"
     name="gz::sim::systems::DiffDrive">
     ...
   </plugin>
</model>
```

Note that besides the different file name, Gazebo also requires the C++ class
to be defined in the `name` attribute.

Also keep in mind that plugins that offer similar functionality may accept
different parameters for each simulator. Be sure to check the documentation of
each plugin before using it.

### Custom plugins

To load custom plugins, users need to set environment variables to the directory
where that plugin is located. The variables are different for each simulator:

* Gazebo classic:
    * `GAZEBO_PLUGIN_PATH` for all plugin types.
* Gazebo:
    * `GZ_SIM_SYSTEM_PLUGIN_PATH` for Gazebo systems (world, model,
      sensor and visual plugins).
    * `GZ_GUI_PLUGIN_PATH` for GUI plugins.

### Keeping plugins separate

Trying to load a plugin from one simulator into the other will:

* Print an error message if the simulator can't find the plugin
* Potentially crash if the simulator can find the plugin and tries to load it

That's why it's recommended not to specify plugins for both simulators
side-by-side on the same file. Instead, keep separate files and inject the plugins as
needed.

There isn't a built-in mechanism on SDFormat to inject plugins into files yet,
but users can make use of templating tools like [ERB](erb_template.html)
and [xacro](http://wiki.ros.org/xacro) to generate SDF files with the correct plugins.

### Default plugins

Gazebo is more modular than Gazebo classic, so most features are optional.
For example, by default, Gazebo will load all the system plugins defined on
the `~/.gz/sim/<#>/server.config` file and all GUI plugins defined on the
`~/.gz/sim/<#>/gui.config` file. But the user can always remove plugins from
those files, or choose different ones by adding `<plugin>` tags to the SDF file.
(For more details, see the [Server configuration tutorial](server_config.html)
and the [GUI configuration tutorial](gui_config.html)).

This is important to keep in mind when migrating your SDF files, because files
that worked on Gazebo classic may need more plugins on Gazebo.

## Materials

Gazebo does not support Ogre material files like Classic does, because Gazebo
can be used with multiple rendering engines. Though, there is limited support for pre-defined
materials, but arbitrary materials defined (below) within a `<script>` aren't supported:

```
        <material>
          <script>
            <uri>model://number1/materials/scripts</uri>
            <uri>model://number1/materials/textures</uri>
            <name>Number/One</name>
          </script>
        </material>
```

### Plain colors

To ease migration, Gazebo automatically parses plain solid colors defined in this material file
[gazebo.material](https://github.com/osrf/gazebo/blob/gazebo11/media/materials/scripts/gazebo.material),
if encountered like:

```
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
```

But this automatic parsing capability is limited to plain solid colors, other colors would be
rendered with a default <material> value. Eventually, to make your models compatible with both
 simulators, such material defining plain colors should be updated to use the `<ambient>`, `<specular>`,
`<emissive>` and `<diffuse>` tags as needed:

```
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>

```

### Textures

If an Ogre material script is being used to define a texture, there are a
couple alternatives.

If using mesh files, the texture can be embedded into it. The advantage is that
this works for both simulators. Some examples:

* [OBJ + MTL](https://app.gazebosim.org/OpenRobotics/fuel/models/DeskChair)
* [COLLADA](https://app.gazebosim.org/OpenRobotics/fuel/models/Lamp%20Post)

For primitive shapes or even meshes, you can pass the texture as the albedo map. If you
want the model to be compatible with both Classic and Gazebo, you can specify both
the script and the albedo map.

```
        <material>
          <pbr>
            <metal>
              <albedo_map>texture.png</albedo_map>
            </metal>
          </pbr>
        </material>
```
