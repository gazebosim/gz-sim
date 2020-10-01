\page migrationsdf

# Migration from Gazebo classic: SDF

Both Gazebo classic and Ignition Gazebo support [SDF](http://sdformat.org/)
files to describe the simulation to be loaded. The SDF file defines the world
environment, the robot's characteristics and what plugins to load.

Despite using the same description format, users will note that the same SDF
file may look and behave differently on each simulator. This tutorial will
explain how to write SDF files in a way that they're as reusable by both
simulators as possible.

The minimum required versions to use this guide are:

* Gazebo 11.2.0
* Igniton Citadel

## URIs

SDF files use URIs to refer to resources from other files, such as meshes and
nested models. These are some of the SDF tags that take URIs:

* `<include><uri>`
* `<mesh><uri>`
* `<material><pbr><...><*_map>` (only on Ignition)
* `<actor><skin><filename>`
* `<actor><animation><filename>`

Here are the recommended ways to use URIs from most recommended to least:

1. Ignition Fuel URL

    It's possible to use URLs to resources on
    [Ignition Fuel](https://app.ignitionrobotics.org) within any of the tags
    above and both simulators will be able to load it.

    For example, this world can be loaded into both simulators:

    ```
    <sdf version="1.7">
      <world name="demo">
        <!-- Included light -->
        <include>
          <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Sun</uri>
        </include>

        <!-- Included model -->
        <include>
          <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Ground Plane</uri>
        </include>

        <model name="Radio">
          <pose>3 -1.5 0 0 0 0</pose>
          <static>true</static>
          <link name="link">
            <collision name="collision">
              <geometry>
                <!-- Collision mesh -->
                <mesh>
                  <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Radio/4/files/meshes/Radio.dae</uri>
                </mesh>
              </geometry>
            </collision>
            <visual name="visual">
              <geometry>
                <!-- Visual mesh -->
                <mesh>
                  <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Radio/4/files/meshes/Radio.dae</uri>
                </mesh>
              </geometry>
            </visual>
          </link>
        </model>

        <actor name="actor_talking">
          <skin>
            <filename>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/actor - relative paths/tip/files/meshes/talk_b.dae</filename>
            <scale>1.0</scale>
          </skin>
          <animation name="talk_b">
            <filename>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/actor - relative paths/tip/files/meshes/talk_b.dae</filename>
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
          That's because a hardcoded offset was removed on Ignition and
          maintained on Gazebo classic for backwards compatibility.

2. Path relative to the SDF file

    * i.e. `meshes/mesh.dae`





    * i.e. `https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Gazebo`
    *
* Absolute path on disk
    * i.e. `/home/user/my_model/meshes/mesh.dae`
* Relative path from the running directory
* Relative path from an environment variable
    * i.e. `model://my_model/meshes/mesh.dae`


## Plugins



## Materials


