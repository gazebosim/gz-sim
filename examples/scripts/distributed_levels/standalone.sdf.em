<?xml version="1.0" ?>

<!--
  Generated from distributed_levels.erb.sdf

  - primary: true
  - secondary: true

-->
<sdf version="1.6">
  <world name="default">


    <plugin
     filename="ignition-gazebo-physics-system"
     name="ignition::gazebo::systems::Physics">
    </plugin>



    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>

    <gui fullscreen="0">

      <!-- 3D scene -->
      <plugin filename="GzScene3D" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>

        <engine>ogre</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
      </plugin>

      <!-- World control -->
      <plugin filename="WorldControl" name="World control">
        <ignition-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>true</start_paused>
        <service>/world/default/control</service>
        <stats_topic>/world/default/stats</stats_topic>

      </plugin>

      <!--World statistics -->
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="double" key="z">1</property>

          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </ignition-gui>

        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
        <topic>/world/default/stats</topic>

      </plugin>

    </gui>


    <scene>
      <ambient>0.8 0.8 0.8 1.0</ambient>
      <background>0.34 0.39 0.43 1.0</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
@
@{
blocks_t = """
    <include>
      <name>explorer_{name}</name>
      <pose>0 {y} 2.5 0 0 0</pose>
      <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/EXPLORER_X1_SENSOR_CONFIG_2</uri>
    </include>

    <model name="block_0_{name}">
      <static>true</static>
      <pose>0.0 {y} -0.0 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
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

    <model name="level_vis_block_0_{name}">
      <static>true</static>
      <pose>0.0 {y} -0.0 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 2.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                4.0
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="block_1_{name}">
      <static>true</static>
      <pose>2.598076211353316 {y} -1.4999999999999998 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
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

    <model name="level_vis_block_1_{name}">
      <static>true</static>
      <pose>2.598076211353316 {y} -1.4999999999999998 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 2.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                4.0
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="block_2_{name}">
      <static>true</static>
      <pose>5.196152422706632 {y} -2.9999999999999996 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
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

    <model name="level_vis_block_2_{name}">
      <static>true</static>
      <pose>5.196152422706632 {y} -2.9999999999999996 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 2.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                4.0
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="block_4_{name}">
      <static>true</static>
      <pose>10.392304845413264 {y} -5.999999999999999 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
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

    <model name="level_vis_block_4_{name}">
      <static>true</static>
      <pose>10.392304845413264 {y} -5.999999999999999 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 2.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                4.0
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="block_5_{name}">
      <static>true</static>
      <pose>12.99038105676658 {y} -7.499999999999999 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 2.0 0.1</size>
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

    <model name="level_vis_block_5_{name}">
      <static>true</static>
      <pose>12.99038105676658 {y} -7.499999999999999 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 2.0 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                4.0
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="end_block_{name}">
      <static>true</static>
      <pose>
        14.289419162443238
        {y}
        -8.249999999999998
        0
        0
        0
      </pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 5.0 4.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 2.0 4.0</size>
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
"""

import sys
import itertools
import math

n_robots = int(sys.argv[-1]) if (len(sys.argv) > 2) else 2
y_offsets = [math.floor((2*n_robots-1)/2) *(-1.5) + 3. * i for i in range(n_robots)]
names = [chr(ord('A') + i) for i in range(n_robots)]
}@
@[for name, y in zip(names, y_offsets)]@
@blocks_t.format(name=name, y=y)@
@[end for]@
@{
block_3_t = f"""
    <model name="block_3">
      <static>true</static>
      <pose>7.794228634059948 0 -4.499999999999999 0 0.5235987755982988 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3.0 {3.0 * n_robots} 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3.0 {3.0 * n_robots} 0.1</size>
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

    <model name="level_vis_block_3">
      <static>true</static>
      <pose>7.794228634059948 0 -4.499999999999999 0 0 0</pose>
      <link name="link">
        <visual name="visual_level">
          <geometry>
            <box>
              <size>3.0 {3.0 * n_robots} 2.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 0.8 0.2</ambient>
            <diffuse>0 0 0.8 0.2</diffuse>
            <specular>0 0 0.8 0.2</specular>
          </material>
        </visual>
        <visual name="visual_buffer">
          <geometry>
            <box>
              <size>
                5.0
                {3.0 * n_robots + 2.0}
                4.0
              </size>
            </box>
          </geometry>
          <material>
            <ambient>0 0.8 0.8 0.2</ambient>
            <diffuse>0 0.8 0.8 0.2</diffuse>
            <specular>0 0.8 0.8 0.2</specular>
          </material>
        </visual>
      </link>
    </model>
"""
}@
@(block_3_t)
@
    <plugin name="ignition::gazebo" filename="dummy">

@[for name in names]@
@(f"""\
      <performer name="perf_explorer_{name}">
        <ref>explorer_{name}</ref>
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </performer>
""")
@[end for]@
@
@[for name, y in zip(names, y_offsets)]@
@(f"""\
    <level name="level_block_0_{name}">
      <pose>0.0 {y} -0.0 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 2.0 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_0_{name}</ref>
    </level>

    <level name="level_block_1_{name}">
      <pose>2.598076211353316 {y} -1.4999999999999998 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 2.0 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_1_{name}</ref>
    </level>

    <level name="level_block_2_{name}">
      <pose>5.196152422706632 {y} -2.9999999999999996 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 2.0 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_2_{name}</ref>
    </level>

    <level name="level_block_4_{name}">
      <pose>10.392304845413264 {y} -5.999999999999999 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 2.0 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_4_{name}</ref>
    </level>

    <level name="level_block_5_{name}">
      <pose>12.99038105676658 {y} -7.499999999999999 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 2.0 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_5_{name}</ref>
    </level>
""")
@[end for]@
@
@(
f"""\
    <level name="level_block_3">
      <pose>7.794228634059948 0 -4.499999999999999 0 0 0</pose>
      <geometry>
        <box>
          <size>3.0 {n_robots * 2.5} 2.0</size>
        </box>
      </geometry>
      <buffer>1.0</buffer>
      <ref>block_3</ref>
    </level>
"""
)
    </plugin>

  </world>
</sdf>
