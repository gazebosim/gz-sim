\page jointcontrollers Joint Controllers

Gazebo provides three joint controller plugins which are `JointController`, `JointPositionController`, and `JointTrajectoryController`.

Let's see a detailed description of each of them and an example usage to help users select the right joint controller for their usage.

## 1) JointController

- Joint controller which can be attached to a model with a reference to a single joint.
- Currently, only the first axis of a joint can be actuated.

### Modes of JointController

1) Velocity mode:
This mode lets the user control the desired joint velocity directly.

2) Force mode:
A user who wants to control joint velocity using a PID controller can use this mode.

**Note**: This force mode is for the user who is looking to manually tune PID gains for velocity control according to a specific use case (e.g. Custom models). For general testing purposes, velocity mode will give the best results.

All the parameters related to this controller can be found \ref gz::sim::systems::JointController "here".

The commanded velocity(cmd_vel) can be published or subscribed at the topic: `/model/<model_name>/joint/<joint_name>/cmd_vel` by default.

Message data type: `Double`

### Example usage

Let's see an example for both modes using a simple model having only one joint.

For controlling joints one would require adding the Gazebo's joint controller plugin to the existing `<model_name>.sdf` file.

1) Save the SDF file in the desired directory or create one

e.g.

```bash
mkdir gz_tutorial
cd gz_turtorial
```

2) In this tutorial we will be using the following SDF file (this is just a slight modification of the original `joint_controller.sdf` [example](https://github.com/gazebosim/gz-sim/blob/gz-sim7/examples/worlds/joint_controller.sdf)).

After changing the directory, name the SDF file as `example.sdf`

- SDF file:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="joint_controller_demo">
      <pose>0 0 0 0 0 0</pose>
      <link name="base_link">
        <pose>0.0 0.0 0.0 0 0 0</pose>
        <inertial>
          <inertia>
            <ixx>2.501</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.501</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertia>
          <mass>120.0</mass>
        </inertial>
        <visual name="base_visual">
          <pose>0.0 0.0 0.0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.5 0.01</size>
            </box>
          </geometry>
        </visual>
        <collision name="base_collision">
          <pose>0.0 0.0 0.0 0 0 0</pose>
          <geometry>
            <box>
              <size>0.5 0.5 0.01</size>
            </box>
          </geometry>
        </collision>
      </link>
      <link name="rotor">
        <pose>0.0 0.0 0.1 0 0 0</pose>
        <inertial>
          <inertia>
            <ixx>0.032</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.032</iyy>
            <iyz>0</iyz>
            <izz>0.00012</izz>
          </inertia>
          <mass>0.6</mass>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.25 0.1 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.8 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.25 0.1 0.05</size>
            </box>
          </geometry>
        </collision>
      </link>

      <joint name="world_fixed" type="fixed">
        <parent>world</parent>
        <child>base_link</child>
      </joint>

      <joint name="j1" type="revolute">
        <pose>0 0 -0.5 0 0 0</pose>
        <parent>base_link</parent>
        <child>rotor</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>
  </world>
</sdf>
```

3) Run the following command to launch the gazebo simulation:

```bash
gz sim -v 4 -r example.sdf
```

This is how the model will look:

<div style="text-align:center;">
  \image html files/joint_controllers/JointController.png width=50%
</div>

4) Now let's add the Gazebo JointController plugin to the SDF file. Add the following line to your file just before the tag `</model>`.

**Note**: All the plugins discussed here should be between `<model>` and `</model>` tags. Ideally just before the `</model>` tag for better readability.

- Velocity mode

```xml
<plugin
 filename="gz-sim-joint-controller-system"
 name="gz::sim::systems::JointController">
 <joint_name>j1</joint_name>
 <initial_velocity>1.0</initial_velocity>
</plugin>
```

The initial velocity is set to 1.0 rad/s.

<div style="text-align:center;">
  \image html files/joint_controllers/JointControllerVelMode1.gif width=50%
</div>

One can change the joint velocity by publishing a message on the topic `/model/joint_controller_demo/joint/j1/cmd_vel` or can change the topic name within the plugin.

To change the topic name add the following line before `</plugin>` tag in the SDF file.

```xml
<topic>topic_name</topic>
```

- Sending velocity commands

```bash
gz topic -t "/topic_name" -m gz.msgs.Double -p "data: 10.0"
```

<div style="text-align:center;">
  \image html files/joint_controllers/JointControllerVelMode2.gif width=50%
</div>

- Force mode

Replace the velocity mode plugin mentioned above with the following lines in the SDF file for force mode.

```xml
<plugin
 filename="gz-sim-joint-controller-system"
 name="gz::sim::systems::JointController">
 <joint_name>j1</joint_name>
 <use_force_commands>true</use_force_commands>
 <p_gain>0.2</p_gain>
 <i_gain>0.01</i_gain>
</plugin>
```

This would look almost the same as velocity mode if PID gains are tuned properly.

5) Checking Joint states.
Here the state of the joint is obtained using the Gazebo’s JointStatepublisher plugin. Please visit \ref gz::sim::systems::JointStatePublisher for more information.
- Add the following lines to the SDF file before `</model>` tag:
```xml
<plugin
 filename="gz-sim-joint-state-publisher-system"
 name="gz::sim::systems::JointStatePublisher">
 <joint_name>j1</joint_name>
</plugin>
```
- To check joint state.

```bash
gz topic -e -t /world/default/model/joint_controller_demo/joint_state
```

```bash
joint {
  name: "j1"
  id: 12
  parent: "base_link"
  child: "rotor"
  pose {
    position {
      z: -0.5
    }
    orientation {
      w: 1
    }
  }
  axis1 {
    xyz {
      z: 1
    }
    limit_lower: -inf
    limit_upper: inf
    position: 35.115896338490096
    velocity: 1.0000051832309742
  }
}
```

An example where p_gain was set to 2.0 and the joint controller failed to reach the desired velocity and behaved absurdly due to improper gains is shown below.

<div style="text-align:center;">
  \image html files/joint_controllers/JoinControllerForceMode.gif width=50%
</div>

```bash
joint {
  name: "j1"
  id: 12
  parent: "base_link"
  child: "rotor"
  pose {
    position {
      z: -0.5
    }
    orientation {
      w: 1
    }
  }
  axis1 {
    xyz {
      z: 1
    }
    limit_lower: -inf
    limit_upper: inf
    position: 44282.754868627489
    velocity: -2891.1685359866523
  }
}
```

## 2) JointPositionController

- Joint position controller which can be attached to a model with a reference to a single joint.
- One can mention the axis of the joint they want to control.

JointPositionController uses a PID controller to reach a desired joint position.

All the parameters related to this controller can be found \ref gz::sim::systems::JointPositionController "here".

Commanded position(cmd_pos) can be published or subscribed at the topic: `/model/<model_name>/joint/<joint_name>/<joint_index>/cmd_pos` by default.

Message data type: `Double`.

### Example usage:

For this let's use the previously discussed SDF file.

1) Replace the JointController plugin with the JointPositionController plugin in  SDF file.

```xml
<plugin
 filename="gz-sim-joint-position-controller-system"
 name="gz::sim::systems::JointPositionController">
 <joint_name>j1</joint_name>
 <topic>topic_name</topic>
 <p_gain>1</p_gain>
 <i_gain>0.1</i_gain>
 <d_gain>0.01</d_gain>
 <i_max>1</i_max>
 <i_min>-1</i_min>
 <cmd_max>1000</cmd_max>
 <cmd_min>-1000</cmd_min>
</plugin>
```

2) Sending joint position command.

```bash
gz topic -t "/topic_name" -m gz.msgs.Double -p "data: -1.0"
```

<div style="text-align:center;">
  \image html files/joint_controllers/JointPositionController.gif width=50%
</div>


3) Checking joint state.

```bash
gz topic -e -t /world/default/model/joint_controller_demo/joint_state
```

```bash
joint {
  name: "j1"
  id: 12
  parent: "base_link"
  child: "rotor"
  pose {
    position {
      z: -0.5
    }
    orientation {
      w: 1
    }
  }
  axis1 {
    xyz {
      z: 1
    }
    limit_lower: -inf
    limit_upper: inf
    position: 0.99999991907580654
    velocity: 8.1005154347602952e-09
  }
}
```

## 3) JointTrajectoryController.

- Joint trajectory controller, which can be attached to a model with reference to one or more 1-axis joints to follow a trajectory.

JointTrajectoryController lets’s user specify the required position, velocity, and effort with respect to time. For velocity and position, this controller uses a PID controller.

A detailed description and related parameter of JointTrajectoryController can be found \ref gz::sim::systems::JointTrajectoryController "here".

The trajectory message can be published or subscribed at `/model/${MODEL_NAME}/joint_trajectory` by default.

Message type: [`JointTrajectory`](https://gazebosim.org/api/msgs/7.2/classignition_1_1msgs_1_1JointTrajectory.html)

### Example usage:

Let’s set up a new model for this example. A two-linked manipulator arm which has a total of two joints to control ([`joint_trajectory_controller.sdf`](https://github.com/gazebosim/gz-sim/blob/gz-sim7/examples/worlds/joint_trajectory_controller.sdf) is the original example). Name it as `example2.sdf`.

- SDF file:

```xml
<?xml version="1.0"
<sdf version="1.6">
  <world name="default">
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <grid>false</grid>
    </scene>
    <!--              -->
    <!-- Illumination -->
    <!--              -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>5 5 5 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-1 -1 -1</direction>
    </light>

    <!--        -->
    <!-- Models -->
    <!--        -->
    <model name="background_plane">
      <pose>-0.1 0 0 0 1.5708 0</pose>
      <static>true</static>
      <link name="background_plane_link">
        <visual name="background_plane_visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5 5</size>
            </plane>
          </geometry>
          <material>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- RR robot (Red) - Position control -->
    <model name="RR_position_control">
      <pose>0 0 0 0 -3.14159 0</pose>
      <!-- Fix To World -->
      <joint name="RR_position_control_world" type="fixed">
        <parent>world</parent>
        <child>RR_position_control_link0</child>
      </joint>
      <!-- Links -->
      <link name="RR_position_control_link0">
        <collision name="RR_position_control_link0_collision_0">
          <geometry>
            <sphere>
              <radius>0.025</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="RR_position_control_link0_visual_0">
          <geometry>
            <sphere>
              <radius>0.025</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0.5 0.5 1</ambient>
            <diffuse>0 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
      <link name="RR_position_control_link1">
        <pose relative_to="RR_position_control_joint1">0 0 0.1 0 0 0</pose>
        <collision name="RR_position_control_link1_collision_0">
          <geometry>
            <cylinder>
              <radius>0.01</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <collision name="RR_position_control_link1_collision_1">
          <geometry>
            <sphere>
              <radius>0.0125</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="RR_position_control_link1_visual_0">
          <geometry>
            <cylinder>
              <radius>0.01</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0 0 1</ambient>
            <diffuse>0.8 0 0 1</diffuse>
            <specular>0.8 0 0 1</specular>
          </material>
        </visual>
        <visual name="RR_position_control_link1_visual_1">
          <pose relative_to="RR_position_control_joint1">0 0 0.2 0 0 0</pose>
          <geometry>
            <sphere>
              <radius>0.0125</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0003358</ixx>
            <iyy>0.0003358</iyy>
            <izz>0.000005</izz>
          </inertia>
        </inertial>
      </link>
      <link name="RR_position_control_link2">
        <pose relative_to="RR_position_control_joint2">0 0 0.1 0 0 0</pose>
        <collision name="RR_position_control_link2_collision">
          <geometry>
            <cylinder>
              <radius>0.01</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="RR_position_control_link2_visual">
          <geometry>
            <cylinder>
              <radius>0.01</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0 0 1</ambient>
            <diffuse>0.8 0 0 1</diffuse>
            <specular>0.8 0 0 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0003358</ixx>
            <iyy>0.0003358</iyy>
            <izz>0.000005</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Joints -->
      <joint name="RR_position_control_joint1" type="revolute">
        <pose relative_to="RR_position_control_link0">0 0 0 0 0 0</pose>
        <parent>RR_position_control_link0</parent>
        <child>RR_position_control_link1</child>
        <axis>
          <xyz>1 0 0</xyz>
          <dynamics>
            <damping>0.5</damping>
          </dynamics>
        </axis>
      </joint>
      <joint name="RR_position_control_joint2" type="revolute">
        <pose relative_to="RR_position_control_link1">0 0 0.1 0 0 0</pose>
        <parent>RR_position_control_link1</parent>
        <child>RR_position_control_link2</child>
        <axis>
          <xyz>1 0 0</xyz>
          <dynamics>
            <damping>0.25</damping>
          </dynamics>
        </axis>
      </joint>
      <!-- Controller -->
    </model>
  </world>
</sdf>
```

1) Launching gazebo simulation.

```bash
gz sim -v 4 -r example2.sdf
```

This is how the model will look:

<div style="text-align:center;">
  \image html files/joint_controllers/JointTrajectoryController.png width=50%
</div>

2) Adding the JointTrajectoryController plugin and let’s do position control for both joints.

Append the following lines before `</model>` tag in the SDF file.

```xml
 <plugin
 filename="gz-sim-joint-trajectory-controller-system"
 name="gz::sim::systems::JointTrajectoryController">
 <joint_name>RR_position_control_joint1</joint_name>
 <initial_position>0.7854</initial_position>
 <position_p_gain>20</position_p_gain>
 <position_i_gain>0.4</position_i_gain>
 <position_d_gain>1.0</position_d_gain>
 <position_i_min>-1</position_i_min>
 <position_i_max>1</position_i_max>
 <position_cmd_min>-20</position_cmd_min>
 <position_cmd_max>20</position_cmd_max>

 <joint_name>RR_position_control_joint2</joint_name>
 <initial_position>-1.5708</initial_position>
 <position_p_gain>10</position_p_gain>
 <position_i_gain>0.2</position_i_gain>
 <position_d_gain>0.5</position_d_gain>
 <position_i_min>-1</position_i_min>
 <position_i_max>1</position_i_max>
 <position_cmd_min>-10</position_cmd_min>
 <position_cmd_max>10</position_cmd_max>
</plugin>
```

3) Sending trajectory message (Can also change the topic name).

```bash
gz topic -t "topic_name" -m gz.msgs.JointTrajectory -p '
    joint_names: "RR_position_control_joint1"
    joint_names: "RR_position_control_joint2"
    points {
      positions: -0.7854
      positions: 1.5708
      time_from_start {
        sec: 0
        nsec: 250000000
      }
    }
    points {
      positions: -1.5708
      positions: 0
      time_from_start {
        sec: 0
        nsec: 500000000
      }
    }
    points {
      positions: -1.5708
      positions: -1.5708
      time_from_start {
        sec: 0
        nsec: 750000000
      }
    }
    points {
      positions: 0
      positions: 0
      time_from_start {
        sec: 1
        nsec: 0
      }
    }
```

<div style="text-align:center;">
  \image html files/joint_controllers/JointTrajectoryController.gif width=50%
</div>

**Note**: by default velocity and position control are disabled if one wants to use this mode, they must specify the PID gains value according to usage.

In case, PID gains are not specified then by default force mode will work.

4) Checking the progress of the commanded trajectory.

```bash
gz topic -e -t "/model/RR_position_control/joint_trajectory_progress"
```

This returns the progress of the commanded trajectory which is a value between (0,1].

Finally, JointTrajectoryController can be used for hybrid control (e.g. In manipulator robots) where value from position PID, velocity PID, and commanded force value are summed up and applied.
