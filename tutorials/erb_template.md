\page erbtemplate ERB Template

This tutorial shows how to place a large number of entites in a simulation world using ERB. ERB is the commonly used templating language to generate structured text files with Ruby code. In this case, since most simulation world used in gazebo is defined with sdformat (XML documents), ERB works conveniently.

## Set up Ruby

Firstly, Ruby needs to be installed. If you have gone through [ignition libraries installation guide](https://ignitionrobotics.org/docs/latest/install), it's most likely you already have ruby installed. To check if Ruby is installed, use 
```{.sh}
ruby --version
```
If it is not found, run the follwing to install Ruby

```{.sh}
sudo apt-get install -y ruby
```

## Create an ERB template

To make a distinction between ERB templates and normal sdformat files, `.sdf.erb` is used as file suffix. ERB language is usually embedded in the sdformat file. Below is a simple example to generate 1000 shapes using ERB templating language. Ruby syntax is used in the template part.

```
    <%
      # number of population
      n = 1000
      for m in (0..n)
    %>

    <model name="box_<%= m.to_s %>">
      <pose><%= -750 + m*1.5 %> 0 0.5 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
          <mass>1.0</mass>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>

        <visual name="box_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
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
    <%
      end
    %>
```


You can also use a nested loop to generate 100 actors spaced out evenly in a simulation world. 

```
    <%
      # number of population
      total = 10
      for i in (0..total-1)
        for j in (1..total)
    %>

    <actor name="actor_<%= 10*i+j %>">
        <skin>
        <filename>https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
        <scale>1.0</scale>
        </skin>
        <animation name="talk_b">
        <filename>https://fuel.ignitionrobotics.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
        <scale>0.055</scale>
        <interpolate_x>true</interpolate_x>
        </animation>
        <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="<%= 10*i+j %>" type="talk_b">
            <waypoint>
            <time>0</time>
            <pose><%= -5 + 2*j %> <%= 5 + -2*i %> 1.0 0 0 0</pose>
            </waypoint>
            <waypoint>
            <time>30</time>
            <pose><%= -5 + 2*j %> <%= 5 + -2*i %> 1.0 0 0 0</pose>
            </waypoint>
        </trajectory>
        </script>
    </actor>

    <%
        end
      end
    %>
```
One thing to note is that each ``<model>`` tag must have their distinct names to show up in simulation as individual entities. 

[Here](https://github.com/ignitionrobotics/ign-gazebo/blob/ign-gazebo3/examples/worlds/shapes_population.sdf.erb) is a complete simulation world example.
 
## Generate SDF from ERB template

Now that a template file is ready, you can then use it to generate the sdf file which can be passed into Ignition Gazebo. Use the following command to generate the sdf file

```{.sh}
erb -T 1 FILENAME.sdf.erb > FILENAME.sdf
```

## Run simulation world

To test if the template worked as intended, run the sdf file with igniton command

```{.sh}
ign gazebo FILENAME.sdf
```