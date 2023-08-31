\page erbtemplate ERB Template

This tutorial shows how to use ERB to generate simulation world files.
[ERB](https://docs.ruby-lang.org/en/2.3.0/ERB.html) is the commonly used templating language to generate structured text files with Ruby code.
In this case, since most simulation world used in gazebo is defined with [SDFormat](http://sdformat.org/), ERB works conveniently.

## Why ERB

There are many use cases and advantages of using ERB in your SDF file.
Some of them are listed below and demonstrated in this [example ERB file](https://github.com/osrf/srcsim/blob/master/worlds/unique.world.erb):

1. Embedding logic into the SDF, such as loops and conditionals
2. Full access to Ruby's math library, for simple things like PI,  to more elaborate ones like matrices and randomization
3. Breaking the SDF into multiple smaller files for better organization
4. Placing multiple instances of the same model into simulation world without manually copy-pasting every tag

## Set up Ruby

Firstly, Ruby needs to be installed.
If you have gone through [Gazebo Sim's installation guide](https://gazebosim.org/docs/latest/install), it's most likely you already have Ruby installed.
To check if Ruby is installed, use
```{.sh}
ruby --version
```
If it is not found, run the following to install Ruby

```{.sh}
sudo apt-get install -y ruby
```

## Create an ERB template

To make a distinction between ERB templates and normal sdformat files, `.erb` is commonly used as file suffix.
ERB language is usually embedded in the SDF file.
Below is a step-by-step tutorial to generate 1000 box shapes in a simulation world using ERB template.
You can copy-and-paste the code block into an editor to try it out.

First is to create a world using SDFormat syntax.
You need to specify both the `xml` and `sdf` versions you are using.
And don't forget to give your simulation world a name.

```
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="shapes">
    <!-- This is where you put `<plugin>`, `<model>` and etc. tags -->
  </world>
</sdf>
```

To create 1000 instances of simple box shapes, you can use a `for` loop in Ruby syntax in the ERB template.
This code block can be inserted in between the `<world>` tags.
Note that the `<model>` tags are wrapped in between the ERB template.
`<% end %>` is to mark the end of loops or `if` statements.
Each box model also has a different name and pose to ensure they show up as individuals in simulation.

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

[Here](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/shapes_population.sdf.erb) is a complete shapes simulation world example.

Instead of simple shapes, you can also use a nested loop to generate 100 actors spaced out evenly in a simulation world.

```
    <%
      # number of population
      total = 10
      for i in (0..total-1)
        for j in (1..total)
    %>

    <actor name="actor_<%= 10*i+j %>">
        <skin>
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
        <scale>1.0</scale>
        </skin>
        <animation name="talk_b">
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae</filename>
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

## Generate SDF from ERB template

Now that an ERB template file is ready and saved as `my_first_erb.erb`, you can run the following terminal command to generate the corresponding SDF file.

```{.sh}
# generate SDF with the ERB template
erb my_first_erb.erb > my_first_erb.sdf
```

## Run simulation world

To test if the ERB template works, run the SDF file with the `gz sim` command

```{.sh}
# run with Gazebo
gz sim my_first_erb.sdf
```

If there are any errors or warnings from running the SDF file, you would need to go back to the ERB file and see if any coding mistakes were made.
