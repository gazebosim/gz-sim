\page environmental_data Visuallizing Environment Data

Lot of times you may come accross environment properties that vary with time and
space. For instance if you'd like to model properties such as gas distribution
and water temperature one needs to have some way of defining values such that
the values vary through out the environment. Gazebo's Environment Data
infrastructure enables such visualization and sensing capabilities.

# Loading Environment Data

Currently gazebo is able to load data from CSV files and display them. It is
assumed the points are sampled at a fixed grid interval. For gaps in between the
sample points, Gazebo will interpolate the data.

## Data Format

As mentioned earlier, the data should be a CSV file with grid based spacing.
The first line of the CSV file should be the heading for each column.
There should be a column for time (in seconds), the x coordinate, y coordinate
and z coordinate (in meters). You may have multiple columns of data like so:
```
timestamp,humidity,x,y,z
0,0,-1,-1,-1
0,0,-1,-1, 1
0,0,-1, 1,-1
0,0,-1, 1, 1
0,0, 1,-1,-1
0,0, 1,-1, 1
0,0, 1, 1,-1
0,0, 1, 1, 1
1,1,-1,-1,-1
1,1,-1,-1, 1
1,1,-1, 1,-1
1,1,-1, 1, 1
1,1, 1,-1,-1
1,1, 1,-1, 1
1,1, 1, 1,-1
1,1, 1, 1, 1

```
For the x and y coordinates you may also use latitude and longitude in degrees.
Heading names don't matter. Order also doesn't matter.
```
timestamp,humidity,latitude,longitude,altitude
1658923062,91,36.80029505,-121.788972517,0.8
1658923062,88,36.80129505,-121.788972517,0.8
1658923062,89,36.80029505,-121.789972517,0.8
1658923062,92,36.80129505,-121.789972517,0.8
1658923063,90,36.80029505,-121.788972517,0.8
1658923063,85,36.80129505,-121.788972517,0.8
1658923063,87,36.80029505,-121.789972517,0.8
1658923063,94,36.80129505,-121.789972517,0.8
```
> :warning: Note that only one Environment Data file per world is allowed. Also,
Gazebo can only load data that fits into memory.

## Pre-Loading
If you are writing SDFormat files by hand or via a script you may want to
pre-load the data. You may use the `EnvironmentPreload` plugin for this. For
instance we could load the first file like so (by adding the environment to the
):
```xml
    <plugin
      filename="gz-sim-environment-preload-system"
      name="gz::sim::systems::EnvironmentPreload">
      <data>environmental_data.csv</data>
      <dimensions>
        <time>timestamp</time>
        <space>
          <x>x</x>
          <y>y</y>
          <z>z</z>
        </space>
      </dimensions>
    </plugin>
```
If you would like to use Latitude and Longitude you may set the `reference` to
`spherical` and the `units` to `degrees`.
```xml
    <plugin
      filename="gz-sim-environment-preload-system"
      name="gz::sim::systems::EnvironmentPreload">
      <data>environmental_data.csv</data>
      <dimensions>
        <time>timestamp</time>
        <space reference="spherical" units="degrees">
          <x>latitude</x>
          <y>longitude</y>
          <z>altitude</z>
        </space>
      </dimensions>
    </plugin>
```

## Via GUI
To load the data, click on the 3 dots and search for the "Environment Loader":
@image html files/environment_data/environment_loader.png

Click on it and you will see the environment loader:

@image html files/environment_data/environment_loader_options.png

Select the file you would want to load by clicking on the arrow next to the
field named "Data file path".

@image html files/environment_data/environment_file_loaded.png

Select the appropriate fields and then click "load".

@image html files/environment_data/environment_fields.png

The data is now loaded into the system.

# Visualizing the data

To visualize the data we will load 2 plugins. You must already have loaded the
data in. First we will search for "Environment Visualization" using the three
dots menu (second from top):

@image html files/environment_data/environment_viz_search.png

Click on it. You should see some sliders.

@image html files/environment_data/environment_viz_sliders.png

Next search for the "Point Cloud" tool.

@image html files/environment_data/environment_pc_search.png

You should see a panel like so:

@image html files/environment_data/environment_pc_panel.png

Hit the refresh button to find the pointcloud generated b y the visuallization
plugin. You can select the field you wish to view using the "Float vector"
field. Once thats done, you should be able to see the data.

@image html files/environment_data/environment_data_viz.png

# Using custom sensors to sense the data

Apart from just visuallization, there are custom sensors available for sampling
the data. So if you would like to have your robot search for a maxima for
instance, you would need a custom sensor. It is easy to add such a sensor.
Simply add a sensor like so (replace column name with your own):
```xml
<sensor name="custom_sensor" type="custom" gz:type="environmental_sensor/{column_name}">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```
By default it'll echo to a topic generated by the scoped name.
```bash
gz topic -e -t "/world/environmental_sensor_test/model/model_with_sensor/link/link/sensor/custom_sensor/environmental_sensor/humidity"
```
If you would like a better topic name you may always specify the topic like so
```
<sensor name="custom_sensor" type="custom" gz:type="environmental_sensor/{column_name}">
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <topic>humidity</topic>
</sensor>
```
Humidity should now be available on the `/humidity` topic.
```bash
gz topic -e -t "/humidity"
```
# Using C++ API to integrate with your own plugins

You may want to use environmental data to model your own plugins for instance,
you may want to apply a force according to a pre-calculated wind model. For such
use cases we provide the [`Environment`](https://github.com/gazebosim/gz-sim/blob/gz-sim7/include/gz/sim/components/Environment.hh) component. To make querying the component easier you may take a
look at [`this method`](https://github.com/gazebosim/gz-sim/blob/4fa1f26a738cbdb3ff0c4a37fdde9506f3cd9b4e/include/gz/sim/Util.hh#L300).
