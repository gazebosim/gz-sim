\page spherical_coordinates Spherical coordinates

Gazebo supports the use of real world latitude and longitude coordinates in its
simulations using the
[WGS84](https://en.wikipedia.org/wiki/World_Geodetic_System#1984_version)
geodetic system.

Gazebo's simulation is always performed in Cartesian coordinates (good old XYZ).
Therefore, in order to use spherical coordinates, it's necessary to project
coordinates expressed in the `WGS84` frame to Cartesian and back.

This tutorial will go over how to:

* Define what latitude / longitude the world origin corresponds to
* Query the world origin or an entity's coordinates in latitude / longitude
* Spawn entities at given latitude / longitude
* Teleport entities based on a given latitude / longitude

## Coordinates for the world origin

The world's Cartesian coordinates correspond to a
[local tangent plane](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates)
at a given point on the planetary surface. By default, this plane follows the
ENU (East-North-Up) convention, as shown on the image below:

@image html files/spherical_coordinates/ENU.svg

Users can define where the origin of the ENU coordinates sits on the surface
of the planet in three different ways: through the SDF file, the GUI, or through
an Ignition Transport service call.

Changing the world origin will only affect operations performed afterwards. For
example, models already in the world will not be moved when the world origin's
coordinates change.

### SDF file

On the SDF file, it's possible to define the world origin's spherical
coordinates with the
[`/world/spherical_coordinates`](http://sdformat.org/spec?ver=1.8&elem=world#world_spherical_coordinates)
element.

For example:

```.xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>-22.9</latitude_deg>
  <longitude_deg>-43.2</longitude_deg>
  <elevation>0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

At the moment, the only surface model supported is WGS84 and the only world
frame orientation is ENU.

Try out an example world that ships with Gazebo and has the coordinates above
as follows:

```
ign gazebo spherical_coordinates.sdf
```

On the component inspector, expand the `Spherical Coordinates` component to see
that the coordinates were set correctly:

@image html files/spherical_coordinates/inspector.png

### GUI

To change the world origin through the GUI, edit values within the component
inspector shown above, and these changes will be reflected on simulation.

### Service

It's also possible to change the coordinates for the world origin at runtime
through the `/world/<world_name>/set_spherical_coordinates` service.

Loading the world above, try calling for example:

```.bash
ign service \
-s /world/spherical_coordinates/set_spherical_coordinates \
--reqtype ignition.msgs.SphericalCoordinates \
--reptype ignition.msgs.Boolean \
--timeout 2000 \
--req "surface_model: EARTH_WGS84, latitude_deg: 35.6, longitude_deg: 140.1, elevation: 10.0"
```

You should see the coordinates changing on the GUI.

## Creating entities

See the [Entity creation](entity_creation.html) tutorial for an introduction
to creating entities.

In order to create an entity at a given spherical coordinate using the
`/world/<world_name>/create` service, omit the `pose` field from the
`gz::msgs::EntityFactory` message and use the `spherical_coordinates`
field instead.

For example, you can spawn a sphere into the `spherical_coordinates.sdf` world
as follows:

```.bash
ign service -s /world/spherical_coordinates/create \
--reqtype ignition.msgs.EntityFactory \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'sdf: '\
'"<?xml version=\"1.0\" ?>'\
'<sdf version=\"1.6\">'\
'<model name=\"spawned_model\">'\
'<link name=\"link\">'\
'<visual name=\"visual\">'\
'<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
'</visual>'\
'<collision name=\"visual\">'\
'<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
'</collision>'\
'</link>'\
'</model>'\
'</sdf>" '\
'spherical_coordinates: {latitude_deg: 35.59999, longitude_deg: 140.09999, elevation: 11.0} '
```

## Moving entities

It's also possible to move entities to a given spherical coordinate. Just use the
`set_spherical_coordinate` service that was used to set the world origin, but
specify the entity to be moved.

For example, to move the sphere created above:

```.bash
ign service -s /world/spherical_coordinates/set_spherical_coordinates \
--reqtype ignition.msgs.SphericalCoordinates \
--reptype ignition.msgs.Boolean \
--timeout 300 \
--req 'entity: {name: "spawned_model", type: MODEL}, latitude_deg: 35.59990, longitude_deg: 140.09990'
```

## Querying entities' coordinates

When writing plugins, developers can use the
`gz::sim::sphericalCoordinates` helper function to query the current
coordinates for any entity that has a pose.

