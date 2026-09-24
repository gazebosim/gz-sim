\page magnetometer Magnetometer

The magnetometer system plugin reports the Earth's magnetic field vector
at each magnetometer sensor based on the sensor's geographic position.

Two modes are available:

1. **Built-in lookup tables** (default) -- A low-resolution
   declination/inclination/strength table derived from earlier WMM data.
   No additional files are needed.

2. **World Magnetic Model (WMM)** -- The high-resolution WMMHR 2025
   spherical harmonic model (degree 133). This provides accurate field
   vectors anywhere on Earth for the 2025--2030 epoch. A coefficient file
   (`WMMHR.COF`) is installed with the plugin.

## Basic usage (lookup tables)

Add the magnetometer system plugin to your world and place a
magnetometer sensor on a model. The world must define
`<spherical_coordinates>` so the plugin can determine geographic
position.

```{.xml}
<world name="magnetometer_world">
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>-33.8688</latitude_deg>
    <longitude_deg>151.2093</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
  </spherical_coordinates>

  <plugin
    filename="gz-sim-magnetometer-system"
    name="gz::sim::systems::Magnetometer">
  </plugin>

  <model name="sensor_model">
    <link name="link">
      <sensor name="mag" type="magnetometer">
        <always_on>1</always_on>
        <update_rate>10</update_rate>
      </sensor>
    </link>
  </model>
</world>
```

## Enabling the World Magnetic Model

To use the high-resolution WMM, add the following parameters to the
magnetometer plugin element:

```{.xml}
<plugin
  filename="gz-sim-magnetometer-system"
  name="gz::sim::systems::Magnetometer">
  <use_world_magnetic_model>true</use_world_magnetic_model>
</plugin>
```

The plugin ships with the `WMMHR.COF` coefficient file and will locate
it automatically in the installed data directory. You can also specify a
custom path:

```{.xml}
<wmm_coefficient_file>/path/to/WMMHR.COF</wmm_coefficient_file>
```

## Plugin parameters

All parameters are optional and go inside the
`<plugin>` element:

* `<use_world_magnetic_model>` (bool, default: `false`):
  Enable the high-resolution WMM. When disabled, the built-in lookup
  tables are used.

* `<wmm_coefficient_file>` (string, default: `"WMMHR.COF"`):
  Path to the WMM spherical harmonic coefficient file. If only a
  filename is given (no directory), the plugin searches the installed
  data directory automatically.

* `<wmm_date>` (double, default: current system date):
  Decimal year for evaluating the model's secular variation. For example,
  `2025.5` corresponds to July 2025. If omitted, the current date is
  used.

* `<wmm_recalculation_distance>` (double, default: `1000`):
  Distance in meters that a sensor must move before the WMM field is
  recomputed for that sensor. The Earth's magnetic field changes slowly
  over distance, so this avoids expensive spherical harmonic evaluation
  on every simulation step. Set to a smaller value if you need the field
  to update more frequently (e.g., for fast-moving vehicles).

* `<use_units_gauss>` (bool, default: `true`):
  When `true`, the magnetic field is reported in Gauss (1 G = 100 uT).
  When `false`, the field is reported in Tesla.

* `<use_earth_frame_ned>` (bool, default: `false`):
  When `true`, the field vector components correspond to North-East-Down.
  When `false` (default), the field uses East-North-Up, matching Gazebo's
  internal convention.

## Published message

The magnetometer sensor publishes `gz.msgs.Magnetometer` messages. The
message includes:

* `field_tesla` -- The magnetic field vector in the sensor's body frame.
  Despite the field name (kept for backward compatibility), the actual
  unit is determined by the `unit` field.

* `unit` -- An enum indicating the measurement unit:
  `GAUSS` (default), `TESLA`, `MICRO_TESLA`, or `NANO_TESLA`.

* `frame` -- An enum indicating the coordinate frame convention used
  when computing the world-frame field before rotating into the body
  frame: `ENU` (default) or `NED`.

## How the WMM works

The World Magnetic Model uses spherical harmonic expansion to compute
the Earth's magnetic field at any point on or above the surface. The
WMMHR 2025 model uses coefficients up to degree and order 133, providing
spatial resolution of approximately 150 km. Secular variation
coefficients allow the model to be evaluated for any date within its
validity epoch (2025.0 to 2030.0).

The plugin evaluates the model in geodetic coordinates (latitude,
longitude, altitude) derived from the sensor's world-frame position
and the world's `<spherical_coordinates>`. The result is a field vector
in the NED frame (nanotesla), which is then converted to the configured
output unit and frame.

To avoid recomputing the spherical harmonic expansion every simulation
step, the plugin caches the field per sensor and only recomputes when
the sensor has moved more than `<wmm_recalculation_distance>` meters
(measured by great-circle distance).

## Try it out

Run the example world:

```bash
gz sim magnetometer_wmm.sdf
```

In another terminal, listen to the magnetometer output:

```bash
gz topic -e -t /world/magnetometer_wmm/model/magnetometer_model/link/link/sensor/magnetometer_sensor/magnetometer
```

You can move the model to a different geographic location using the
spherical coordinates service:

```bash
gz service \
  -s /world/magnetometer_wmm/set_spherical_coordinates \
  --reqtype gz.msgs.SphericalCoordinates \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "entity: {name: 'magnetometer_model', type: MODEL}, latitude_deg: 35.6762, longitude_deg: 139.6503"
```

This moves the model to Tokyo, Japan. The magnetometer output will
update to reflect the different magnetic field at that location.
