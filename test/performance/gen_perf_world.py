#!/usr/bin/env python3
#
# Copyright (C) 2026 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Generate reproducible SDF worlds for performance measurements.

The world holds N copies of a small differential-drive robot (`amr`) on a grid.
Every model is defined inline, so a run needs no Fuel access. The output
depends only on the arguments: the same arguments always produce the same
bytes.

Only the systems named in --plugins are loaded. The world sets
`include_server_config_plugins` to false, so the server never adds
SceneBroadcaster or UserCommands on its own.

Every robot receives the same constant velocity command. All DiffDrive systems
subscribe to one shared topic, and a TriggeredPublisher system publishes the
command each time the world statistics message arrives. The robots start
moving on the first statistics message and drive circles of radius
linear_speed / angular_speed, so they stay near their grid cell.

Example:

  gen_perf_world.py --robots 100 --plugins physics \\
      --output amr_100.sdf
  gen_perf_world.py --robots 50 --plugins physics,sensors \\
      --lidar gpu_lidar --warehouse --output amr_50_lidar.sdf
"""

import argparse
import math
import sys

# World-level systems that --plugins can request.
WORLD_PLUGINS = {
    'physics': ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
    'scene_broadcaster': ('gz-sim-scene-broadcaster-system',
                          'gz::sim::systems::SceneBroadcaster'),
    'user_commands': ('gz-sim-user-commands-system',
                      'gz::sim::systems::UserCommands'),
    'sensors': ('gz-sim-sensors-system', 'gz::sim::systems::Sensors'),
}

PHYSICS_ENGINES = {
    'dartsim': None,  # The Physics system default.
    'bullet-featherstone': 'gz-physics-bullet-featherstone-plugin',
}

# Command topic shared by every robot.
COMMAND_TOPIC = '/bench/cmd_vel'

# Robot dimensions in meters.
CHASSIS_SIZE = (0.8, 0.5, 0.2)
WHEEL_RADIUS = 0.1
WHEEL_WIDTH = 0.04
WHEEL_SEPARATION = 0.5
CASTER_RADIUS = 0.05
CHASSIS_HEIGHT = 0.15  # Chassis center above the ground.

# Robots drive circles with a diameter below this value, in meters, when the
# default speeds are used. Closer grid spacing risks robot-to-robot contact.
MIN_SPACING = 6.0


def fmt(_value):
    """Format a number with a stable, short representation."""
    return ('%.6g' % _value)


def world_plugin(_name, _args):
    """Return the <plugin> element for a world-level system."""
    filename, classname = WORLD_PLUGINS[_name]
    body = ''
    if _name == 'physics':
        engine = PHYSICS_ENGINES[_args.physics_engine]
        if engine:
            body = ('\n      <engine>%s</engine>\n    ' % engine)
    elif _name == 'sensors':
        body = ('\n      <render_engine>%s</render_engine>\n    ' %
                _args.render_engine)
    return ('    <plugin filename="%s" name="%s">%s</plugin>\n' %
            (filename, classname, body))


def triggered_publisher(_world_name, _args):
    """Return the plugin that publishes the constant robot command."""
    return '''    <plugin filename="gz-sim-triggered-publisher-system"
            name="gz::sim::systems::TriggeredPublisher">
      <input type="gz.msgs.WorldStatistics" topic="/world/%s/stats"/>
      <output type="gz.msgs.Twist" topic="%s">
        linear: {x: %s}, angular: {z: %s}
      </output>
    </plugin>
''' % (_world_name, COMMAND_TOPIC, fmt(_args.linear_speed),
       fmt(_args.angular_speed))


def lidar_sensor(_args):
    """Return the <sensor> element for a gpu_lidar."""
    half_fov = math.radians(_args.lidar_fov) / 2.0
    return '''        <sensor name="lidar" type="gpu_lidar">
          <pose>0 0 %s 0 0 0</pose>
          <update_rate>%s</update_rate>
          <always_on>true</always_on>
          <visualize>false</visualize>
          <ray>
            <scan>
              <horizontal>
                <samples>%d</samples>
                <resolution>1</resolution>
                <min_angle>%s</min_angle>
                <max_angle>%s</max_angle>
              </horizontal>
              <vertical>
                <samples>1</samples>
                <resolution>1</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
              </vertical>
            </scan>
            <range>
              <min>0.1</min>
              <max>%s</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
        </sensor>
''' % (fmt(CHASSIS_SIZE[2] / 2.0 + 0.05), fmt(_args.lidar_rate),
       _args.lidar_samples, fmt(-half_fov), fmt(half_fov),
       fmt(_args.lidar_range))


def inertia_box(_mass, _size):
    """Return (ixx, iyy, izz) of a solid box."""
    x, y, z = _size
    return (_mass * (y * y + z * z) / 12.0,
            _mass * (x * x + z * z) / 12.0,
            _mass * (x * x + y * y) / 12.0)


def inertia_cylinder(_mass, _radius, _length):
    """Return (ixx, iyy, izz) of a solid cylinder along z."""
    transverse = _mass * (3.0 * _radius * _radius + _length * _length) / 12.0
    return (transverse, transverse, 0.5 * _mass * _radius * _radius)


def inertia_sphere(_mass, _radius):
    """Return the diagonal inertia of a solid sphere."""
    i = 0.4 * _mass * _radius * _radius
    return (i, i, i)


def inertial(_mass, _inertia):
    """Return an <inertial> element."""
    return '''          <inertial>
            <mass>%s</mass>
            <inertia>
              <ixx>%s</ixx>
              <iyy>%s</iyy>
              <izz>%s</izz>
            </inertia>
          </inertial>
''' % (fmt(_mass), fmt(_inertia[0]), fmt(_inertia[1]), fmt(_inertia[2]))


def amr_model(_name, _x, _y, _args):
    """Return the inline amr model, placed at (_x, _y)."""
    cx, cy, cz = CHASSIS_SIZE
    wheel_y = WHEEL_SEPARATION / 2.0
    wheel_z = WHEEL_RADIUS - CHASSIS_HEIGHT
    caster_x = -cx / 2.0 + 0.1
    caster_z = CASTER_RADIUS - CHASSIS_HEIGHT

    wheel = '''      <link name="%(side)s_wheel">
        <pose relative_to="chassis">0 %(y)s %(z)s -1.5707 0 0</pose>
%(inertial)s        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>%(r)s</radius>
              <length>%(w)s</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>%(r)s</radius>
              <length>%(w)s</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1</mu>
                <mu2>1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
'''
    wheel_inertial = inertial(1.0, inertia_cylinder(1.0, WHEEL_RADIUS,
                                                    WHEEL_WIDTH))
    wheels = ''
    for side, y in (('left', wheel_y), ('right', -wheel_y)):
        wheels += wheel % {'side': side, 'y': fmt(y), 'z': fmt(wheel_z),
                           'r': fmt(WHEEL_RADIUS), 'w': fmt(WHEEL_WIDTH),
                           'inertial': wheel_inertial}

    joint = '''      <joint name="%s_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>%s_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit>
            <lower>-1e16</lower>
            <upper>1e16</upper>
          </limit>
        </axis>
      </joint>
'''

    sensor = lidar_sensor(_args) if _args.lidar == 'gpu_lidar' else ''
    drive = ''
    if _args.drive:
        drive = '''      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>%s</wheel_separation>
        <wheel_radius>%s</wheel_radius>
        <topic>%s</topic>
      </plugin>
''' % (fmt(WHEEL_SEPARATION), fmt(WHEEL_RADIUS), COMMAND_TOPIC)

    return '''    <model name="%(name)s">
      <pose>%(x)s %(y)s %(z)s 0 0 0</pose>
      <link name="chassis">
%(chassis_inertial)s        <visual name="visual">
          <geometry>
            <box>
              <size>%(csize)s</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>%(csize)s</size>
            </box>
          </geometry>
        </collision>
%(sensor)s      </link>
%(wheels)s      <link name="caster">
        <pose relative_to="chassis">%(cx)s 0 %(cz)s 0 0 0</pose>
%(caster_inertial)s        <visual name="visual">
          <geometry>
            <sphere>
              <radius>%(cr)s</radius>
            </sphere>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>%(cr)s</radius>
            </sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0</mu>
                <mu2>0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
%(joints)s      <joint name="caster_joint" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
%(drive)s    </model>
''' % {
        'name': _name, 'x': fmt(_x), 'y': fmt(_y), 'z': fmt(CHASSIS_HEIGHT),
        'chassis_inertial': inertial(10.0, inertia_box(10.0, CHASSIS_SIZE)),
        'csize': ' '.join(fmt(v) for v in CHASSIS_SIZE),
        'sensor': sensor, 'wheels': wheels,
        'cx': fmt(caster_x), 'cz': fmt(caster_z), 'cr': fmt(CASTER_RADIUS),
        'caster_inertial': inertial(0.5, inertia_sphere(0.5, CASTER_RADIUS)),
        'joints': joint % ('left', 'left') + joint % ('right', 'right'),
        'drive': drive}


def static_box(_name, _pose, _size):
    """Return a <link> with a box collision and visual."""
    return '''      <link name="%s">
        <pose>%s</pose>
        <visual name="visual">
          <geometry>
            <box>
              <size>%s</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>%s</size>
            </box>
          </geometry>
        </collision>
      </link>
''' % (_name, _pose, _size, _size)


def warehouse(_xs, _ys, _spacing):
    """Return a static model of shelf rows between the robot columns.

    The shelves run along y, one line in every gap between robot columns, so
    the entity count grows with the number of columns and not with the number
    of robots.
    """
    x_min = _xs[0] - _spacing / 2.0
    y_min = _ys[0] - _spacing / 2.0
    y_max = _ys[-1] + _spacing / 2.0
    length = y_max - y_min
    y_mid = (y_min + y_max) / 2.0
    shelf_height = 2.0
    shelf_depth = 0.6

    links = ''
    for i in range(len(_xs) + 1):
        x = x_min + i * _spacing
        links += static_box(
            'shelf_%d' % i,
            '%s %s %s 0 0 0' % (fmt(x), fmt(y_mid), fmt(shelf_height / 2.0)),
            '%s %s %s' % (fmt(shelf_depth), fmt(length), fmt(shelf_height)))
    return '''    <model name="warehouse">
      <static>true</static>
%s    </model>
''' % links


def ground_plane(_extent):
    """Return a static ground model with a box collision."""
    return '''    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>%(e)s %(e)s 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>%(e)s %(e)s 0.1</size>
            </box>
          </geometry>
        </visual>
        <pose>0 0 -0.05 0 0 0</pose>
      </link>
    </model>
''' % {'e': fmt(_extent)}


def generate(_args):
    """Return the SDF text for the given arguments."""
    plugins = [p for p in _args.plugins.split(',') if p]
    unknown = [p for p in plugins if p not in WORLD_PLUGINS]
    if unknown:
        raise ValueError('Unknown plugin(s) %s. Choose from: %s' %
                         (unknown, ', '.join(sorted(WORLD_PLUGINS))))
    if len(set(plugins)) != len(plugins):
        raise ValueError('Duplicate entries in --plugins')
    if _args.robots < 1:
        raise ValueError('--robots must be at least 1')
    if _args.lidar == 'gpu_lidar' and 'sensors' not in plugins:
        raise ValueError('--lidar gpu_lidar needs "sensors" in --plugins')
    if _args.lidar == 'none' and 'sensors' in plugins:
        raise ValueError('"sensors" in --plugins but --lidar is none')
    if _args.drive and 'physics' not in plugins:
        raise ValueError('DiffDrive needs "physics" in --plugins')

    columns = _args.columns or int(math.ceil(math.sqrt(_args.robots)))
    rows = int(math.ceil(_args.robots / float(columns)))
    spacing = _args.spacing
    xs = [i * spacing for i in range(columns)]
    ys = [j * spacing for j in range(rows)]

    # Robots drive circles that start at their cell center and curve left.
    if spacing < MIN_SPACING:
        sys.stderr.write('warning: --spacing below %s m may cause robots to '
                         'collide\n' % fmt(MIN_SPACING))

    out = '<?xml version="1.0" ?>\n'
    out += ('<!--\n  Generated by gen_perf_world.py. Do not edit.\n'
            '  robots=%d columns=%d spacing=%s plugins=%s lidar=%s'
            ' step_size=%s\n-->\n' %
            (_args.robots, columns, fmt(spacing), _args.plugins,
             _args.lidar, fmt(_args.step_size)))
    out += '<sdf version="1.11">\n'
    out += '  <world name="%s">\n' % _args.world_name
    out += '''    <physics name="bench" type="ignored">
      <max_step_size>%s</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gz:policies xmlns:gz="http://gazebosim.org/schema">
      <include_server_config_plugins>false</include_server_config_plugins>
    </gz:policies>
''' % fmt(_args.step_size)
    for name in plugins:
        out += world_plugin(name, _args)
    if _args.drive:
        out += triggered_publisher(_args.world_name, _args)
    out += '''    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
'''
    extent = max(columns, rows) * spacing + 20.0
    out += ground_plane(extent)
    if _args.warehouse:
        out += warehouse(xs, ys, spacing)
    for i in range(_args.robots):
        out += amr_model('amr_%d' % i, xs[i % columns], ys[i // columns],
                         _args)
    out += '  </world>\n</sdf>\n'
    return out


def main():
    """Parse arguments and write the world."""
    parser = argparse.ArgumentParser(
        description='Generate a reproducible performance world.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robots', type=int, default=1,
                        help='Number of amr models')
    parser.add_argument('--spacing', type=float, default=8.0,
                        help='Distance between robot grid cells in meters')
    parser.add_argument('--columns', type=int, default=0,
                        help='Grid columns (0 selects ceil(sqrt(robots)))')
    parser.add_argument('--plugins', default='physics',
                        help='Comma separated world systems to load. '
                        'Choices: ' + ', '.join(sorted(WORLD_PLUGINS)))
    parser.add_argument('--physics-engine', default='dartsim',
                        choices=sorted(PHYSICS_ENGINES),
                        help='Physics engine plugin')
    parser.add_argument('--drive', action=argparse.BooleanOptionalAction,
                        default=True,
                        help='Add DiffDrive and the constant command')
    parser.add_argument('--linear-speed', type=float, default=0.5,
                        help='Commanded linear speed in m/s')
    parser.add_argument('--angular-speed', type=float, default=0.2,
                        help='Commanded angular speed in rad/s')
    parser.add_argument('--lidar', default='none',
                        choices=['none', 'gpu_lidar'],
                        help='Sensor on each robot')
    parser.add_argument('--lidar-samples', type=int, default=720,
                        help='Horizontal samples per scan')
    parser.add_argument('--lidar-rate', type=float, default=10.0,
                        help='Lidar update rate in Hz')
    parser.add_argument('--lidar-fov', type=float, default=270.0,
                        help='Horizontal field of view in degrees')
    parser.add_argument('--lidar-range', type=float, default=10.0,
                        help='Maximum range in meters')
    parser.add_argument('--render-engine', default='ogre2',
                        help='Render engine of the Sensors system')
    parser.add_argument('--step-size', type=float, default=0.001,
                        help='Physics step size in seconds')
    parser.add_argument('--warehouse', action='store_true',
                        help='Add static shelves so lidars see geometry')
    parser.add_argument('--world-name', default='perf_world',
                        help='Name of the world')
    parser.add_argument('--output', '-o', default='-',
                        help='Output file, or - for stdout')
    args = parser.parse_args()

    try:
        sdf = generate(args)
    except ValueError as err:
        parser.error(str(err))

    if args.output == '-':
        sys.stdout.write(sdf)
    else:
        with open(args.output, 'w') as handle:
            handle.write(sdf)


if __name__ == '__main__':
    main()
