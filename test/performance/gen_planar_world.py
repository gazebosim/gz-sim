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

"""Generate a warehouse world of DiffDrive robots between static shelves.

The world is for measuring the SimplifiedPhysics system (physics mode
planar_kinematic) against the default Physics system on the same robots. It
is reproducible: the output depends only on the arguments, and every model is
defined inline, so a run needs no Fuel access.

Layout: the robots drive along aisles. An aisle runs along x, and the aisles
are `--aisle-pitch` meters apart in y. Shelf rows run along x between the
aisles, cut into segments with gaps. Each aisle holds `--per-aisle` robots,
`--robot-pitch` meters apart.

Every robot starts with a heading that is drawn from the seed: within
`--yaw-spread` degrees of the aisle direction, and against the aisle direction
(facing the next robot, so that the two meet head on) with the probability
`--head-on`. The driver sends the same forward command to every robot, so
the robots that start at an angle run into a shelf, and the ones that face
each other run into each other. The robots are named r0, r1, and so on, and
each listens on /model/r<i>/cmd_vel.

--mode planar puts each robot on planar_kinematic (the SimplifiedPhysics
system moves it). --mode full leaves them on the default Physics system. The
two worlds differ in nothing else.

Example:

  gen_planar_world.py --robots 200 --mode planar --output planar_200.sdf
  gen_planar_world.py --robots 200 --mode full --output full_200.sdf
"""

import argparse
import math
import random
import sys

# Robot dimensions in meters. The chassis is 0.4 m by 0.2 m, the wheels are
# 4 cm wide cylinders 0.4 m apart, and a ball caster holds the rear.
WHEEL_RADIUS = 0.1
WHEEL_SEPARATION = 0.4
SHELF_DEPTH = 0.6
SHELF_HEIGHT = 2.0

ROBOT = """    <model name="r{i}">
      <pose>{x} {y} 0.1 0 0 {yaw}</pose>
      <link name="chassis">
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.0073</ixx><iyy>0.0272</iyy><izz>0.0333</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.4 0.2 0.06</size></box></geometry>
        </collision>
      </link>
      <link name="left_wheel">
        <pose relative_to="chassis">0.05 0.2 0 -1.5707963268 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00132</ixx><iyy>0.00132</iyy><izz>0.0025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.04</length></cylinder>
          </geometry>
        </collision>
      </link>
      <link name="right_wheel">
        <pose relative_to="chassis">0.05 -0.2 0 -1.5707963268 0 0</pose>
        <inertial>
          <mass>0.5</mass>
          <inertia>
            <ixx>0.00132</ixx><iyy>0.00132</iyy><izz>0.0025</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.1</radius><length>0.04</length></cylinder>
          </geometry>
        </collision>
      </link>
      <link name="caster">
        <pose relative_to="chassis">-0.2 0 -0.07 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0</mu><mu2>0</mu2></ode></friction>
          </surface>
        </collision>
      </link>
      <joint name="left_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
      <joint name="caster_joint" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>
      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>{sep}</wheel_separation>
        <wheel_radius>{radius}</wheel_radius>
        <odom_publish_frequency>10</odom_publish_frequency>
      </plugin>{mode}
    </model>
"""

PLANAR_PLUGIN = """
      <plugin filename="gz-sim-simplified-physics-system"
              name="gz::sim::systems::SimplifiedPhysicsModel">
        <mode>planar_kinematic</mode>
      </plugin>"""

HEAD = """<?xml version="1.0" ?>
<!--
  Generated by gen_planar_world.py. Do not edit.
  {summary}
-->
<sdf version="1.9">
  <world name="{name}">
    <physics name="bench" type="ignored">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gz:policies xmlns:gz="http://gazebosim.org/schema">
      <include_server_config_plugins>false</include_server_config_plugins>
    </gz:policies>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>{system}
    <model name="ground">
      <static>true</static>
      <pose>0 0 -0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{width} {depth} 0.2</size></box></geometry>
        </collision>
      </link>
    </model>
"""

SYSTEM_PLUGIN = """
    <plugin filename="gz-sim-simplified-physics-system"
            name="gz::sim::systems::SimplifiedPhysics"/>"""


def fmt(_value):
    """Format a number with a short, stable representation."""
    return '%.6g' % _value


def shelves(_args, _aisles, _length):
    """Return a static model with the shelf rows between the aisles.

    A row has one segment every `shelf_length + shelf_gap` meters.
    """
    links = ''
    stride = _args.shelf_length + _args.shelf_gap
    count = max(1, int(_length // stride))
    x0 = -_args.robot_pitch / 2.0
    for row in range(_aisles + 1):
        y = (row - 0.5) * _args.aisle_pitch
        for seg in range(count):
            x = x0 + (seg + 0.5) * stride
            links += ('      <link name="s{r}_{s}">\n'
                      '        <pose>{x} {y} {z} 0 0 0</pose>\n'
                      '        <collision name="collision">\n'
                      '          <geometry><box><size>{sx} {sy} {sz}</size>'
                      '</box></geometry>\n'
                      '        </collision>\n'
                      '      </link>\n').format(
                          r=row, s=seg, x=fmt(x), y=fmt(y),
                          z=fmt(SHELF_HEIGHT / 2.0),
                          sx=fmt(_args.shelf_length), sy=fmt(SHELF_DEPTH),
                          sz=fmt(SHELF_HEIGHT))
    return ('    <model name="warehouse">\n      <static>true</static>\n'
            + links + '    </model>\n')


def generate(_args):
    """Return the SDF text for the given arguments."""
    if _args.robots < 1:
        raise ValueError('--robots must be at least 1')
    if _args.per_aisle < 1:
        raise ValueError('--per-aisle must be at least 1')
    if _args.aisle_pitch <= SHELF_DEPTH + 1.0:
        raise ValueError('--aisle-pitch leaves no room between the shelves')

    rng = random.Random(_args.seed)
    per_aisle = min(_args.per_aisle, _args.robots)
    aisles = int(math.ceil(_args.robots / float(per_aisle)))
    length = per_aisle * _args.robot_pitch

    summary = ('robots=%d mode=%s per_aisle=%d aisles=%d robot_pitch=%s '
               'aisle_pitch=%s yaw_spread=%s head_on=%s seed=%d step=%s' % (
                   _args.robots, _args.mode, per_aisle, aisles,
                   fmt(_args.robot_pitch), fmt(_args.aisle_pitch),
                   fmt(_args.yaw_spread), fmt(_args.head_on), _args.seed,
                   fmt(_args.step_size)))
    out = HEAD.format(
        summary=summary, name=_args.world_name, step=fmt(_args.step_size),
        system=SYSTEM_PLUGIN if _args.mode == 'planar' else '',
        width=fmt(length + 40.0), depth=fmt(aisles * _args.aisle_pitch + 40.0))
    out += shelves(_args, aisles, length)

    spread = math.radians(_args.yaw_spread)
    for i in range(_args.robots):
        aisle, column = divmod(i, per_aisle)
        # Draw both numbers for every robot, so the world of N robots is a
        # prefix of the world of N+1 robots.
        offset = rng.uniform(-spread, spread)
        against = rng.random() < _args.head_on
        yaw = offset + (math.pi if against else 0.0)
        out += ROBOT.format(
            i=i, x=fmt(column * _args.robot_pitch),
            y=fmt(aisle * _args.aisle_pitch), yaw=fmt(yaw),
            sep=fmt(WHEEL_SEPARATION), radius=fmt(WHEEL_RADIUS),
            mode=PLANAR_PLUGIN if _args.mode == 'planar' else '')
    out += '  </world>\n</sdf>\n'
    return out


def main():
    """Parse arguments and write the world."""
    parser = argparse.ArgumentParser(
        description='Generate a warehouse world of DiffDrive robots.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--robots', type=int, default=100,
                        help='Number of robots')
    parser.add_argument('--mode', choices=['planar', 'full'],
                        default='planar',
                        help='planar_kinematic on SimplifiedPhysics, or the '
                        'default Physics system')
    parser.add_argument('--per-aisle', type=int, default=25,
                        help='Robots in one aisle')
    parser.add_argument('--robot-pitch', type=float, default=4.0,
                        help='Distance between robots along an aisle, m')
    parser.add_argument('--aisle-pitch', type=float, default=3.0,
                        help='Distance between aisle center lines, m')
    parser.add_argument('--shelf-length', type=float, default=5.0,
                        help='Length of one shelf segment, m')
    parser.add_argument('--shelf-gap', type=float, default=0.5,
                        help='Gap between shelf segments, m')
    parser.add_argument('--yaw-spread', type=float, default=35.0,
                        help='Largest start heading off the aisle, degrees')
    parser.add_argument('--head-on', type=float, default=0.3,
                        help='Probability that a robot faces against the '
                        'aisle direction')
    parser.add_argument('--seed', type=int, default=1,
                        help='Seed of the start headings')
    parser.add_argument('--step-size', type=float, default=0.001,
                        help='Physics step size in seconds')
    parser.add_argument('--world-name', default='planar_bench',
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
