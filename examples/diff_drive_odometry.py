# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This example will only work if the Python interface library was compiled and
# installed.
#
# Modify the PYTHONPATH environment variable to include the ignition math
# library install path. For example, if you install to /usr:
#
# $ export PYTHONPATH=/usr/lib/python:$PYTHONPATH

import datetime
import math

from ignition.math import Angle, DiffDriveOdometry

odom = DiffDriveOdometry()

wheelSeparation = 2.0
wheelRadius = 0.5
wheelCircumference = 2 * math.pi * wheelRadius

# This is the linear distance traveled per degree of wheel rotation.
distPerDegree = wheelCircumference / 360.0

# Setup the wheel parameters, and initialize
odom.set_wheel_params(wheelSeparation, wheelRadius, wheelRadius)
startTime = datetime.datetime.now()
odom.init(datetime.timedelta())

# Sleep for a little while, then update the odometry with the new wheel
# position.
print('--- Rotate both wheels by 1 degree. ---')
time1 = startTime + datetime.timedelta(milliseconds=100)
odom.update(Angle(1.0 * math.pi / 180),
            Angle(1.0 * math.pi / 180),
            time1 - startTime)

print('\tLinear velocity:\t{} m/s\n\tOdom linear velocity:\t{} m/s'.
    format(distPerDegree / 0.1, odom.linear_velocity()))

print('Angular velocity should be zero since the "robot" is traveling\n' +
      'in a straight line:\n' +
      '\tOdom angular velocity:\t{} rad/s'
      .format(odom.angular_velocity()))

# Sleep again, this time rotate the left wheel by 1 and the right wheel by 2
# degrees.
print('--- This time rotate the left wheel by 1 and the right wheel ' +
      'by 2 degrees ---');
time2 = time1 + datetime.timedelta(milliseconds=100)
odom.update(Angle(2.0 * math.pi / 180),
            Angle(3.0 * math.pi / 180),
            time2 - startTime)

print('The heading should be the arc tangent of the linear distance\n' +
      'traveled by the right wheel (the left wheel was stationary)\n' +
      'divided by the wheel separation.\n' +
      '\tHeading:\t\t{} rad\n\tOdom Heading:\t\t{} rad'.format(
            math.atan2(distPerDegree, wheelSeparation),
                  odom.heading()))

# The X odom reading should have increased by the sine of the heading *
# half the wheel separation.
xDistTraveled = math.sin(
    math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
prevXPos = distPerDegree * 2.0
print('\tX distance traveled:\t{} m\n\tOdom X:\t\t{} m'.format(
        xDistTraveled + prevXPos, odom.x()))

# The Y odom reading should have increased by the cosine of the heading *
# half the wheel separation.
yDistTraveled = (wheelSeparation * 0.5) - math.cos(
        math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
prevYPos = 0.0
print('\tY distance traveled:\t{} m\n\tOdom Y:\t\t{} m'.format(
        yDistTraveled + prevYPos, odom.y()))

print('Angular velocity should be the difference between the x and y\n' +
      'distance traveled divided by the wheel separation divided by\n' +
      'the seconds elapsed.\n' +
      '\tAngular velocity:\t{} rad/s\n\tOdom angular velocity:\t{} rad/s'.format(
        ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1,
        odom.angular_velocity()))
