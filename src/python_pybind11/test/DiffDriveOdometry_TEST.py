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

import datetime
import math
import unittest

from ignition.math import Angle, DiffDriveOdometry


class TestDiffDriveOdometry(unittest.TestCase):

    def test_diff_drive_odometry(self):
        odom = DiffDriveOdometry()
        self.assertEqual(0.0, odom.heading().radian())
        self.assertEqual(0.0, odom.x())
        self.assertEqual(0.0, odom.y())
        self.assertEqual(0.0, odom.linear_velocity())
        self.assertEqual(0.0, odom.angular_velocity().radian())

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
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(Angle(1.0 * math.pi / 180),
                    Angle(1.0 * math.pi / 180),
                    time1 - startTime)
        self.assertEqual(0.0, odom.heading().radian())
        self.assertEqual(distPerDegree, odom.x())
        self.assertEqual(0.0, odom.y())
        # Linear velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(distPerDegree / 0.1, odom.linear_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)

        # Sleep again, then update the odometry with the new wheel position.
        time2 = time1 + datetime.timedelta(milliseconds=100)
        odom.update(Angle(2.0 * math.pi / 180),
                    Angle(2.0 * math.pi / 180),
                    time2 - startTime)
        self.assertEqual(0.0, odom.heading().radian())
        self.assertAlmostEqual(distPerDegree * 2.0, odom.x(), delta=3e-6)
        self.assertEqual(0.0, odom.y())
        # Linear velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(distPerDegree / 0.1, odom.linear_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)

        # Initialize again, and odom values should be reset.
        startTime = datetime.datetime.now()
        odom.init(datetime.timedelta())
        self.assertEqual(0.0, odom.heading().radian())
        self.assertEqual(0.0, odom.x())
        self.assertEqual(0.0, odom.y())
        self.assertEqual(0.0, odom.linear_velocity())
        self.assertEqual(0.0, odom.angular_velocity().radian())

        # Sleep again, this time move 2 degrees in 100ms.
        time1 = startTime + datetime.timedelta(milliseconds=100)
        odom.update(Angle(2.0 * math.pi / 180),
                    Angle(2.0 * math.pi / 180),
                    time1 - startTime)
        self.assertEqual(0.0, odom.heading().radian())
        self.assertAlmostEqual(distPerDegree * 2.0, odom.x(), delta=3e-6)
        self.assertEqual(0.0, odom.y())
        # Linear velocity should be dist_traveled / time_elapsed.
        self.assertAlmostEqual(distPerDegree * 2 / 0.1, odom.linear_velocity(), delta=1e-3)
        # Angular velocity should be zero since the "robot" is traveling in a
        # straight line.
        self.assertAlmostEqual(0.0, odom.angular_velocity().radian(), delta=1e-3)

        # Sleep again, this time rotate the right wheel by 1 degree.
        time2 = time1 + datetime.timedelta(milliseconds=100)
        odom.update(Angle(2.0 * math.pi / 180),
                    Angle(3.0 * math.pi / 180),
                    time2 - startTime)
        # The heading should be the arc tangent of the linear distance traveled
        # by the right wheel (the left wheel was stationary) divided by the
        # wheel separation.
        self.assertAlmostEqual(math.atan2(distPerDegree, wheelSeparation),
                               odom.heading().radian(),
                               delta=1e-6)

        # The X odom reading should have increased by the sine of the heading *
        # half the wheel separation.
        xDistTraveled = math.sin(math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
        prevXPos = distPerDegree * 2.0
        self.assertAlmostEqual(xDistTraveled + prevXPos, odom.x(), delta=3e-6)

        # The Y odom reading should have increased by the cosine of the header *
        # half the wheel separation.
        yDistTraveled = (wheelSeparation * 0.5) - math.cos(math.atan2(distPerDegree, wheelSeparation)) * wheelSeparation * 0.5
        prevYPos = 0.0
        self.assertAlmostEqual(yDistTraveled + prevYPos, odom.y(), delta=3e-6)

        # Angular velocity should be the difference between the x and y distance
        # traveled divided by the wheel separation divided by the seconds
        # elapsed.
        self.assertAlmostEqual(
          ((xDistTraveled - yDistTraveled) / wheelSeparation) / 0.1,
          odom.angular_velocity().radian(), delta=1e-3)


if __name__ == '__main__':
    unittest.main()
