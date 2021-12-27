# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from ignition.math import BiQuadd
from ignition.math import BiQuadVector3
from ignition.math import OnePoled
from ignition.math import OnePoleQuaternion
from ignition.math import OnePoleVector3
from ignition.math import Quaterniond
from ignition.math import Vector3d


class TestFilter(unittest.TestCase):

    def test_one_pole(self):
        filter_a = OnePoled()
        self.assertAlmostEqual(filter_a.process(0.2), 0.0)

        filter_a.fc(0.6, 1.4)
        self.assertAlmostEqual(filter_a.process(2.5), 2.3307710879153634)

        filter_b = OnePoled(0.1, 0.2)
        self.assertAlmostEqual(filter_b.process(0.5), 0.47839304086811385)

        filter_b.set(5.4)
        self.assertAlmostEqual(filter_b.value(), 5.4)

    def test_one_pole_quaternion(self):
        filter_a = OnePoleQuaternion()
        self.assertAlmostEqual(filter_a.value(), Quaterniond(1, 0, 0, 0))

        filter_b = OnePoleQuaternion(0.4, 1.4)
        self.assertAlmostEqual(filter_b.value(), Quaterniond(1, 0, 0, 0))

        self.assertAlmostEqual(filter_a.process(Quaterniond(0.1, 0.2, 0.3)),
                               Quaterniond(1, 0, 0, 0))

        self.assertAlmostEqual(filter_b.process(Quaterniond(0.1, 0.2, 0.3)),
                               Quaterniond(0.98841, 0.0286272,
                                           0.0885614, 0.119929))

    def test_one_pole_vector3(self):
        filter_a = OnePoleVector3()
        self.assertAlmostEqual(filter_a.value(), Vector3d(0, 0, 0))

        filter_b = OnePoleVector3(1.2, 3.4)
        self.assertAlmostEqual(filter_b.value(), Vector3d(0, 0, 0))

        self.assertAlmostEqual(filter_a.process(Vector3d(0.1, 0.2, 0.3)),
                               Vector3d(0, 0, 0))

        self.assertAlmostEqual(filter_b.process(Vector3d(0.1, 0.2, 0.3)),
                               Vector3d(0.089113, 0.178226, 0.267339))

    def test_biquad(self):
        filter_a = BiQuadd()
        self.assertAlmostEqual(filter_a.value(), 0.0, delta=1e-10)
        self.assertAlmostEqual(filter_a.process(1.1), 0.0, delta=1e-10)

        filter_a.fc(0.3, 1.4)
        self.assertAlmostEqual(filter_a.process(1.2), 0.66924691484768517)

        filter_a.fc(0.3, 1.4, 0.1)
        self.assertAlmostEqual(filter_a.process(10.25), 0.96057152402651302)

        filter_b = BiQuadd(4.3, 10.6)
        self.assertAlmostEqual(filter_b.value(), 0.0, delta=1e-10)
        self.assertAlmostEqual(filter_b.process(0.1234),  0.072418159950486546)

        filter_b.set(4.5)
        self.assertAlmostEqual(filter_b.value(), 4.5)

    def test_biquad_vector3(self):
        filter_a = BiQuadVector3()
        self.assertEqual(filter_a.value(), Vector3d(0, 0, 0))
        self.assertEqual(filter_a.process(Vector3d(1.1, 2.3, 3.4)),
                         Vector3d(0, 0, 0))

        filter_b = BiQuadVector3(6.5, 22.4)
        self.assertEqual(filter_b.value(), Vector3d(0, 0, 0))
        self.assertEqual(filter_b.process(Vector3d(0.1, 20.3, 33.45)),
                         Vector3d(0.031748, 6.44475, 10.6196))


if __name__ == '__main__':
    unittest.main()
