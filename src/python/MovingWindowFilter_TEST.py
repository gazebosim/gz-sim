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
from ignition.math import MovingWindowFilterd
from ignition.math import MovingWindowFilteri
from ignition.math import MovingWindowFilterv3
from ignition.math import Vector3d


class TestFilter(unittest.TestCase):

    def test_set_window_size(self):
        filter_int = MovingWindowFilteri()

        self.assertEqual(filter_int.window_size(), 4)
        self.assertFalse(filter_int.window_filled())

        filter_int.set_window_size(10)
        self.assertEqual(filter_int.window_size(), 10)
        self.assertFalse(filter_int.window_filled())

    def test_filter_something(self):
        double_mwf = MovingWindowFilterd()
        double_mwf_2 = MovingWindowFilterd()
        vector_mwf = MovingWindowFilterv3()

        double_mwf.set_window_size(10)
        double_mwf_2.set_window_size(2)
        vector_mwf.set_window_size(40)

        for i in range(20):
            double_mwf.update(i)
            double_mwf_2.update(i)
            v = Vector3d(1.0*i,
                         2.0*i,
                         3.0*i)
            vector_mwf.update(v)

        sum = 0
        for i in range(20-10, 20, 1):
            sum += i
        self.assertAlmostEqual(double_mwf.value(), sum/10.0)
        self.assertAlmostEqual(double_mwf_2.value(), (18.0+19.0)/2.0)

        vsum = Vector3d()
        for i in range(20):
            vsum += Vector3d(1.0*i,
                             2.0*i,
                             3.0*i)
        self.assertAlmostEqual(vector_mwf.value(), vsum / 20.0)


if __name__ == '__main__':
    unittest.main()
