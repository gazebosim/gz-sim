# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
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

import math
import unittest
from ignition.math import RollingMean


class TestRollingMean(unittest.TestCase):

    def test_rolling_mean(self):
        mean = RollingMean()
        self.assertEqual(0, mean.count())
        self.assertEqual(10, mean.window_size())

        mean.set_window_size(4)
        self.assertEqual(4, mean.window_size())
        mean.set_window_size(0)
        self.assertEqual(4, mean.window_size())

        mean.push(1.0)
        self.assertAlmostEqual(1.0, mean.mean())
        mean.push(2.0)
        self.assertAlmostEqual(1.5, mean.mean())
        mean.push(3.0)
        self.assertAlmostEqual(2.0, mean.mean())
        mean.push(10.0)
        self.assertAlmostEqual(4.0, mean.mean())
        mean.push(20.0)
        self.assertAlmostEqual(8.75, mean.mean())

        mean.clear()
        self.assertTrue(math.isnan(mean.mean()))

        mean.push(100.0)
        mean.push(200.0)
        mean.push(300.0)
        self.assertEqual(3, mean.count())
        mean.set_window_size(2)
        self.assertEqual(0, mean.count())


if __name__ == '__main__':
    unittest.main()
