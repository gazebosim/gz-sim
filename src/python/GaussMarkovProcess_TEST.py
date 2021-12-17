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

import unittest
from ignition.math import GaussMarkovProcess


class TestGaussMarkovProcess(unittest.TestCase):

    def test_default_constructor(self):
        gmp = GaussMarkovProcess()
        self.assertAlmostEqual(0.0, gmp.start())
        self.assertAlmostEqual(0.0, gmp.value())
        self.assertAlmostEqual(0.0, gmp.theta())
        self.assertAlmostEqual(0.0, gmp.mu())
        self.assertAlmostEqual(0.0, gmp.sigma())

    def test_no_noise(self):
        # Start value of -1.2
        # Theta (rate at which the process should approach the mean) of 1.0
        # Mu (mean value) 2.5
        # Sigma (volatility) of 0.0
        gmp = GaussMarkovProcess(-1.2, 1.0, 2.5, 0)
        self.assertAlmostEqual(-1.2, gmp.start())
        self.assertAlmostEqual(-1.2, gmp.value())
        self.assertAlmostEqual(1.0, gmp.theta())
        self.assertAlmostEqual(2.5, gmp.mu())
        self.assertAlmostEqual(0.0, gmp.sigma())

        # This process should steadily increase to the mean value of 2.5 since
        # there is no noise.
        for i in range(200):
            value = gmp.update(0.1)
            self.assertGreater(value, -1.2)

        self.assertAlmostEqual(2.5, value, delta=1e-4)

        gmp.reset()
        for i in range(200):
            value = gmp.update(0.1)
            self.assertGreater(value, -1.2)

        self.assertAlmostEqual(2.5, value, delta=1e-4)


if __name__ == '__main__':
    unittest.main()
