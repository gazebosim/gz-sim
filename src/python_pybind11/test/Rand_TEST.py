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
from ignition.math import Rand


class TestRand(unittest.TestCase):

    def test_rand(self):
        d = Rand.dbl_uniform(1, 2)
        self.assertGreaterEqual(d, 1)
        self.assertLessEqual(d, 2)

        i = Rand.int_uniform(1, 2)
        self.assertGreaterEqual(i, 1)
        self.assertLessEqual(i, 2)

    def test_set_seed(self):
        N = 10
        first = []
        second = []

        for i in range(N):
            Rand.seed(i)
            first.append(Rand.int_uniform(-10, 10))
            second.append(Rand.int_uniform(-10, 10))

        for i in range(N):
            Rand.seed(i)
            self.assertEqual(Rand.seed(), i)
            self.assertEqual(first[i], Rand.int_uniform(-10, 10))
            self.assertEqual(second[i], Rand.int_uniform(-10, 10))


if __name__ == '__main__':
    unittest.main()
