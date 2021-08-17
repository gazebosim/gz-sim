# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import ignition.math


class TestPythonInterface(unittest.TestCase):

    def test_construction(self):
        angle1 = ignition.math.Angle()
        self.assertEqual(angle1.Radian(), 0.0)
        v1 = ignition.math.Vector3d(0, 0, 0)
        self.assertEqual(v1, ignition.math.Vector3d.Zero)
        v2 = ignition.math.Vector2d(1, 2)
        self.assertEqual(v2.X(), 1)
        self.assertEqual(v2.Y(), 2)


if __name__ == '__main__':
    unittest.main()
