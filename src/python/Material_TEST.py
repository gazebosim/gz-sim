# Copyright (C) 2021 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http:#www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import unittest

import ignition.math


class TestMaterial(unittest.TestCase):

    def test_init(self):
        mats = ignition.math.Material.predefined()
        self.assertTrue(len(mats))

        # Make sure that the number of elements in the MaterialType enum matches
        # the number of elements in the MaterialDensity::materials map.
        self.assertEqual(ignition.math.MaterialType_UNKNOWN_MATERIAL, len(mats))

        # Iterate over each element in the enum. Check the that enum value
        # matches the type value in the mats map.
        for i in range(ignition.math.MaterialType_UNKNOWN_MATERIAL):
            # Get the type of the material for MaterialType i.
            self.assertEqual(i, next(iter(mats.find(i)))[1].type())

            # The name should not be empty
            self.assertTrue(next(iter(mats.find(i)))[1].name())

            # The density should be less than the max double value and greater than
            # zero.
            self.assertLess(next(iter(mats.find(i)))[1].density(), sys.float_info.max)
            self.assertGreater(next(iter(mats.find(i)))[1].density(), 0.0)

        malicious = ignition.math.Material(42)
        self.assertEqual(-1.0, malicious.density())
        self.assertEqual('', malicious.name())

        byDensity = ignition.math.Material(42.2)
        self.assertEqual(42.2, byDensity.density())
        self.assertEqual(ignition.math.MaterialType_UNKNOWN_MATERIAL, byDensity.type())

    def test_comparison(self):
        aluminum = ignition.math.Material(ignition.math.MaterialType_ALUMINUM)

        modified = ignition.math.Material(aluminum)
        self.assertEqual(modified, aluminum)

        modified.set_density(1234.0)
        self.assertNotEqual(modified, aluminum)

        modified = ignition.math.Material(aluminum)
        self.assertEqual(modified, aluminum)

        modified.set_type(ignition.math.MaterialType_PINE)
        self.assertNotEqual(modified, aluminum)

    def test_accessors(self):

        mat = ignition.math.Material("Aluminum")
        mat1 = ignition.math.Material("aluminum")
        mat2 = ignition.math.Material(ignition.math.MaterialType_ALUMINUM)
        mat3 = ignition.math.Material(mat2)

        self.assertAlmostEqual(2700.0, mat.density())
        self.assertEqual(mat, mat1)
        self.assertEqual(mat1, mat2)
        self.assertEqual(mat2, mat3)

        # Test constructor
        mat4 = ignition.math.Material(mat3)
        self.assertEqual(mat2, mat4)

        mat5 = ignition.math.Material(mat4)
        self.assertEqual(mat2, mat5)

        mat = ignition.math.Material("Notfoundium")
        self.assertGreater(0.0, mat.density())
        self.assertEqual(ignition.math.MaterialType_UNKNOWN_MATERIAL,
                         mat.type())
        self.assertFalse(mat.name())

        material = ignition.math.Material()
        material.set_to_nearest_density(19300.0)
        self.assertEqual(ignition.math.MaterialType_TUNGSTEN, material.type())
        self.assertAlmostEqual(19300.0, material.density())

        material = ignition.math.Material()
        material.set_to_nearest_density(1001001.001, 1e-3)
        self.assertEqual(ignition.math.MaterialType_UNKNOWN_MATERIAL,
                         material.type())
        self.assertGreater(0.0, material.density())
        material = ignition.math.Material()
        material.set_to_nearest_density(1001001.001)
        self.assertEqual(ignition.math.MaterialType_TUNGSTEN, material.type())
        self.assertAlmostEqual(19300, material.density())


if __name__ == '__main__':
    unittest.main()
