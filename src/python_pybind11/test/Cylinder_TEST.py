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

import math
import unittest

import ignition
from ignition.math import Cylinderd, MassMatrix3d, Material, Quaterniond


class TestCylinder(unittest.TestCase):

    def test_constructor(self):
        # Default constructor
        cylinder = Cylinderd()
        self.assertEqual(0.0, cylinder.length())
        self.assertEqual(0.0, cylinder.radius())
        self.assertEqual(Quaterniond.IDENTITY, cylinder.rotational_offset())
        self.assertEqual(Material(), cylinder.mat())

        cylinder2 = Cylinderd()
        self.assertEqual(cylinder, cylinder2)

        # Length and radius constructor
        cylinder = Cylinderd(1.0, 2.0)
        self.assertEqual(1.0, cylinder.length())
        self.assertEqual(2.0, cylinder.radius())
        self.assertEqual(Quaterniond.IDENTITY, cylinder.rotational_offset())
        self.assertEqual(Material(), cylinder.mat())

        cylinder2 = Cylinderd(1.0, 2.0)
        self.assertEqual(cylinder, cylinder2)

        # Length, radius, and rot constructor
        cylinder = Cylinderd(1.0, 2.0, Quaterniond(0.1, 0.2, 0.3))
        self.assertEqual(1.0, cylinder.length())
        self.assertEqual(2.0, cylinder.radius())
        self.assertEqual(Quaterniond(0.1, 0.2, 0.3),
                         cylinder.rotational_offset())
        self.assertEqual(Material(), cylinder.mat())

        cylinder2 = Cylinderd(1.0, 2.0, Quaterniond(0.1, 0.2, 0.3))
        self.assertEqual(cylinder, cylinder2)

        # Length, radius, mat and rot constructor
        cylinder = Cylinderd(1.0, 2.0, Material(ignition.math.MaterialType.WOOD),
                             Quaterniond(0.1, 0.2, 0.3))
        self.assertEqual(1.0, cylinder.length())
        self.assertEqual(2.0, cylinder.radius())
        self.assertEqual(Quaterniond(0.1, 0.2, 0.3), cylinder.rotational_offset())
        self.assertEqual(Material(ignition.math.MaterialType.WOOD), cylinder.mat())

        cylinder2 = Cylinderd(1.0, 2.0, Material(ignition.math.MaterialType.WOOD),
                              Quaterniond(0.1, 0.2, 0.3))
        self.assertEqual(cylinder, cylinder2)

    def test_mutators(self):
        cylinder = Cylinderd()
        self.assertEqual(0.0, cylinder.length())
        self.assertEqual(0.0, cylinder.radius())
        self.assertEqual(Quaterniond.IDENTITY, cylinder.rotational_offset())
        self.assertEqual(Material(), cylinder.mat())

        cylinder.set_length(100.1)
        cylinder.set_radius(.123)
        cylinder.set_rotational_offset(Quaterniond(1.2, 2.3, 3.4))
        cylinder.set_mat(Material(ignition.math.MaterialType.PINE))

        self.assertEqual(100.1, cylinder.length())
        self.assertEqual(.123, cylinder.radius())
        self.assertEqual(Quaterniond(1.2, 2.3, 3.4), cylinder.rotational_offset())
        self.assertEqual(Material(ignition.math.MaterialType.PINE), cylinder.mat())

    def test_volume_and_density(self):
        mass = 1.0
        cylinder = Cylinderd(1.0, 0.001)
        expectedVolume = (math.pi * math.pow(0.001, 2) * 1.0)
        self.assertEqual(expectedVolume, cylinder.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, cylinder.density_from_mass(mass))

        # Bad density
        cylinder2 = Cylinderd()
        self.assertGreater(0.0, cylinder2.density_from_mass(mass))

    def test_mass(self):
        mass = 2.0
        length = 2.0
        r = 0.1
        cylinder = Cylinderd(length, r)
        cylinder.set_density_from_mass(mass)

        massMat = MassMatrix3d()
        ixxIyy = (1/12.0) * mass * (3*r*r + length*length)
        izz = 0.5 * mass * r * r

        expectedMassMat = MassMatrix3d()
        expectedMassMat.set_inertia_matrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0)
        expectedMassMat.set_mass(mass)

        cylinder.mass_matrix(massMat)
        self.assertEqual(expectedMassMat, massMat)
        self.assertEqual(expectedMassMat.mass(), massMat.mass())


if __name__ == '__main__':
    unittest.main()
