# Copyright (C) 2022 Open Source Robotics Foundation
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

import unittest

import ignition
from ignition.math import Ellipsoidd, Material, MassMatrix3d, Vector3d

import math

class TestEllipsoid(unittest.TestCase):

    def test_constructor(self):
        ellipsoid = Ellipsoidd()
        self.assertEqual(Vector3d.ZERO, ellipsoid.radii())
        self.assertEqual(Material(), ellipsoid.material())

        ellipsoid2 = Ellipsoidd()
        self.assertEqual(ellipsoid, ellipsoid2)

        # Vector3 of radii constructor
        expectedRadii = Vector3d(1.0, 2.0, 3.0)
        ellipsoid = Ellipsoidd(expectedRadii)
        self.assertEqual(expectedRadii, ellipsoid.radii())
        self.assertEqual(Material(), ellipsoid.material())

        ellipsoid2 = Ellipsoidd(expectedRadii)
        self.assertEqual(ellipsoid, ellipsoid2)

        # Vector3 of radii and material
        expectedRadii = Vector3d(1.0, 2.0, 3.0)
        expectedMaterial = Material(ignition.math.MaterialType.WOOD)
        ellipsoid = Ellipsoidd(expectedRadii, expectedMaterial)
        self.assertEqual(expectedRadii, ellipsoid.radii())
        self.assertEqual(expectedMaterial, ellipsoid.material())

        ellipsoid2 = Ellipsoidd(expectedRadii, expectedMaterial)
        self.assertEqual(ellipsoid, ellipsoid2)


    def test_mutators(self):
        ellipsoid = Ellipsoidd()
        self.assertEqual(Vector3d.ZERO, ellipsoid.radii())
        self.assertEqual(Material(), ellipsoid.material())

        expectedRadii = Vector3d(1.0, 2.0, 3.0)
        ellipsoid.set_radii(expectedRadii)

        expectedMaterial = Material(ignition.math.MaterialType.PINE)
        ellipsoid.set_material(expectedMaterial)

        self.assertEqual(expectedRadii, ellipsoid.radii())
        self.assertEqual(expectedMaterial, ellipsoid.material())


    def test_volume_an_density(self):
        mass = 1.0
        # Basic sphere
        ellipsoid = Ellipsoidd(Vector3d.ONE * 2.)

        expectedVolume = (4. / 3.) * math.pi * math.pow(2.0, 3)
        self.assertEqual(expectedVolume, ellipsoid.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, ellipsoid.density_from_mass(mass))

        ellipsoid2 = Ellipsoidd(Vector3d(1, 10, 100))
        expectedVolume = (4. / 3.) * math.pi * 1. * 10. * 100.
        self.assertEqual(expectedVolume, ellipsoid2.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, ellipsoid2.density_from_mass(mass))

        # Check bad cases
        ellipsoid3 = Ellipsoidd(Vector3d.ZERO)
        self.assertFalse(ellipsoid3.set_density_from_mass(mass))

        ellipsoid4 = Ellipsoidd(-Vector3d.ONE)
        self.assertFalse(ellipsoid4.set_density_from_mass(mass))

        ellipsoid5 = Ellipsoidd(Vector3d(-1, 1, 1))
        self.assertFalse(ellipsoid5.set_density_from_mass(mass))

        ellipsoid6 = Ellipsoidd(Vector3d(-1, -1, 1))
        self.assertFalse(ellipsoid6.set_density_from_mass(mass))


    def test_mass(self):
        mass = 2.0
        ellipsoid = Ellipsoidd(Vector3d(1, 10, 100))
        ellipsoid.set_density_from_mass(mass)

        ixx = (mass / 5.0) * (10. * 10. + 100. * 100.)
        iyy = (mass / 5.0) * (1. * 1. + 100. * 100.)
        izz = (mass / 5.0) * (1. * 1. + 10. * 10.)
        expectedMassMat = MassMatrix3d(
            mass, Vector3d(ixx, iyy, izz), Vector3d.ZERO)

        massMat = ellipsoid.mass_matrix()
        self.assertEqual(expectedMassMat, massMat)
        self.assertEqual(expectedMassMat.diagonal_moments(), massMat.diagonal_moments())
        self.assertEqual(expectedMassMat.mass(), massMat.mass())

        # Zero case
        ellipsoid2 = Ellipsoidd()
        self.assertEqual(None, ellipsoid2.mass_matrix())

        # Check bad cases
        ellipsoid3 = Ellipsoidd(-Vector3d.ONE)
        self.assertEqual(None, ellipsoid3.mass_matrix())

        ellipsoid4 = Ellipsoidd(Vector3d(-1, 1, 1))
        self.assertEqual(None, ellipsoid4.mass_matrix())

        ellipsoid5 = Ellipsoidd(Vector3d(-1, -1, 1))
        self.assertEqual(None, ellipsoid5.mass_matrix())

if __name__ == '__main__':
    unittest.main()
