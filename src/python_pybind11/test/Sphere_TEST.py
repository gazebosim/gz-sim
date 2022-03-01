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
from ignition.math import MassMatrix3d, Material, Planed, Sphered, Vector2d, Vector3d


class TestSphere(unittest.TestCase):

    def test_constructor(self):
        # Default constructor
        sphere = Sphered()
        self.assertEqual(0.0, sphere.radius())
        self.assertEqual(Material(), sphere.material())

        sphere2 = Sphered()
        self.assertEqual(sphere, sphere2)

        # Radius constructor
        sphere = Sphered(1.0)
        self.assertEqual(1.0, sphere.radius())
        self.assertEqual(Material(), sphere.material())

        sphere2 = Sphered(1.0)
        self.assertEqual(sphere, sphere2)

        # Radius and mat
        sphere = Sphered(1.0, Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(1.0, sphere.radius())
        self.assertEqual(Material(ignition.math.MaterialType.WOOD), sphere.material())

        sphere2 = Sphered(1.0, Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(sphere, sphere2)

    def test_comparison(self):
        wood = Sphered(0.1, Material(ignition.math.MaterialType.WOOD))

        modified = wood
        self.assertEqual(wood, modified)

        modified.set_radius(1.0)
        wood = Sphered(0.1, Material(ignition.math.MaterialType.WOOD))
        self.assertNotEqual(wood, modified)

        modified = wood
        wood = Sphered(0.1, Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(wood, modified)

        modified.set_material(Material(ignition.math.MaterialType.PINE))
        self.assertNotEqual(wood, modified)

    def test_mutators(self):
        sphere = Sphered()
        self.assertEqual(0.0, sphere.radius())
        self.assertEqual(Material(), sphere.material())

        sphere.set_radius(.123)
        sphere.set_material(Material(ignition.math.MaterialType.PINE))

        self.assertEqual(.123, sphere.radius())
        self.assertEqual(Material(ignition.math.MaterialType.PINE), sphere.material())

    def test_volume_and_density(self):
        mass = 1.0
        sphere = Sphered(0.001)
        expectedVolume = (4.0/3.0) * math.pi * math.pow(0.001, 3)
        self.assertEqual(expectedVolume, sphere.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, sphere.density_from_mass(mass))

        # Bad density
        sphere2 = Sphered()
        self.assertGreater(0.0, sphere2.density_from_mass(mass))
        sphere2.set_radius(1.0)
        self.assertGreater(0.0, sphere2.density_from_mass(0.0))
        self.assertFalse(sphere.set_density_from_mass(0.0))

    def test_mass(self):
        mass = 2.0
        r = 0.1
        sphere = Sphered(r)
        self.assertTrue(sphere.set_density_from_mass(mass))

        massMat = MassMatrix3d()
        ixxIyyIzz = 0.4 * mass * r * r

        expectedMassMat = MassMatrix3d()
        expectedMassMat.set_inertia_matrix(ixxIyyIzz, ixxIyyIzz, ixxIyyIzz,
                                           0.0, 0.0, 0.0)
        expectedMassMat.set_mass(mass)

        sphere.mass_matrix(massMat)
        self.assertEqual(expectedMassMat, massMat)
        self.assertEqual(expectedMassMat.mass(), massMat.mass())

    def test_volume_below(self):

        r = 2
        sphere = Sphered(r)

        # Fully below
        plane = Planed(Vector3d(0, 0, 1), Vector2d(4, 4), 2*r)
        self.assertAlmostEqual(sphere.volume(), sphere.volume_below(plane), delta=1e-3)

        # Fully below (because plane is rotated down)
        plane = Planed(Vector3d(0, 0, -1), Vector2d(4, 4), 2*r)
        self.assertAlmostEqual(sphere.volume(), sphere.volume_below(plane), delta=1e-3)

        # Fully above
        plane = Planed(Vector3d(0, 0, 1), Vector2d(4, 4), -2*r)
        self.assertAlmostEqual(sphere.volume_below(plane), 0, 1e-3)

        # Hemisphere
        plane = Planed(Vector3d(0, 0, 1), 0)
        self.assertAlmostEqual(sphere.volume() / 2, sphere.volume_below(plane), delta=1e-3)

        # Vertical plane
        plane = Planed(Vector3d(1, 0, 0), 0)
        self.assertAlmostEqual(sphere.volume() / 2, sphere.volume_below(plane), delta=1e-3)

        # Expectations from https:#planetcalc.com/283/
        plane = Planed(Vector3d(0, 0, 1), 0.5)
        self.assertAlmostEqual(22.90745, sphere.volume_below(plane), delta=1e-3)

        plane = Planed(Vector3d(0, 0, 1), -0.5)
        self.assertAlmostEqual(10.60288, sphere.volume_below(plane), delta=1e-3)

    def test_center_of_volume_below(self):
        r = 2
        sphere = Sphered(r)

        # Entire sphere below plane
        plane = Planed(Vector3d(0, 0, 1), Vector2d(0, 0), 2 * r)
        self.assertEqual(Vector3d(0, 0, 0), sphere.center_of_volume_below(plane))

        # Entire sphere above plane
        plane = Planed(Vector3d(0, 0, 1), Vector2d(0, 0), -2 * r)
        self.assertFalse(sphere.center_of_volume_below(plane) is not None)

        # Halfway point is a good spot to test. Center of Volume for a hemisphere
        # is 3/8 its radius. In this case the point should fall below the y-plane
        plane = Planed(Vector3d(0, 1, 0), Vector2d(0, 0), 0)
        self.assertEqual(Vector3d(0, -0.75, 0), sphere.center_of_volume_below(plane))

        # Halfway point is a good spot to test. Center of Volume for a hemisphere
        # is 3/8 its radius. In this case the point should fall above the y-plane
        # thanks to flipped normal
        plane = Planed(Vector3d(0, -1, 0), Vector2d(0, 0), 0)
        self.assertEqual(Vector3d(0, 0.75, 0), sphere.center_of_volume_below(plane))

        # Handcalculated value.
        # Plane at y = 0.8 pointing upwards
        # Cap height is 2.8
        # Centroid should be at 0.3375. However, keep in mind this assumes an
        # inverted cap.
        # Center of volume below should be at -0.3375
        plane = Planed(Vector3d(0, 1, 0), Vector2d(0, 0), 0.4 * r)
        self.assertEqual(Vector3d(0, -0.3375, 0), sphere.center_of_volume_below(plane))

        # Handcalculated value.
        plane = Planed(Vector3d(0, 1, 0), Vector2d(0, 0), -0.4 * r)

        self.assertEqual(Vector3d(0, -1.225, 0), sphere.center_of_volume_below(plane))

        # Handcalculated value.
        plane = Planed(Vector3d(0, -1, 0), Vector2d(0, 0), -0.4 * r)

        self.assertEqual(Vector3d(0, 1.225, 0), sphere.center_of_volume_below(plane))

        # Handcalculated value.
        plane = Planed(Vector3d(0, -1, 0), Vector2d(0, 0), 0.4 * r)

        self.assertEqual(Vector3d(0, 0.3375, 0), sphere.center_of_volume_below(plane))

        # Weighted sums of the center of volume results in (0,0,0).
        plane1 = Planed(Vector3d(0, 0, 1), -0.5)
        # Flip plane1 axis
        plane2 = Planed(Vector3d(0, 0, -1), -0.5)
        self.assertEqual(
            sphere.center_of_volume_below(plane1) * sphere.volume_below(plane1) +
            sphere.center_of_volume_below(plane2) *
            sphere.volume_below(plane2),
            Vector3d(0, 0, 0))


if __name__ == '__main__':
    unittest.main()
