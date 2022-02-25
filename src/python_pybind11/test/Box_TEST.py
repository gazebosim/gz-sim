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

import unittest

import ignition
from ignition.math import Boxd, MassMatrix3d, Material, Planed, Vector3d


class TestBox(unittest.TestCase):

    def test_constructor(self):
        # Default constructor
        box = Boxd()
        self.assertEqual(Vector3d.ZERO, box.size())
        self.assertEqual(Material(), box.material())

        box2 = Boxd()
        self.assertEqual(box, box2)

        # Individual dimension constructor
        box = Boxd(1.0, 2.0, 3.0)
        self.assertEqual(Vector3d(1.0, 2.0, 3.0), box.size())
        self.assertEqual(Material(), box.material())

        box2 = Boxd(1.0, 2.0, 3.0)
        self.assertEqual(box, box2)

        # Vector dimension constructor
        box = Boxd(Vector3d(1.3, 2.5, 4.6))
        self.assertEqual(Vector3d(1.3, 2.5, 4.6), box.size())
        self.assertEqual(Material(), box.material())

        box2 = Boxd(Vector3d(1.3, 2.5, 4.6))
        self.assertEqual(box, box2)

        # Dimension and mat constructor
        box = Boxd(1.0, 2.0, 5.0, Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(Vector3d(1.0, 2.0, 5.0), box.size())
        self.assertEqual(Material(ignition.math.MaterialType.WOOD), box.material())

        box2 = Boxd(1.0, 2.0, 5.0, Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(box, box2)

        # Vector Dimension and mat constructor
        box = Boxd(Vector3d(2.2, 2.0, 10.0), Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(Vector3d(2.2, 2.0, 10.0), box.size())
        self.assertEqual(Material(ignition.math.MaterialType.WOOD), box.material())

        box2 = Boxd(Vector3d(2.2, 2.0, 10.0), Material(ignition.math.MaterialType.WOOD))
        self.assertEqual(box, box2)

    def test_mutators(self):
        box = Boxd()
        box.set_size(100.1, 2.3, 5.6)
        box.set_material(Material(ignition.math.MaterialType.PINE))

        self.assertEqual(100.1, box.size().x())
        self.assertEqual(2.3, box.size().y())
        self.assertEqual(5.6, box.size().z())
        self.assertEqual(Material(ignition.math.MaterialType.PINE), box.material())

        box.set_size(Vector3d(3.4, 1.2, 0.5))
        self.assertEqual(3.4, box.size().x())
        self.assertEqual(1.2, box.size().y())
        self.assertEqual(0.5, box.size().z())

    def test_volume_and_density(self):
        mass = 1.0
        box = Boxd(1.0, 0.1, 10.4)
        expectedVolume = 1.0 * 0.1 * 10.4
        self.assertEqual(expectedVolume, box.volume())

        expectedDensity = mass / expectedVolume
        self.assertEqual(expectedDensity, box.density_from_mass(mass))

        # Bad density
        box2 = Boxd()
        self.assertGreater(0.0, box2.density_from_mass(mass))

    def test_intersections(self):
        # No intersections
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), -5.0)
        self.assertEqual(0, len(box.intersections(plane)))

        # Plane crosses 4 edges
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), 0.0)

        intersections = box.intersections(plane)
        self.assertEqual(4, len(intersections))
        self.assertEqual(intersections.count(Vector3d(-1.0, -1.0, 0.0)), 1)
        self.assertEqual(intersections.count(Vector3d(-1.0, 1.0, 0.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, -1.0, 0.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, 1.0, 0.0)), 1)

        # Plane coincides with box's face
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), 1.0)

        intersections = box.intersections(plane)
        self.assertEqual(4, len(intersections))
        self.assertEqual(intersections.count(Vector3d(-1.0, -1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(-1.0, 1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, -1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, 1.0, 1.0)), 1)

        # 3 intersections
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(1.0, 1.0, 1.0), 1.0)

        intersections = box.intersections(plane)
        self.assertEqual(3, len(intersections))
        self.assertEqual(intersections.count(Vector3d(1.0, -1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(-1.0, 1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, 1.0, -1.0)), 1)

        # 6 intersections
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(1.0, 1.0, 1.0), 0.5)

        intersections = box.intersections(plane)
        self.assertEqual(6, len(intersections))
        self.assertEqual(intersections.count(Vector3d(-1.0, 1.0, 0.5)), 1)
        self.assertEqual(intersections.count(Vector3d(-1.0, 0.5, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, -1.0, 0.5)), 1)
        self.assertEqual(intersections.count(Vector3d(0.5, -1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, 0.5, -1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(0.5, 1.0, -1.0)), 1)

        # 5 intersections
        # This is the plane above tilted further up
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(1.0, 1.0, 2.0), 0.5)

        intersections = box.intersections(plane)
        self.assertEqual(5, len(intersections))
        self.assertEqual(intersections.count(Vector3d(-1.0, 1.0, 0.25)), 1)
        self.assertEqual(intersections.count(Vector3d(-1.0, -0.5, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, -1.0, 0.25)), 1)
        self.assertEqual(intersections.count(Vector3d(-0.5, -1.0, 1.0)), 1)
        self.assertEqual(intersections.count(Vector3d(1.0, 1.0, -0.75)), 1)

    def test_volume_below(self):
        # Fully above
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), -5.0)
        self.assertEqual(0.0, box.volume_below(plane))

        # Fully below
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), 20.0)
        self.assertEqual(box.volume(), box.volume_below(plane))

        # Fully below (because plane is rotated down)
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, -1.0), 20.0)
        self.assertEqual(box.volume(), box.volume_below(plane))

        # Cut in half
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0, 0, 1.0), 0)

        self.assertEqual(box.volume()/2, box.volume_below(plane))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0, 1, 0), 0)

        self.assertEqual(box.volume()/2, box.volume_below(plane))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(-1, 0, 0), 0)

        self.assertEqual(box.volume()/2, box.volume_below(plane))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(-1, -1, 0), 0)

        self.assertEqual(box.volume()/2, box.volume_below(plane))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0, 1, 1), 0)

        self.assertEqual(box.volume()/2, box.volume_below(plane))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(1, 1, 1), 0)

        self.assertAlmostEqual(box.volume()/2, box.volume_below(plane), delta=1e-15)

        # Cut in 3/4
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0, 0, 1.0), 0.5)

        self.assertEqual(3*box.volume()/4, box.volume_below(plane))

        # Opposites add to the total volume
        box = Boxd(2.0, 2.0, 2.0)
        planeA = Planed(Vector3d(0, 0, 1.0), 0.5)
        planeB = Planed(Vector3d(0, 0, 1.0), -0.5)

        self.assertEqual(box.volume(),
                         box.volume_below(planeA) + box.volume_below(planeB))

        box = Boxd(2.0, 2.0, 2.0)
        planeA = Planed(Vector3d(0, 1.0, 1.0), 0.5)
        planeB = Planed(Vector3d(0, 1.0, 1.0), -0.5)

        self.assertEqual(box.volume(),
                         box.volume_below(planeA) + box.volume_below(planeB))

        box = Boxd(2.0, 2.0, 2.0)
        planeA = Planed(Vector3d(-1, 1.0, 1.0), 0.5)
        planeB = Planed(Vector3d(-1, 1.0, 1.0), -0.5)

        self.assertEqual(box.volume(),
                         box.volume_below(planeA) + box.volume_below(planeB))

    def test_center_of_volume_below(self):
        # Fully above
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), -5.0)
        self.assertFalse(box.center_of_volume_below(plane) is not None)

        # Fully below
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), 5.0)
        self.assertTrue(box.center_of_volume_below(plane) is not None)
        self.assertEqual(box.center_of_volume_below(plane), Vector3d(0, 0, 0))

        # Cut in half
        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, 1.0), 0)
        self.assertTrue(box.center_of_volume_below(plane) is not None)
        self.assertEqual(box.center_of_volume_below(plane),
                         Vector3d(0, 0, -0.5))

        box = Boxd(2.0, 2.0, 2.0)
        plane = Planed(Vector3d(0.0, 0.0, -1.0), 0)
        self.assertTrue(box.center_of_volume_below(plane) is not None)
        self.assertEqual(box.center_of_volume_below(plane),
                         Vector3d(0, 0, 0.5))

    def test_vertices_below(self):

        box = Boxd(2.0, 2.0, 2.0)
        size = box.size()

        pXpYpZ = Vector3d(size.x()/2,  size.y()/2,  size.z()/2)
        nXpYpZ = Vector3d(-size.x()/2,  size.y()/2,  size.z()/2)
        pXnYpZ = Vector3d(size.x()/2, -size.y()/2,  size.z()/2)
        nXnYpZ = Vector3d(-size.x()/2, -size.y()/2,  size.z()/2)
        pXpYnZ = Vector3d(size.x()/2,  size.y()/2, -size.z()/2)
        nXpYnZ = Vector3d(-size.x()/2,  size.y()/2, -size.z()/2)
        pXnYnZ = Vector3d(size.x()/2, -size.y()/2, -size.z()/2)
        nXnYnZ = Vector3d(-size.x()/2, -size.y()/2, -size.z()/2)

        # Fully above
        plane = Planed(Vector3d(0.0, 0.0, 1.0), -5.0)
        self.assertTrue(len(box.vertices_below(plane)) == 0)

        # Fully below

        plane = Planed(Vector3d(0.0, 0.0, 1.0), 20.0)
        vertices = box.vertices_below(plane)
        self.assertEqual(8, len(vertices))

        # Fully below (because plane is rotated down)

        plane = Planed(Vector3d(0.0, 0.0, -1.0), 20.0)
        self.assertEqual(8, len(box.vertices_below(plane)))

        # 4 vertices

        plane = Planed(Vector3d(0, 0, 1.0), 0)

        vertices = box.vertices_below(plane)
        self.assertEqual(4, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)
        self.assertEqual(vertices.count(nXpYnZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXpYnZ), 1)

        plane = Planed(Vector3d(0, 1, 0), 0.5)

        vertices = box.vertices_below(plane)
        self.assertEqual(4, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)
        self.assertEqual(vertices.count(nXnYpZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXnYpZ), 1)

        plane = Planed(Vector3d(-1, 0, 0), -0.5)

        vertices = box.vertices_below(plane)
        self.assertEqual(4, len(vertices))

        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXnYpZ), 1)
        self.assertEqual(vertices.count(pXpYnZ), 1)
        self.assertEqual(vertices.count(pXpYpZ), 1)

        plane = Planed(Vector3d(1, 1, 1), 0.0)

        vertices = box.vertices_below(plane)
        self.assertEqual(4, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)
        self.assertEqual(vertices.count(nXnYpZ), 1)
        self.assertEqual(vertices.count(nXpYnZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)

        # 6 vertices
        plane = Planed(Vector3d(-1, -1, 0), 0.3)

        vertices = box.vertices_below(plane)
        self.assertEqual(6, len(vertices))

        self.assertEqual(vertices.count(nXpYnZ), 1)
        self.assertEqual(vertices.count(nXpYpZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXnYpZ), 1)
        self.assertEqual(vertices.count(pXpYnZ), 1)
        self.assertEqual(vertices.count(pXpYpZ), 1)

        plane = Planed(Vector3d(0, 1, 1), 0.9)

        vertices = box.vertices_below(plane)
        self.assertEqual(6, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)
        self.assertEqual(vertices.count(nXnYpZ), 1)
        self.assertEqual(vertices.count(pXnYpZ), 1)
        self.assertEqual(vertices.count(nXpYnZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXpYnZ), 1)

        # 2 vertices
        plane = Planed(Vector3d(-1, -1, 0), -0.5)

        vertices = box.vertices_below(plane)
        self.assertEqual(2, len(vertices))

        self.assertEqual(vertices.count(pXpYnZ), 1)
        self.assertEqual(vertices.count(pXpYpZ), 1)

        # 7 vertices
        plane = Planed(Vector3d(1, 1, 1), 1.0)

        vertices = box.vertices_below(plane)
        self.assertEqual(7, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)
        self.assertEqual(vertices.count(nXnYpZ), 1)
        self.assertEqual(vertices.count(pXnYpZ), 1)
        self.assertEqual(vertices.count(nXpYnZ), 1)
        self.assertEqual(vertices.count(nXpYpZ), 1)
        self.assertEqual(vertices.count(pXnYnZ), 1)
        self.assertEqual(vertices.count(pXpYnZ), 1)

        # 1 vertex
        plane = Planed(Vector3d(1, 1, 1), -1.2)

        vertices = box.vertices_below(plane)
        self.assertEqual(1, len(vertices))

        self.assertEqual(vertices.count(nXnYnZ), 1)

    def test_mass(self):
        mass = 2.0
        length = 2.0
        w = 0.1
        h = 34.12
        box = Boxd(length, w, h)
        box.set_density_from_mass(mass)

        massMat = MassMatrix3d()
        ixx = (1.0/12.0) * mass * (w*w + h*h)
        iyy = (1.0/12.0) * mass * (length * length + h*h)
        izz = (1.0/12.0) * mass * (length * length + w*w)

        expectedMassMat = MassMatrix3d()
        expectedMassMat.set_inertia_matrix(ixx, iyy, izz, 0.0, 0.0, 0.0)
        expectedMassMat.set_mass(mass)

        box.mass_matrix(massMat)
        self.assertEqual(expectedMassMat, massMat)
        self.assertEqual(expectedMassMat.mass(), massMat.mass())


if __name__ == '__main__':
    unittest.main()
