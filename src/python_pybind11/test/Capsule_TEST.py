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
from ignition.math import Capsuled, Material, MassMatrix3d

import math

class TestBox(unittest.TestCase):

    def test_constructor(self):
        capsule = Capsuled();
        self.assertEqual(0.0, capsule.length());
        self.assertEqual(0.0, capsule.radius());
        self.assertEqual(Material(), capsule.material());

        capsule2 = Capsuled();
        self.assertEqual(capsule, capsule2);

        # Length and radius constructor
        capsule = Capsuled(1.0, 2.0);
        self.assertEqual(1.0, capsule.length());
        self.assertEqual(2.0, capsule.radius());
        self.assertEqual(Material(), capsule.material());

        capsule2 = Capsuled(1.0, 2.0);
        self.assertEqual(capsule, capsule2);

        # Length, radius, mat
        capsule = Capsuled(1.0, 2.0,
            Material(ignition.math.MaterialType.WOOD));
        self.assertEqual(1.0, capsule.length());
        self.assertEqual(2.0, capsule.radius());
        self.assertEqual(Material(ignition.math.MaterialType.WOOD),
                         capsule.material());

        capsule2 = Capsuled(1.0, 2.0,
            Material(ignition.math.MaterialType.WOOD));
        self.assertEqual(capsule, capsule2);


    def test_mutators(self):
        capsule = Capsuled();
        self.assertEqual(0.0, capsule.length());
        self.assertEqual(0.0, capsule.radius());
        self.assertEqual(Material(), capsule.material());

        capsule.set_length(100.1);
        capsule.set_radius(.123);
        capsule.set_material(Material(ignition.math.MaterialType.PINE));

        self.assertEqual(100.1, capsule.length());
        self.assertEqual(.123, capsule.radius());
        self.assertEqual(Material(ignition.math.MaterialType.PINE),
                         capsule.material());


    def test_volume_and_density(self):
        mass = 1.0;
        capsule = Capsuled(1.0, 0.001);
        expectedVolume = (math.pi * math.pow(0.001, 2) * (1.0 + 4./3. * 0.001));
        self.assertEqual(expectedVolume, capsule.volume());

        expectedDensity = mass / expectedVolume;
        self.assertEqual(expectedDensity, capsule.density_from_mass(mass));

        # Bad density
        capsule2 = Capsuled();
        self.assertTrue(math.isnan(capsule2.density_from_mass(mass)));


    def test_mass(self):
      mass = 2.0;
      l = 2.0;
      r = 0.1;
      capsule = Capsuled(l, r);
      capsule.set_density_from_mass(mass);

      cylinderVolume = math.pi * r*r * l;
      sphereVolume = math.pi * 4. / 3. * r*r*r;
      volume = cylinderVolume + sphereVolume;
      cylinderMass = mass * cylinderVolume / volume;
      sphereMass = mass * sphereVolume / volume;

      # expected values based on formula used in Open Dynamics Engine
      # https://bitbucket.org/odedevs/ode/src/0.16.2/ode/src/mass.cpp#lines-148:153
      # and the following article:
      # https://www.gamedev.net/tutorials/_/technical/math-and-physics/capsule-inertia-tensor-r3856/
      ixxIyy = (1/12.0) * cylinderMass * (3*r*r + l*l) + sphereMass * (0.4*r*r + 0.375*r*l + 0.25*l*l);
      izz = r*r * (0.5 * cylinderMass + 0.4 * sphereMass);

      expectedMassMat = MassMatrix3d();
      expectedMassMat.set_inertia_matrix(ixxIyy, ixxIyy, izz, 0.0, 0.0, 0.0);
      expectedMassMat.set_mass(mass);

      massMat = capsule.mass_matrix();
      self.assertEqual(expectedMassMat, massMat);
      self.assertEqual(expectedMassMat.diagonal_moments(), massMat.diagonal_moments());
      self.assertEqual(expectedMassMat.mass(), massMat.mass());

if __name__ == '__main__':
    unittest.main()
