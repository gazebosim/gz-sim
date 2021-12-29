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

from ignition.math import Angle, AxisAlignedBox, Frustum, FrustumPlane, Planed, Pose3d, Vector3d


class TestFrustrum(unittest.TestCase):

    def test_constructor(self):
        frustum = Frustum()

        self.assertEqual(frustum.near(), 0.0)
        self.assertEqual(frustum.far(), 1.0)
        self.assertEqual(frustum.fov().radian(), 45 * math.pi / 180.0)
        self.assertEqual(frustum.aspect_ratio(), 1.0)
        self.assertEqual(frustum.pose(), Pose3d.ZERO)

    def test_copy_constructor(self):

        # Frustum pointing down the +x axis
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          320.0 / 240.0,
          # Pose
          Pose3d(0, 0, 0, 0, 0, 0))

        frustum2 = Frustum(frustum)

        self.assertEqual(frustum.fov(), frustum2.fov())
        self.assertEqual(frustum.near(), frustum2.near())
        self.assertEqual(frustum.far(), frustum2.far())
        self.assertEqual(frustum.aspect_ratio(), frustum2.aspect_ratio())
        self.assertEqual(frustum.aspect_ratio(), frustum2.aspect_ratio())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_NEAR).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_NEAR).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_FAR).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_FAR).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_LEFT).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_LEFT).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_RIGHT).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_RIGHT).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_TOP).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_TOP).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_BOTTOM).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_BOTTOM).normal())

    def test_assignment_operator(self):
        # Frustum pointing to the +X+Y diagonal
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          320.0/240.0,
          # Pose
          Pose3d(0, 0, 0, 0, 0, 45 * math.pi / 180.0))

        frustum2 = Frustum()
        frustum2 = frustum

        self.assertEqual(frustum.fov(), frustum2.fov())
        self.assertEqual(frustum.near(), frustum2.near())
        self.assertEqual(frustum.far(), frustum2.far())
        self.assertEqual(frustum.aspect_ratio(), frustum2.aspect_ratio())
        self.assertEqual(frustum.aspect_ratio(), frustum2.aspect_ratio())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_NEAR).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_NEAR).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_FAR).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_FAR).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_LEFT).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_LEFT).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_RIGHT).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_RIGHT).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_TOP).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_TOP).normal())

        self.assertEqual(frustum.plane(FrustumPlane.FRUSTUM_PLANE_BOTTOM).normal(),
                         frustum2.plane(FrustumPlane.FRUSTUM_PLANE_BOTTOM).normal())

    def test_pyramid_x_axis_pos(self):
        # Frustum pointing down the +x axis
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          320.0/240.0,
          # Pose
          Pose3d(0, 0, 0, 0, 0, 0))

        self.assertFalse(frustum.contains(Vector3d(0, 0, 0)))
        self.assertTrue(frustum.contains(Vector3d(1, 0, 0)))

        self.assertTrue(frustum.contains(Vector3d(2, 0, 0)))
        self.assertTrue(frustum.contains(Vector3d(10, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(10.1, 0, 0)))

        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1, 0, 0), Vector3d(5, 5, 5))))
        self.assertFalse(frustum.contains(
                         AxisAlignedBox(Vector3d(-1, 0, 0), Vector3d(.1, .2, .3))))

    def test_pyramid_x_axis_neg(self):
        # Frustum pointing down the -x axis
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          320.0/240.0,
          # Pose
          Pose3d(0, 0, 0, 0, 0, math.pi))

        self.assertFalse(frustum.contains(Vector3d(0, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(-0.5, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(-10.1, 0, 0)))

        self.assertTrue(frustum.contains(Vector3d(-1, 0, 0)))
        self.assertTrue(frustum.contains(Vector3d(-2, 0, 0)))
        self.assertTrue(frustum.contains(Vector3d(-10, 0, 0)))

        self.assertFalse(frustum.contains(
            AxisAlignedBox(Vector3d(1, 0, 0), Vector3d(5, 5, 5))))
        self.assertTrue(frustum.contains(
            AxisAlignedBox(Vector3d(-1, 0, 0), Vector3d(.1, .2, .3))))

    def test_pyramid_y_axis(self):
        # Frustum pointing down the +y axis
        frustum = Frustum(
          # Near distance
          .1,
          # Far distance
          5,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, 0, math.pi*0.5))

        self.assertFalse(frustum.contains(Vector3d(0, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(1, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(.05, 0, 0)))

        self.assertTrue(frustum.contains(Vector3d(0, .1, 0)))
        self.assertTrue(frustum.contains(Vector3d(0, 1, 0)))
        self.assertTrue(frustum.contains(Vector3d(0, 5, 0)))

        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0, 1, 0), Vector3d(5, 5, 5))))
        self.assertFalse(frustum.contains(
                         AxisAlignedBox(Vector3d(0, -1, 0), Vector3d(.1, 0, .3))))

    def test_pyramid_z_axis(self):
        # Frustum pointing down the -z axis
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        self.assertFalse(frustum.contains(Vector3d(0, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(0, 0, -0.9)))
        self.assertFalse(frustum.contains(Vector3d(0, 0, -10.5)))
        self.assertFalse(frustum.contains(Vector3d(0, 0, 0.9)))
        self.assertFalse(frustum.contains(Vector3d(0, 0, 10.5)))

        self.assertTrue(frustum.contains(Vector3d(0, 0, -1.1)))
        self.assertTrue(frustum.contains(Vector3d(0.5, 0.5, -5.5)))
        self.assertTrue(frustum.contains(Vector3d(0, 0, -10)))

        self.assertFalse(frustum.contains(
                         AxisAlignedBox(Vector3d(0, 0, 0), Vector3d(5, 5, 5))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0, 0, -1), Vector3d(.1, 0, .3))))

    def test_near_far(self):
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        self.assertEqual(frustum.near(), 1.0)
        self.assertEqual(frustum.far(), 10.0)

        frustum.set_near(-1.0)
        frustum.set_far(-10.0)

        self.assertEqual(frustum.near(), -1.0)
        self.assertEqual(frustum.far(), -10.0)

    def test_fov(self):
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        self.assertEqual(frustum.fov(), Angle(45 * math.pi / 180.0))

        frustum.set_fov(Angle(1.5707))

        self.assertEqual(frustum.fov(), Angle(1.5707))

    def test_aspect_ratio(self):
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        self.assertEqual(frustum.aspect_ratio(), 1)

        frustum.set_aspect_ratio(1.3434)

        self.assertEqual(frustum.aspect_ratio(), 1.3434)

    def test_pose(self):

        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(45 * math.pi / 180.0),
          # Aspect ratio
          1.0,
          # Pose
          Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        self.assertEqual(frustum.pose(), Pose3d(0, 0, 0, 0, math.pi*0.5, 0))

        frustum.set_pose(Pose3d(1, 2, 3, math.pi, 0, 0))

        self.assertEqual(frustum.pose(), Pose3d(1, 2, 3, math.pi, 0, 0))

    def test_pose_contains(self):
        frustum = Frustum(
          # Near distance
          1,
          # Far distance
          10,
          # Field of view
          Angle(60 * math.pi / 180.0),
          # Aspect ratio
          1920.0/1080.0,
          # Pose
          Pose3d(0, -5, 0, 0, 0, math.pi*0.5))

        # Test the near clip boundary
        self.assertFalse(frustum.contains(Vector3d(0, -4.01, 0)))
        self.assertTrue(frustum.contains(Vector3d(0, -4.0, 0)))

        # Test a point between the near and far clip planes
        self.assertTrue(frustum.contains(Vector3d(0, 1, 0)))

        # Test the far clip boundary
        self.assertTrue(frustum.contains(Vector3d(0, 5, 0)))
        self.assertFalse(frustum.contains(Vector3d(0, 5.001, 0)))

        # Use an offset for the test points. This makes the test more stable, and
        # is also used to generate point outside the frustum.
        offset = 0.00001

        # Compute near clip points
        nearTopLeft = Vector3d(
          -math.tan(30 * math.pi / 180.0) + offset,
          frustum.pose().pos().y() + frustum.near() + offset,
          math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() - offset)

        nearTopLeftBad = Vector3d(
          -math.tan(30 * math.pi / 180.0) - offset,
          frustum.pose().pos().y() + frustum.near() - offset,
          math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() + offset)

        nearTopRight = Vector3d(
          math.tan(30 * math.pi / 180.0) - offset,
          frustum.pose().pos().y() + frustum.near() + offset,
          math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() - offset)

        nearTopRightBad = Vector3d(
          math.tan(30 * math.pi / 180.0) + offset,
          frustum.pose().pos().y() + frustum.near() - offset,
          math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() + offset)

        nearBottomLeft = Vector3d(
          -math.tan(30 * math.pi / 180.0) + offset,
          frustum.pose().pos().y() + frustum.near() + offset,
          -math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() + offset)

        nearBottomLeftBad = Vector3d(
          -math.tan(30 * math.pi / 180.0) - offset,
          frustum.pose().pos().y() + frustum.near() - offset,
          -math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() - offset)

        nearBottomRight = Vector3d(
          math.tan(30 * math.pi / 180.0) - offset,
          frustum.pose().pos().y() + frustum.near() + offset,
          -math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() + offset)

        nearBottomRightBad = Vector3d(
          math.tan(30 * math.pi / 180.0) + offset,
          frustum.pose().pos().y() + frustum.near() - offset,
          -math.tan(30 * math.pi / 180.0) / frustum.aspect_ratio() - offset)

        # Test near clip corners
        self.assertTrue(frustum.contains(nearTopLeft))
        self.assertFalse(frustum.contains(nearTopLeftBad))

        self.assertTrue(frustum.contains(nearTopRight))
        self.assertFalse(frustum.contains(nearTopRightBad))

        self.assertTrue(frustum.contains(nearBottomLeft))
        self.assertFalse(frustum.contains(nearBottomLeftBad))

        self.assertTrue(frustum.contains(nearBottomRight))
        self.assertFalse(frustum.contains(nearBottomRightBad))

        # Compute far clip points
        farTopLeft = Vector3d(
          -math.tan(30 * math.pi / 180.0) * frustum.far() + offset,
          frustum.pose().pos().y() + frustum.far() - offset,
          (math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() - offset)

        farTopLeftBad = Vector3d(
          -math.tan(30 * math.pi / 180.0)*frustum.far() - offset,
          frustum.pose().pos().y() + frustum.far() + offset,
          (math.tan(30 * math.pi / 180.0 * frustum.far())) / frustum.aspect_ratio() + offset)

        farTopRight = Vector3d(
          math.tan(30 * math.pi / 180.0)*frustum.far() - offset,
          frustum.pose().pos().y() + frustum.far() - offset,
          (math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() - offset)

        farTopRightBad = Vector3d(
          math.tan(30 * math.pi / 180.0)*frustum.far() + offset,
          frustum.pose().pos().y() + frustum.far() + offset,
          (math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() + offset)

        farBottomLeft = Vector3d(
          -math.tan(30 * math.pi / 180.0)*frustum.far() + offset,
          frustum.pose().pos().y() + frustum.far() - offset,
          (-math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() + offset)

        farBottomLeftBad = Vector3d(
          -math.tan(30 * math.pi / 180.0)*frustum.far() - offset,
          frustum.pose().pos().y() + frustum.far() + offset,
          (-math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() - offset)

        farBottomRight = Vector3d(
          math.tan(30 * math.pi / 180.0)*frustum.far() - offset,
          frustum.pose().pos().y() + frustum.far() - offset,
          (-math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() + offset)

        farBottomRightBad = Vector3d(
          math.tan(30 * math.pi / 180.0)*frustum.far() + offset,
          frustum.pose().pos().y() + frustum.far() + offset,
          (-math.tan(30 * math.pi / 180.0) * frustum.far()) / frustum.aspect_ratio() - offset)

        # Test far clip corners
        self.assertTrue(frustum.contains(farTopLeft))
        self.assertFalse(frustum.contains(farTopLeftBad))

        self.assertTrue(frustum.contains(farTopRight))
        self.assertFalse(frustum.contains(farTopRightBad))

        self.assertTrue(frustum.contains(farBottomLeft))
        self.assertFalse(frustum.contains(farBottomLeftBad))

        self.assertTrue(frustum.contains(farBottomRight))
        self.assertFalse(frustum.contains(farBottomRightBad))

        # Adjust to 45 degrees rotation
        frustum.set_pose(Pose3d(1, 1, 0, 0, 0, -math.pi*0.25))
        self.assertTrue(frustum.contains(Vector3d(2, -1, 0)))
        self.assertFalse(frustum.contains(Vector3d(0, 0, 0)))
        self.assertFalse(frustum.contains(Vector3d(1, 1, 0)))

    def test_contains_aabb_no_overlap(self):
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        # AxisAlignedBoxes that don't overlapp any planes
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.05, 1.95), Vector3d(1.55, 0.05, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.55, -0.05, 1.95), Vector3d(2.65, 0.05, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(0.35, -0.05, 1.95), Vector3d(0.45, 0.05, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.05, 2.55), Vector3d(1.55, 0.05, 2.65))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.05, 1.35), Vector3d(1.55, 0.05, 1.45))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -1.05, 1.95), Vector3d(1.55, -0.95, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, 0.95, 1.95), Vector3d(1.55, 1.05, 2.05))))

    def test_contains_aabb_one_plane(self):
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        # AxisAlignedBoxes overlapping one plane
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.43, -0.05, 1.95), Vector3d(2.53, 0.05, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.05, 1.95), Vector3d(0.595, 0.05, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.05, 2.42), Vector3d(1.55, 0.05, 2.52))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.05, 1.48), Vector3d(1.55, 0.05, 1.58))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, -0.9, 1.95), Vector3d(1.55, -0.8, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1.45, 0.8, 1.95), Vector3d(1.55, 0.9, 2.05))))

    def test_contains_aabb_two_planes(self):
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        # AxisAlignedBoxes overlapping two planes
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, -0.05, 2.7), Vector3d(2.52, 0.05, 2.8))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, -0.05, 1.2), Vector3d(2.52, 0.05, 1.3))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, -1.44, 1.95), Vector3d(2.52, -1.34, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, 1.34, 1.95), Vector3d(2.52, 1.44, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.05, 2.1), Vector3d(0.595, 0.05, 2.2))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.05, 1.8), Vector3d(0.595, 0.05, 1.9))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, 0.25, 1.95), Vector3d(0.595, 0.35, 2.05))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.35, 1.95), Vector3d(0.595, -0.25, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, -0.05, 2.81), Vector3d(2.58, 0.05, 2.91))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, -0.05, 1.09), Vector3d(2.58, 0.05, 1.19))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, -1.55, 1.95), Vector3d(2.58, -1.45, 2.05))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, 1.45, 1.95), Vector3d(2.58, 1.55, 2.05))))

    def test_contains_aabb_three_planes(self):
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        # AxisAlignedBoxes overlapping three planes
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, 0.25, 2.1), Vector3d(0.595, 0.35, 2.2))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, 0.25, 1.8), Vector3d(0.595, 0.35, 1.9))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.35, 2.1), Vector3d(0.595, -0.25, 2.2))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(0.495, -0.35, 1.8), Vector3d(0.595, -0.25, 1.9))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, 1.34, 2.7), Vector3d(2.52, 1.44, 2.8))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, 1.34, 1.2), Vector3d(2.52, 1.44, 1.3))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, -1.44, 2.7), Vector3d(2.52, -1.34, 2.8))))
        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(2.42, -1.44, 1.2), Vector3d(2.52, -1.34, 1.3))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, 1.45, 2.81), Vector3d(2.58, 1.55, 2.91))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, -1.55, 2.81), Vector3d(2.58, -1.45, 2.91))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, 1.45, 1.09), Vector3d(2.58, 1.55, 1.19))))
        self.assertFalse(frustum.contains(
                        AxisAlignedBox(Vector3d(2.48, -1.55, 1.09), Vector3d(2.58, -1.45, 1.19))))

    def test_contains_frustum(self):
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(-100, -100, -100), Vector3d(100, 100, 100))))

    def test_AABB_frustum_edge_overlap(self):
        # This test case has the top of an AABB overlap a frustum, but all the
        # corners of AABB fall outside the frustum.

        ybounds = 10

        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        self.assertTrue(frustum.contains(
                        AxisAlignedBox(Vector3d(1, -ybounds, 0), Vector3d(2, ybounds, 2))))

    def test_AABBB_f_wall(self):
        # Frustum contains at a large but thin wall
        frustum = Frustum()
        frustum.set_near(0.55)
        frustum.set_far(2.5)
        frustum.set_fov(Angle(1.05))
        frustum.set_aspect_ratio(1.8)
        frustum.set_pose(Pose3d(0, 0, 2, 0, 0, 0))

        self.assertTrue(frustum.contains(
            AxisAlignedBox(Vector3d(1, -10, -10), Vector3d(2, 10, 10))))
        self.assertTrue(frustum.contains(
            AxisAlignedBox(Vector3d(-10, 1, -10), Vector3d(10, 1.1, 10))))
        self.assertTrue(frustum.contains(
            AxisAlignedBox(Vector3d(-10, -10, 1.95), Vector3d(10, 10, 2.05))))


if __name__ == '__main__':
    unittest.main()
