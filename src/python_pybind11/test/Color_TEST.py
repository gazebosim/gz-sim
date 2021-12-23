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

import math
import unittest
from ignition.math import Color
from ignition.math import Vector3f


class TestColor(unittest.TestCase):

    def test_const_color(self):
        self.assertAlmostEqual(1.0, Color.WHITE.r())
        self.assertAlmostEqual(1.0, Color.WHITE.g())
        self.assertAlmostEqual(1.0, Color.WHITE.b())
        self.assertAlmostEqual(1.0, Color.WHITE.a())

        self.assertAlmostEqual(0.0, Color.BLACK.r())
        self.assertAlmostEqual(0.0, Color.BLACK.g())
        self.assertAlmostEqual(0.0, Color.BLACK.b())
        self.assertAlmostEqual(1.0, Color.BLACK.a())

        self.assertAlmostEqual(1.0, Color.RED.r())
        self.assertAlmostEqual(0.0, Color.RED.g())
        self.assertAlmostEqual(0.0, Color.RED.b())
        self.assertAlmostEqual(1.0, Color.RED.a())

        self.assertAlmostEqual(0.0, Color.GREEN.r())
        self.assertAlmostEqual(1.0, Color.GREEN.g())
        self.assertAlmostEqual(0.0, Color.GREEN.b())
        self.assertAlmostEqual(1.0, Color.GREEN.a())

        self.assertAlmostEqual(0.0, Color.BLUE.r())
        self.assertAlmostEqual(0.0, Color.BLUE.g())
        self.assertAlmostEqual(1.0, Color.BLUE.b())
        self.assertAlmostEqual(1.0, Color.BLUE.a())

        self.assertAlmostEqual(1.0, Color.YELLOW.r())
        self.assertAlmostEqual(1.0, Color.YELLOW.g())
        self.assertAlmostEqual(0.0, Color.YELLOW.b())
        self.assertAlmostEqual(1.0, Color.YELLOW.a())

        self.assertAlmostEqual(1.0, Color.MAGENTA.r())
        self.assertAlmostEqual(0.0, Color.MAGENTA.g())
        self.assertAlmostEqual(1.0, Color.MAGENTA.b())
        self.assertAlmostEqual(1.0, Color.MAGENTA.a())

        self.assertAlmostEqual(0.0, Color.CYAN.r())
        self.assertAlmostEqual(1.0, Color.CYAN.g())
        self.assertAlmostEqual(1.0, Color.CYAN.b())
        self.assertAlmostEqual(1.0, Color.CYAN.a())

    def test_color(self):
        clr0 = Color()
        self.assertAlmostEqual(0.0, clr0.r())
        self.assertAlmostEqual(0.0, clr0.g())
        self.assertAlmostEqual(0.0, clr0.b())
        self.assertAlmostEqual(1.0, clr0.a())
        self.assertEqual(clr0.as_rgba(), 255)
        clr0.a(0.0)
        self.assertEqual(clr0.as_rgba(), 0)

        clr = Color(.1, .2, .3, 1.0)
        self.assertAlmostEqual(0.1, clr.r())
        self.assertAlmostEqual(0.2, clr.g())
        self.assertAlmostEqual(0.3, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set(1, 0, 0, 0)
        self.assertAlmostEqual(clr.as_rgba(), 255 << 24)
        self.assertAlmostEqual(clr.as_bgra(), 255 << 8)
        self.assertAlmostEqual(clr.as_argb(), 255 << 16)
        self.assertAlmostEqual(clr.as_abgr(), 255)
        clr0.set_from_rgba(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.set_from_bgra(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.set_from_argb(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.set_from_abgr(255)
        self.assertEqual(clr0, clr)

        clr.set(0, 1, 0, 0)
        self.assertAlmostEqual(clr.as_rgba(), 255 << 16)
        self.assertAlmostEqual(clr.as_bgra(), 255 << 16)
        self.assertAlmostEqual(clr.as_argb(), 255 << 8)
        self.assertAlmostEqual(clr.as_abgr(), 255 << 8)
        clr0.set_from_rgba(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.set_from_bgra(255 << 16)
        self.assertEqual(clr0, clr)
        clr0.set_from_argb(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.set_from_abgr(255 << 8)
        self.assertEqual(clr0, clr)

        clr.set(0, 0, 1, 0)
        self.assertAlmostEqual(clr.as_rgba(), 255 << 8)
        self.assertAlmostEqual(clr.as_bgra(), 255 << 24)
        self.assertAlmostEqual(clr.as_argb(), 255)
        self.assertAlmostEqual(clr.as_abgr(), 255 << 16)
        clr0.set_from_rgba(255 << 8)
        self.assertEqual(clr0, clr)
        clr0.set_from_bgra(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.set_from_argb(255)
        self.assertEqual(clr0, clr)
        clr0.set_from_abgr(255 << 16)
        self.assertEqual(clr0, clr)

        clr.set(0, 0, 0, 1)
        self.assertAlmostEqual(clr.as_rgba(), 255)
        self.assertAlmostEqual(clr.as_bgra(), 255)
        self.assertAlmostEqual(clr.as_argb(), 255 << 24)
        self.assertAlmostEqual(clr.as_abgr(), 255 << 24)
        clr0.set_from_rgba(255)
        self.assertEqual(clr0, clr)
        clr0.set_from_bgra(255)
        self.assertEqual(clr0, clr)
        clr0.set_from_argb(255 << 24)
        self.assertEqual(clr0, clr)
        clr0.set_from_abgr(255 << 24)
        self.assertEqual(clr0, clr)

        clr.reset()
        self.assertAlmostEqual(0.0, clr.r())
        self.assertAlmostEqual(0.0, clr.g())
        self.assertAlmostEqual(0.0, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set_from_hsv(0, 0.5, 1.0)
        self.assertAlmostEqual(1.0, clr.r())
        self.assertAlmostEqual(0.5, clr.g())
        self.assertAlmostEqual(0.5, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        self.assertTrue(clr.hsv() == Vector3f(360, 0.5, 1))

        clr.set_from_hsv(60, 0.0, 1.0)
        self.assertAlmostEqual(1.0, clr.r())
        self.assertAlmostEqual(1.0, clr.g())
        self.assertAlmostEqual(1.0, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set_from_hsv(120, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.r())
        self.assertAlmostEqual(1.0, clr.g())
        self.assertAlmostEqual(0.5, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set_from_hsv(180, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.r())
        self.assertAlmostEqual(1.0, clr.g())
        self.assertAlmostEqual(1.0, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set_from_hsv(240, 0.5, 1.0)
        self.assertAlmostEqual(0.5, clr.r())
        self.assertAlmostEqual(0.5, clr.g())
        self.assertAlmostEqual(1.0, clr.b())
        self.assertAlmostEqual(1.0, clr.a())

        clr.set_from_hsv(300, 0.5, 1.0)
        self.assertAlmostEqual(1.0, clr[0])
        self.assertAlmostEqual(0.5, clr[1])
        self.assertAlmostEqual(1.0, clr[2])
        self.assertAlmostEqual(1.0, clr[3])
        self.assertTrue(math.isnan(clr[4]))

        clr.set(0.1, 0.2, 0.3, 0.4)
        clr = clr + 0.2
        self.assertTrue(clr == Color(0.3, 0.4, 0.5, 0.6))

        clr.set(0.1, 0.2, 0.3, 0.4)
        clr += Color(0.2, 0.2, 0.2, 0.2)
        self.assertTrue(clr == Color(0.3, 0.4, 0.5, 0.6))

        clr.set(0.1, 0.2, 0.3, 0.4)
        clr = clr - 0.1
        self.assertTrue(clr == Color(0.0, 0.1, 0.2, 0.3))

        clr.set(0.1, 0.2, 0.3, 0.4)
        clr -= Color(0.1, 0.1, 0.1, 0.1)
        self.assertTrue(clr == Color(0.0, 0.1, 0.2, 0.3))

        clr.set(1.0, 1.0, 1.0, 1.0)
        clr = clr / 1.6
        self.assertTrue(clr == Color(0.625, 0.625, 0.625, 0.625))

        clr.set(1.0, 1.0, 1.0, 1.0)
        clr /= Color(1.0, 1.0, 1.0, 1.0)
        self.assertTrue(clr == Color(1.0, 1.0, 1.0, 1.0))

        clr.set(.1, .2, .3, .4)
        clr = clr * .1
        self.assertTrue(clr == Color(0.01, 0.02, 0.03, 0.04))

        clr.set(.1, .2, .3, .4)
        clr *= Color(0.1, 0.1, 0.1, 0.1)
        self.assertTrue(clr == Color(0.01, 0.02, 0.03, 0.04))

        clr.set_from_yuv(0.5, 0.2, 0.8)
        self.assertAlmostEqual(0.00553, clr.r(), delta=1e-3)
        self.assertAlmostEqual(0.0, clr.g())
        self.assertAlmostEqual(0.9064, clr.b(), delta=1e-3)
        self.assertAlmostEqual(0.04, clr.a())

        self.assertTrue(clr.yuv() == Vector3f(0.104985, 0.95227, 0.429305))

        clr = Color(1.0, 0.0, 0.5, 1.0) + Color(0.1, 0.3, 0.4, 1.0)
        self.assertAlmostEqual(0.00431373, clr.r(), delta=1e-4)
        self.assertAlmostEqual(0.3, clr.g(), delta=1e-4)
        self.assertAlmostEqual(0.9, clr.b(), delta=1e-4)
        self.assertAlmostEqual(1.0, clr.a(), delta=1e-4)

        clr = Color(1.0, 0.0, 0.5, 1.0) - Color(0.1, 0.3, 0.4, 1.0)
        self.assertAlmostEqual(0.9, clr.r(), delta=1e-4)
        self.assertAlmostEqual(0.0, clr.g(), delta=1e-4)
        self.assertAlmostEqual(0.1, clr.b(), delta=1e-4)
        self.assertAlmostEqual(0.0, clr.a(), delta=1e-4)

        clr = Color(0.5, 0.2, 0.4, 0.6) / 2.0
        self.assertAlmostEqual(0.25, clr.r(), delta=1e-4)
        self.assertAlmostEqual(0.1, clr.g(), delta=1e-4)
        self.assertAlmostEqual(0.2, clr.b(), delta=1e-4)
        self.assertAlmostEqual(0.3, clr.a(), delta=1e-4)

    def test_mul(self):
        clr = Color(0.0, 0.01, 0.2, 1.0)
        clr2 = Color(1.0, 0.2, 0.2, 0.0)
        clr3 = clr * clr2

        self.assertAlmostEqual(clr3.r(), 0.0)
        self.assertAlmostEqual(clr3.g(), 0.002)
        self.assertAlmostEqual(clr3.b(), 0.04)
        self.assertAlmostEqual(clr3.a(), 0.0)

    def test_division(self):
        clr = Color(0.0, 0.01, 0.2, 1.0)
        clr2 = clr / 0.2
        self.assertAlmostEqual(clr2.r(), 0.0)
        self.assertAlmostEqual(clr2.g(), 0.05)
        self.assertAlmostEqual(clr2.b(), 1.0)
        self.assertAlmostEqual(clr2.a(), 1.0)

        clr2 = clr / 2.0
        self.assertAlmostEqual(clr2.r(), 0.0)
        self.assertAlmostEqual(clr2.g(), 0.005)
        self.assertAlmostEqual(clr2.b(), 0.1)
        self.assertAlmostEqual(clr2.a(), 0.5)

        clr2.set(0.0, 0.2, 0.4, 0.5)
        clr3 = clr / clr2
        self.assertAlmostEqual(clr3.r(), 0.0)
        self.assertAlmostEqual(clr3.g(), 0.05)
        self.assertAlmostEqual(clr3.b(), 0.5)
        self.assertAlmostEqual(clr3.a(), 1.0)

        clr.set(0.0, 0.0, 0.0, 0.0)
        clr2.set(0.0, 0.0, 0.0, 0.0)
        clr3 = clr / clr2
        self.assertAlmostEqual(clr3.r(), 0.0)
        self.assertAlmostEqual(clr3.g(), 0.0)
        self.assertAlmostEqual(clr3.b(), 0.0)
        self.assertAlmostEqual(clr3.a(), 0.0)

    def test_const_set(self):
        clr = Color(0.1, 0.2, 0.3, 0.4)
        self.assertAlmostEqual(clr.r(), 0.1)
        self.assertAlmostEqual(clr.g(), 0.2)
        self.assertAlmostEqual(clr.b(), 0.3)
        self.assertAlmostEqual(clr.a(), 0.4)

        clr2 = Color()
        clr2.r(0.4)
        clr2.g(0.3)
        clr2.b(0.2)
        clr2.a(0.1)
        self.assertAlmostEqual(clr2.r(), 0.4)
        self.assertAlmostEqual(clr2.g(), 0.3)
        self.assertAlmostEqual(clr2.b(), 0.2)
        self.assertAlmostEqual(clr2.a(), 0.1)

        self.assertTrue(clr2 != clr)

    def test_stream_out(self):
        c = Color(0.1, 0.2, 0.3, 0.5)
        self.assertEqual(str(c), "0.1 0.2 0.3 0.5")

    def test_HSV(self):
        clr = Color()
        hsv = clr.hsv()
        self.assertAlmostEqual(hsv.x(), 0.0)
        self.assertAlmostEqual(hsv.y(), 0.0)
        self.assertAlmostEqual(hsv.z(), 0.0)

        clr.set(0.1, 0.2, 0.3, 1.0)
        hsv = clr.hsv()
        self.assertAlmostEqual(hsv.x(), 210, delta=1e-3)
        self.assertAlmostEqual(hsv.y(), 0.666667, delta=1e-3)
        self.assertAlmostEqual(hsv.z(), 0.3, delta=1e-3)

        clr.set(0.3, 0.2, 0.1, 1.0)
        hsv = clr.hsv()
        self.assertAlmostEqual(hsv.x(), 30, delta=1e-3)
        self.assertAlmostEqual(hsv.y(), 0.666667, delta=1e-3)
        self.assertAlmostEqual(hsv.z(), 0.3, delta=1e-3)

        clr.set_from_hsv(60, 10, 5)
        self.assertAlmostEqual(clr.r(), 0.0196078, delta=1e-3)
        self.assertAlmostEqual(clr.g(), 0.0196078, delta=1e-3)
        self.assertAlmostEqual(clr.b(), 0.0, delta=1e-3)
        self.assertAlmostEqual(clr.a(), 1.0, delta=1e-3)

        clr.set_from_hsv(360.0, 0.5, 0.6)
        self.assertAlmostEqual(clr.r(), 0.6, delta=1e-3)
        self.assertAlmostEqual(clr.g(), 0.3, delta=1e-3)
        self.assertAlmostEqual(clr.b(), 0.3, delta=1e-3)
        self.assertAlmostEqual(clr.a(), 1.0, delta=1e-3)


if __name__ == '__main__':
    unittest.main()
