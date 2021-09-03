# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, version 2.0 (the "License")
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
from ignition.math import SemanticVersion


class TestSemanticVersion(unittest.TestCase):

    def test_prerelease(self):
        a = SemanticVersion("0.1.0")
        b = SemanticVersion("0.1.0-pr2")

        self.assertTrue(b < a)
        self.assertFalse(a.prerelease())
        self.assertEqual(b.prerelease(), "pr2")

        self.assertEqual(b.major(), a.major())
        self.assertEqual(b.minor(), a.minor())
        self.assertEqual(b.patch(), a.patch())
        self.assertEqual(b.build(), a.build())

        self.assertEqual(a.version(), "0.1.0")
        self.assertEqual(b.version(), "0.1.0-pr2")

    def test_build(self):
        a = SemanticVersion("0.1.0")
        b = SemanticVersion("0.1.0+012345")

        self.assertTrue(b == a)
        self.assertFalse(a.build())
        self.assertEqual(b.build(), "012345")

        self.assertEqual(b.major(), a.major())
        self.assertEqual(b.minor(), a.minor())
        self.assertEqual(b.patch(), a.patch())
        self.assertEqual(b.prerelease(), a.prerelease())

        self.assertEqual(a.version(), "0.1.0")
        self.assertEqual(b.version(), "0.1.0+012345")

    def test_prerelease_build(self):
        a = SemanticVersion("0.1.0")
        b = SemanticVersion("0.1.0-pr2")
        c = SemanticVersion("0.1.0+012345")
        d = SemanticVersion("0.1.0-pr2+012345")

        self.assertEqual(a.version(), "0.1.0")
        self.assertEqual(b.version(), "0.1.0-pr2")
        self.assertEqual(c.version(), "0.1.0+012345")
        self.assertEqual(d.version(), "0.1.0-pr2+012345")

        self.assertTrue(b < a)
        self.assertTrue(b < c)
        self.assertTrue(b == d)
        self.assertTrue(a == c)

        self.assertEqual(a.major(), b.major())
        self.assertEqual(a.minor(), b.minor())
        self.assertEqual(a.patch(), b.patch())
        self.assertFalse(a.prerelease())
        self.assertFalse(a.build())

        self.assertEqual(b.major(), c.major())
        self.assertEqual(b.minor(), c.minor())
        self.assertEqual(b.patch(), c.patch())
        self.assertEqual(b.prerelease(), d.prerelease())

        self.assertEqual(c.major(), d.major())
        self.assertEqual(c.minor(), d.minor())
        self.assertEqual(c.patch(), d.patch())
        self.assertEqual(c.build(), d.build())

        self.assertEqual(d.build(), "012345")
        self.assertEqual(d.prerelease(), "pr2")

    def test_sream_out(self):
        a = SemanticVersion("0.1.0")
        b = SemanticVersion("0.1.0-pr2")
        c = SemanticVersion("0.1.0+012345")
        d = SemanticVersion("0.1.0-pr2+012345")

        self.assertEqual(str(a), "0.1.0")
        self.assertEqual(str(b), "0.1.0-pr2")
        self.assertEqual(str(c), "0.1.0+012345")
        self.assertEqual(str(d), "0.1.0-pr2+012345")

    def test_operators(self):
        a = SemanticVersion("0.1.0")
        b = SemanticVersion("1.0.0")
        c = SemanticVersion("1.0.0")
        c2 = SemanticVersion("1.0.2")
        d = SemanticVersion("0.2.0")

        # check that the short form is the same as the long one
        aa = SemanticVersion("0.1")
        self.assertEqual(aa.version(), "0.1.0")

        # check second constructor
        c2b = SemanticVersion(1, 0, 2)
        self.assertTrue(c2 == c2b)

        self.assertFalse(a < a)
        self.assertFalse(b < a)
        self.assertTrue(a < d)
        self.assertFalse(d < a)

        self.assertTrue(a < b)
        self.assertTrue(a <= b)

        # equality
        self.assertTrue(a != b)
        self.assertTrue(b == c)
        self.assertFalse(a == b)
        self.assertFalse(a != a)

        # other operators
        self.assertTrue(b <= c)
        self.assertTrue(b >= c)
        self.assertTrue(c2 > c)
        self.assertFalse(c2 < c)
        self.assertTrue(b == b)

    def test_assign_copy(self):
        a = SemanticVersion("0.1+pr2")
        b = SemanticVersion("0.2")

        aa = SemanticVersion(a)
        self.assertTrue(a == aa)
        aaa = SemanticVersion(aa)
        self.assertTrue(a == aaa)
        # change a
        a = SemanticVersion(b)
        # aaa unchanged
        self.assertTrue(aa == aaa)
        # a and aaa now different
        self.assertTrue(a != aaa)

    def test_parse(self):
        a = SemanticVersion()
        self.assertFalse(a.parse(""))
        self.assertFalse(a.parse("0.1.2+1-1"))

    def test_constructor(self):
        a = SemanticVersion()

        self.assertEqual(a.major(), 0)
        self.assertEqual(a.minor(), 0)
        self.assertEqual(a.patch(), 0)
        self.assertFalse(a.prerelease())
        self.assertFalse(a.build())


if __name__ == '__main__':
    unittest.main()
