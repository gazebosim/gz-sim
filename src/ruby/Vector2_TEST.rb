# Copyright (C) 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env ruby

require 'test/unit/ui/console/testrunner'
require 'test/unit'
require 'math'

class Vector2_TEST < Test::Unit::TestCase
  def test_construction
    v = Gz::Math::Vector2d.new
    assert(v.X.zero?, "v.X should equal zero")
    assert(v.Y.zero?, "v.Y should equal zero")

    v2 = Gz::Math::Vector2d.new(1, 2)
    assert(v2.X == 1, "v2.X should equal 1")
    assert(v2.Y == 2, "v2.Y should equal 2")
  end

  def test_functions
    v = Gz::Math::Vector2d.new(1, 2)

    # ::Distance
    assert((v.Distance(Gz::Math::Vector2d.Zero) - 2.236).abs < 1e-2,
           "Distance from (1,2)->(0,0) should equal 2.236")

    # ::Normalize
    v.Normalize()
    assert(v == Gz::Math::Vector2d.new(0.447214, 0.894427),
          "v should equal (0.447214, 0.894427)")

    # ::Set
    v.Set(4, 5)
    assert(v == Gz::Math::Vector2d.new(4, 5),
           "v should equal (4, 5)")

    # ::operator=
    v = Gz::Math::Vector2d.new(7, 8)
    assert(v == Gz::Math::Vector2d.new(7, 8),
           "v should equal (7, 8)")

    # ::operator+
    v = Gz::Math::Vector2d.new(1, 2) + 5
    assert(v == Gz::Math::Vector2d.new(6, 7),
           "v should equal (6, 7) after addition")

    # ::operator -
    v = v - Gz::Math::Vector2d.new(2, 4)
    assert(v == Gz::Math::Vector2d.new(4, 3),
           "v should equal (4, 3)")

    # ::operator /
    v.Set(10, 6)
    v = v / Gz::Math::Vector2d.new(2, 3)
    assert(v == Gz::Math::Vector2d.new(5, 2),
           "v should equal (5, 2)")

    # ::operator / int
    v.Set(10, 6)
    v = v / 2
    assert(v == Gz::Math::Vector2d.new(5, 3),
           "v should equal (5, 3)")

    # ::operator * int
    v.Set(10, 6)
    v = v * 2
    assert(v == Gz::Math::Vector2d.new(20, 12),
           "v should equal (20, 12)")

    # ::operator * vector2i
    v.Set(10, 6)
    v = v * Gz::Math::Vector2d.new(2, 4)
    assert(v == Gz::Math::Vector2d.new(20, 24),
           "v should equal (20, 24)")

    # ::IsFinite
    assert(v.IsFinite(), "v should be finite")
  end

  def test_equal_tolerance
    assert(!Gz::Math::Vector2d.Zero.Equal(
      Gz::Math::Vector2d.One, 1e-6),
      "Zero should not equal 1 with 1e-6 tolerance")

    assert(!Gz::Math::Vector2d.Zero.Equal(
      Gz::Math::Vector2d.One, 1e-3),
      "Zero should not equal 1 with 1e-3 tolerance")
    assert(!Gz::Math::Vector2d.Zero.Equal(
      Gz::Math::Vector2d.One, 1e-1),
      "Zero should not equal 1 with 1e-1 tolerance")
    assert(Gz::Math::Vector2d.Zero.Equal(
      Gz::Math::Vector2d.One, 1),
      "Zero should equal 1 with 1 tolerance")
    assert(Gz::Math::Vector2d.Zero.Equal(
      Gz::Math::Vector2d.One, 1.1),
      "Zero should equal 1 with 1.1 tolerance")
  end

  def test_max
     vec1 = Gz::Math::Vector2d.new(0.1, 0.2)
     vec2 = Gz::Math::Vector2d.new(0.3, 0.5)
     vec3 = Gz::Math::Vector2d.new(0.4, 0.2)

     assert((vec1.Max() - 0.2).abs() < 1e-10,
           "Vector2 vec1.Max should equal 0.2")

     vec1.Max(vec2)
     assert(vec1 == Gz::Math::Vector2d.new(0.3, 0.5),
            "Vector2 vec1 should equal [0.3, 0.5]")

     vec1.Max(vec3)
     assert(vec1 == Gz::Math::Vector2d.new(0.4, 0.5),
            "Vector2 vec1 should equal [0.4, 0.5]")
  end

  def test_min
     vec1 = Gz::Math::Vector2d.new(0.3, 0.5)
     vec2 = Gz::Math::Vector2d.new(0.1, 0.2)
     vec3 = Gz::Math::Vector2d.new(0.05, 0.1)

     assert((vec1.Min() - 0.3).abs() < 1e-10,
           "Vector2 vec1.Min should equal 0.3")

     vec1.Min(vec2)
     assert(vec1 == Gz::Math::Vector2d.new(0.1, 0.2),
            "Vector2 vec1 should equal [0.1, 0.2]")

     vec3.Max(vec2)
     assert(vec3 == Gz::Math::Vector2d.new(0.1, 0.2),
            "Vector2 vec3 should equal [0.1, 0.2]")
  end

  def test_dot
    v = Gz::Math::Vector2d.new(1, 2)

    assert(v.Dot(Gz::Math::Vector2d.new(3, 4)) == 11.0,
           "v.dot((3,4)) should equal 11")
    assert(v.Dot(Gz::Math::Vector2d.new(0, 0)) == 0.0,
           "v.dot((0,0)) should equal 0")
    assert(v.Dot(Gz::Math::Vector2d.new(1, 0)) == 1.0,
           "v.dot((1,0)) should equal 1")
    assert(v.Dot(Gz::Math::Vector2d.new(0, 1)) == 2.0,
           "v.dot((0,1)) should equal 2")
  end

  def test_add
    vec1 = Gz::Math::Vector2d.new(0.1, 0.2)
    vec2 = Gz::Math::Vector2d.new(1.1, 2.2)
    vec3 = vec1
    vec3 += vec2

    assert(vec1 + vec2 == Gz::Math::Vector2d.new(1.2, 2.4),
           "vec1 + vec2 should equal (1.2, 2.4")
    assert(vec3 == Gz::Math::Vector2d.new(1.2, 2.4),
           "vec3 should equal (1.2, 2.4)")

    # Add zeros
    begin
      # Scalar left and right
      assert(vec1 + 0 == vec1, "vec1 should equal vec1 + 0")

      # Vector left and right
      assert(Gz::Math::Vector2d.Zero + vec1 == vec1,
             "Gz::Math::Vector2d.Zero + vec1 should equal vec1")
      assert(vec1 + Gz::Math::Vector2d.Zero == vec1,
             "vec1 + Gz::Math::Vector2d.Zero should equal vec1")
    end

    # Add non-trivial scalar values left and right
    assert(vec1 + 2.5 == Gz::Math::Vector2d.new(2.6, 2.7),
           "vec1 + 2.5 should equal (2.6, 2.7)")
  end

  def test_sub
    vec1 = Gz::Math::Vector2d.new(0.1, 0.2)
    vec2 = Gz::Math::Vector2d.new(1.1, 2.2)
    vec3 = vec2
    vec3 -= vec1

    assert(vec2 - vec1 == Gz::Math::Vector2d.new(1.0, 2.0),
           "vec2 - vec1 should equal (1.0, 2.0)")
    assert(vec3 == Gz::Math::Vector2d.new(1.0, 2.0),
           "vec3 should equal (1.0, 2.0)")

    # Scalar left and right
    assert(vec1 - 0 == vec1, "vec1 - 0 should equal vec1")

    # Vector left and right
    assert(Gz::Math::Vector2d.Zero - vec1 == -vec1,
           "Gz::Math::Vector2d.Zero - vec1 should equal -vec1")
    assert(vec1 - Gz::Math::Vector2d.Zero == vec1,
           "vec1 - Gz::Math::Vector2d.Zero should equal vec1")

    # Subtract non-trivial scalar values left and right
    assert(vec1 - 2.5 == -Gz::Math::Vector2d.new(2.4, 2.3),
           "vec1 - 2.5 should equal (2.4, 2.3)")
  end

  def test_multiply
    v = Gz::Math::Vector2d.new(0.1, -4.2)

    # Scalar left and right
    assert(v * 0 == Gz::Math::Vector2d.Zero,
           "v * 0 should equal Zero")

    # Element-wise vector multiplication
    assert(v * Gz::Math::Vector2d.Zero == Gz::Math::Vector2d.Zero,
           "v * Gz::Math::Vector2d::Zero should equal zero")

    # Scalar left and right
    assert(v * 1 == v, "v * 1 should equal v")

    # Element-wise vector multiplication
    assert(v * Gz::Math::Vector2d.One == v,
           "v * Gz::Math::Vector2d.One should equal v")

    # Multiply by non-trivial scalar value
    scalar = 2.5
    expect = Gz::Math::Vector2d.new(0.25, -10.5)
    assert(v * scalar == expect,
           "v * 2.5 should equal (0.25, -10.5)")

    # Multiply by itself element-wise
    assert(v*v == Gz::Math::Vector2d.new(0.01, 17.64),
           "v*v should equal (0.01, 17.64)")
  end

  def test_length
    # Zero vector
    assert(Gz::Math::Vector2d.Zero.Length() == 0.0,
           "Length of zero should equal 0.0")
    assert(Gz::Math::Vector2d.Zero.SquaredLength() == 0.0,
           "Squared length of zero should equal 0.0")

    # One vector
    assert((Gz::Math::Vector2d.One.Length() - Math.sqrt(2)).abs < 1e-10,
           "Length of one should be near square root of 2")
    assert(Gz::Math::Vector2d.One.SquaredLength() == 2.0,
           "Squared lenght of one should equal 2")

    # Arbitrary vector
    v = Gz::Math::Vector2d.new(0.1, -4.2)
    assert((v.Length() - 4.20119030752).abs < 1e-10,
           "Length should be near 4.20119030752")
    assert((v.SquaredLength() - 17.65).abs < 1e-8,
           "Squared length of v should be near 17.65")
  end

  def test_nan
    nanVec = Gz::Math::Vector2d.NaN
    assert(!nanVec.IsFinite(),
           "NaN vector shouldn't be finite")
    assert(nanVec.X().nan?, "X should be NaN")
    assert(nanVec.Y().nan?, "Y should be NaN")

    nanVec.Correct()
    assert(Gz::Math::Vector2d.Zero == nanVec,
           "Corrected vector should equal zero")

    nanVecF = Gz::Math::Vector2f.NaN
    assert(!nanVecF.IsFinite(),
           "NaN vector shouldn't be finite")
    assert(nanVecF.X().nan?, "X should be NaN")
    assert(nanVecF.Y().nan?, "Y should be NaN")

    nanVecF.Correct()
    assert(Gz::Math::Vector2f.Zero == nanVecF,
           "Corrected vector should equal zero")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Vector2_TEST).passed? ? 0 : -1
