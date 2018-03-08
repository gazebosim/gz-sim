# Copyright (C) 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
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


class Vector3_TEST < Test::Unit::TestCase
  def test_construction
    v = Ignition::Math::Vector3d.new

    # ::Distance, ::Length()
    v.Set(1, 2, 3)
    assert(v.Length() == v.Distance(Ignition::Math::Vector3d.Zero),
           "Vector3d::Lenth() should equal Vector3d::Distance(zero)")

    # ::Rounded
    v.Set(1.23, 2.34, 3.55)
    assert(v.Rounded() == Ignition::Math::Vector3d.new(1, 2, 4),
           "Vector3d::Rounded() should equal [1, 2, 4]")

    # ::Round
    v.Round()
    assert(v.Round() == Ignition::Math::Vector3d.new(1, 2, 4),
           "Vector3d::Round should equal [1,2,4]")

    # ::DotProd
    assert(v.Dot(Ignition::Math::Vector3d.new(1, 2, 3)) == 17.0,
           "Vector3d::Dot([1,2,3]) should equal 17.0")

    # ::DistToLine
    v.Set(0, 0, 0)
    assert(1.0 == v.DistToLine(Ignition::Math::Vector3d.new(1, -1, 0),
                               Ignition::Math::Vector3d.new(1, 1, 0)),
           "Vector3d::DistToLine([1, -1, 0], [1, 1, 0]) should equal 1")

    # ::operator/ vector3
    v.Set(4, 4, 4)
    v = v / Ignition::Math::Vector3d.new(1, 2, 4)
    assert(v == Ignition::Math::Vector3d.new(4, 2, 1),
           "v / Vector3d(1, 2, 4) should equal Vector3d(4, 2, 1)")

    # ::operator / double
    v = v / 2
    assert(v == Ignition::Math::Vector3d.new(2, 1, 0.5),
           "v / 2 should equal Vector3d(2, 1, .5)")

    # ::operator * vector3
    v = v * Ignition::Math::Vector3d.new(2, 3, 4)
    assert(v == Ignition::Math::Vector3d.new(4, 3, 2),
           "v * Vector3d(2, 3, 4) should equal Vector3d(4, 3, 2)")

    v.Set(1.23, 2.35, 3.654321)
    v.Round(1)
    assert(v == Ignition::Math::Vector3d.new(1.2, 2.4, 3.7))

    # operator GetAbs
    v.Set(-1, -2, -3)
    assert(v.Abs() == Ignition::Math::Vector3d.new(1, 2, 3))

    # operator /=
    v.Set(1, 2, 4)
    v /= Ignition::Math::Vector3d.new(1, 4, 4)
    assert(v == Ignition::Math::Vector3d.new(1, 0.5, 1))

    # operator *=
    v.Set(1, 2, 4)
    v *= Ignition::Math::Vector3d.new(2, 0.5, 0.1)
    assert(v.Equal(Ignition::Math::Vector3d.new(2, 1, 0.4)))

    # Test the static defines.
    assert(Ignition::Math::Vector3d.Zero ==
           Ignition::Math::Vector3d.new(0, 0, 0),
           "Vector3d::Zero should equal [0, 0, 0]")

    assert(Ignition::Math::Vector3d.One ==
           Ignition::Math::Vector3d.new(1, 1, 1),
           "Vector3d::One should equal [1, 1, 1]")

    assert(Ignition::Math::Vector3d.UnitX ==
           Ignition::Math::Vector3d.new(1, 0, 0),
           "Vector3d::UnitX should equal [1, 0, 0]")

    assert(Ignition::Math::Vector3d.UnitY ==
           Ignition::Math::Vector3d.new(0, 1, 0),
           "Vector3d::UnitY should equal [0, 1, 0]")

    assert(Ignition::Math::Vector3d.UnitZ ==
           Ignition::Math::Vector3d.new(0, 0, 1),
           "Vector3d::UnitZ should equal [0, 0, 1]")
  end

  def test_distance
    vec1 = Ignition::Math::Vector3d.new(0, 0, 0)
    vec2 = Ignition::Math::Vector3d.new(1, 2, 3)

    dist = vec1.Distance(vec2)
    assert((dist - 3.74165738677).abs() < 1e-6,
           "Vector3 distance should be near 3.74165738677")

    dist2 = vec1.Distance(1, 2, 3)
    assert(dist == dist2,
           "Vector3 dist should equal dist2")
  end

  def test_sum
    vec1 = Ignition::Math::Vector3d.new(0, 0, 0)
    vec2 = Ignition::Math::Vector3d.new(1, 2, 3)

    sum1 = vec1.Sum()
    sum2 = vec2.Sum()

    assert(sum1 == 0, "Vector3 sum1 should equal 0")
    assert(sum2 == 6, "Vector3 sum2 should equal 6")
  end

  def test_squared_length
    vec1 = Ignition::Math::Vector3d.new(0, 0, 0)
    vec2 = Ignition::Math::Vector3d.new(1, 2, 3)

    sum1 = vec1.SquaredLength()
    sum2 = vec2.SquaredLength()

    assert(sum1 == 0, "Vector3 sum1 should equal 0")
    assert(sum2 == 14, "Vector3 sum2 should equal 0")
  end

  def test_length
    # Zero vector
    assert(Ignition::Math::Vector3d.Zero.Length() == 0.0,
           "Vector3 length of [0, 0, 0] should equal 0")
    assert(Ignition::Math::Vector3d.Zero.SquaredLength() == 0.0,
           "Vector3 squared length of [0, 0, 0] should equal 0")

    # UnitXYZ vectorsIgnition::
    assert(Ignition::Math::Vector3d.UnitX.Length() == 1.0,
           "Vector3 length of unitx should equal 1")
    assert(Ignition::Math::Vector3d.UnitY.Length() == 1.0,
           "Vector3 length of unity should equal 1")
    assert(Ignition::Math::Vector3d.UnitZ.Length() == 1.0,
           "Vector3 length of unitz should equal 1")
    assert(Ignition::Math::Vector3d.UnitX.SquaredLength() == 1.0,
           "Vector3 squared length of unitx should equal 1")
    assert(Ignition::Math::Vector3d.UnitY.SquaredLength() == 1.0,
           "Vector3 squared length of unity should equal 1")
    assert(Ignition::Math::Vector3d.UnitZ.SquaredLength() == 1.0,
           "Vector3 squared length of unitz should equal 1")

    # One vector
    assert((Ignition::Math::Vector3d.One.Length() -
            Math.sqrt(3.0)).abs() < 1e-10,
           "Vector3 length of [1, 1, 1] should equal sqrt(3.0)")

    assert(Ignition::Math::Vector3d.One.SquaredLength() == 3.0,
           "Vector3 squared lenght of [1, 1, 1] should equal 3.0")

    # Arbitrary vector
    v = Ignition::Math::Vector3d.new(0.1, -4.2, 2.5)
    assert((v.Length() - 4.88876262463).abs() < 1e-10,
           "Vector3 v length should equal 4.88876262463")

    assert((v.SquaredLength() - 23.9).abs() < 1e-10 ,
           "Vector3 v squared length should equal 23.9")
  end

  def test_normalize
    vec1 = Ignition::Math::Vector3d.new(0, 0, 0)
    vec2 = Ignition::Math::Vector3d.new(1, 2, 3)

    vec3 = vec1.Normalize()
    assert(vec3 == vec1, "Vector3 vec3 should equal vec1")
    assert(vec1 == Ignition::Math::Vector3d.Zero,
           "Vector3 should equal [0, 0, 0]")

    vec3 = vec2.Normalize()
    assert(vec3 == vec2, "Vector3 vec3 should equal vec2")
    assert(vec2 == Ignition::Math::Vector3d.new(0.267261, 0.534522, 0.801784),
           "Vector3 vec2 should equal [0.267261, 0.534522, 0.801784]")

    vecConst = Ignition::Math::Vector3d.new(1, 2, 3)
    assert(vecConst.Normalized() == vec3,
           "Vector3 vecConst should equal vec3")
    assert(vecConst == Ignition::Math::Vector3d.new(1, 2, 3),
           "Vector3 vecConst should euqal [1, 2, 3]")
  end

  def test_ge_normal
    vec1 = Ignition::Math::Vector3d.new(0, 0, 0)
    vec2 = Ignition::Math::Vector3d.new(0, 1, 0)
    vec3 = Ignition::Math::Vector3d.new(1, 1, 0)

    norm = Ignition::Math::Vector3d::Normal(vec1, vec2, vec3)
    assert(norm == Ignition::Math::Vector3d.new(0, 0, -1),
           "Vector3 norm should equal [0, 0, -1]")
  end

  def test_perpendicular
    vec1 = Ignition::Math::Vector3d.new(1, 1, 0)
    vec2 = Ignition::Math::Vector3d.new(0, 1, 1)
    vec3 = Ignition::Math::Vector3d.new(1e-7, 1e-7, 1e-7)
    vec4 = Ignition::Math::Vector3d.new(1, 0, 0)

    assert(vec1.Perpendicular() == Ignition::Math::Vector3d.new(0, 0, -1),
           "Vector3 vec1.Perpendicular() should equal [0, 0, -1]")
    assert(vec2.Perpendicular() == Ignition::Math::Vector3d.new(0, 1, -1),
           "Vector3 vec2.Perpendicular() should equal [0, 1, -1]")
    assert(vec3.Perpendicular() == Ignition::Math::Vector3d.new(0, 0, 0),
           "Vector3 vec3.Perpendicular() should equal [0, 0, 0]")
    assert(vec4.Perpendicular() == Ignition::Math::Vector3d.new(0, 0, 1),
           "Vector3 vec4.Perpendicular() should equal [0, 0, 1]")
  end

  def test_max
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)
    vec2 = Ignition::Math::Vector3d.new(0.2, 0.3, 0.4)
    vec3 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)

    assert((vec1.Max() - 0.3).abs() < 1e-10,
           "Vector3 vec1.Max should equal 0.3")

    vec1.Max(vec2)
    assert(vec1 == Ignition::Math::Vector3d.new(0.2, 0.3, 0.4),
           "Vector3 vec1 should equal [0.2, 0.3, 0.4]")

    vec1.Max(vec3)
    assert(vec1 == Ignition::Math::Vector3d.new(0.2, 0.3, 0.4),
           "Vector3 vec1 should equal [0.2, 0.3, 0.4]")
  end

  def test_min
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)
    vec2 = Ignition::Math::Vector3d.new(0.2, 0.3, 0.4)
    vec3 = Ignition::Math::Vector3d.new(0.05, 0.1, 0.2)

    assert((vec1.Min() - 0.1).abs < 1e-10,
           "Vector3 vec1.Min should equal 0.1")

    vec1.Min(vec2)
    assert(vec1 == Ignition::Math::Vector3d.new(0.1, 0.2, 0.3),
           "Vector3 vec1 should equal [0.1, 0.2, 0.3]")

    vec1.Min(vec3)
    assert(vec1 == Ignition::Math::Vector3d.new(0.05, 0.1, 0.2),
           "Vector3 vec1 should equal [0.05, 0.1, 0.2]")
  end

  def test_add
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.4)
    vec2 = Ignition::Math::Vector3d.new(1.1, 2.2, 3.4)

    vec3 = vec1
    vec3 += vec2

    assert(vec1 + vec2 == Ignition::Math::Vector3d.new(1.2, 2.4, 3.8),
           "Vector3 vec1 + vec2 should equal [1.2, 2.4, 3.8]")
    assert(vec3 == Ignition::Math::Vector3d.new(1.2, 2.4, 3.8),
           "Vector3 vec3 should equal [1.2, 2.4, 3.8]")

    # Add zeros
    # Scalar left and right
    assert(vec1 + 0 == vec1, "Vector3 vec1+0 should equal vec1")

    # Vector left and right
    assert(Ignition::Math::Vector3d.Zero + vec1 == vec1,
           "Vector3 Zero + vec1 should equal vec1")
    assert(vec1 + Ignition::Math::Vector3d.Zero == vec1,
           "Vector3 vec1 + Zero should equal vec1")

    # Addition assignment
    vec4 = vec1
    vec4 += 0
    assert(vec4 == vec1, "Vector3 vec4 should equal vec1")
    vec4 += Ignition::Math::Vector3d.Zero
    assert(vec4 == vec1, "Vector3 vec4 should equal vec1")

    # Add non-trivial scalar values left and right
    assert(vec1 + 2.5 == Ignition::Math::Vector3d.new(2.6, 2.7, 2.9),
           "Vector3 vec1 + 2.5 should equal [2.6, 2.7, 2.9]")

    vec1 = vec4
    vec4 += 2.5
    assert(vec4 == Ignition::Math::Vector3d.new(2.6, 2.7, 2.9),
           "Vector3 vec4 should equal [2.6, 2.7, 2.9]")
  end

  def test_sub
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.4)
    vec2 = Ignition::Math::Vector3d.new(1.1, 2.2, 3.4)

    vec3 = vec2
    vec3 -= vec1

    assert(vec2 - vec1 === Ignition::Math::Vector3d.new(1.0, 2.0, 3.0),
           "Vector3 vec2 - vec1 should equal [1.0, 2.0 3.0]")
    assert(vec3 == Ignition::Math::Vector3d.new(1.0, 2.0, 3.0),
           "Vector3 vec3 should equal [1.0 2.0 3.0]")

    #Subtraction with zeros

    # Scalar left and right
    assert(vec1 - 0 == vec1, "Vector3 vec1 - 0 should equal vec1")

    # Vector left and right
    assert(Ignition::Math::Vector3d.Zero - vec1 == -vec1,
           "Vector3 Zero - vec1 should equal -vec1")
    assert(vec1 - Ignition::Math::Vector3d.Zero == vec1,
           "Vector3 vec1 - Zero should equal vec1")

    # Subtraction assignment
    vec4 = vec1
    vec4 -= 0
    assert(vec4 == vec1, "Vector3 vec4 should equal vec1")
    vec4 -= Ignition::Math::Vector3d.Zero
    assert(vec4 == vec1, "Vector3 vec4 should equal vec1")

    # Subtract non-trivial scalar values left and right
    assert(vec1 - 2.5 == -Ignition::Math::Vector3d.new(2.4, 2.3, 2.1),
           "Vecetor3 vec1 - 2.5 should equal [2.4, 2.3, 2.1]")

    vec4 = vec1
    vec4 -= 2.5
    assert(vec4 == -Ignition::Math::Vector3d.new(2.4, 2.3, 2.1),
           "Vector3 vec4 should equal [2.4, 2.3, 2.1]")
  end

  def test_divide
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.4)

    vec3 = vec1 / 2.0
    assert(vec3 == Ignition::Math::Vector3d.new(0.05, 0.1, 0.2),
           "Vector3 vec3 should equal [0.05, 0.1, 0.2]")

    vec3 /= 4.0
    assert(vec3 == Ignition::Math::Vector3d.new(0.0125, 0.025, 0.05),
           "Vector3 vec3 should qual [0.0125, 0.025, 0.05]")
  end

  def test_multiply
    v = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)

    vec3 = v * 2.0
    assert(vec3 == Ignition::Math::Vector3d.new(0.2, 0.4, 0.6),
           "Vector3 vec3 should equal[0.2, 0.4, 0.6]")

    vec3 *= 4.0
    assert(vec3 == Ignition::Math::Vector3d.new(0.8, 1.6, 2.4),
           "Vector3 vec3 should equal [0.8, 1.6, 2.4]")

    # Multiply by zero

    # Scalar left and right
    assert(v * 0 == Ignition::Math::Vector3d.Zero,
           "Vector3 v * 0 should equal Zero")

    # Element-wise vector multiplication
    assert(v * Ignition::Math::Vector3d.Zero == Ignition::Math::Vector3d.Zero,
           "Vector3 v * Zero should equal Zero")

    # Multiply by one

    # Scalar left and right
    assert(v * 1 == v, "Vector3 v * 1 should equal v")

    # Element-wise vector multiplication
    assert(v * Ignition::Math::Vector3d.One == v,
           "Vector3 v * One should equal v")

    # Multiply by non-trivial scalar value

    scalar = 2.5
    expect = Ignition::Math::Vector3d.new(0.25, 0.5, 0.75)
    assert(v * scalar == expect,
           "Vector3 v * scalar should equal [0.25, 0.5, 0.75]")

    # Multiply by itself element-wise
    assert(v*v == Ignition::Math::Vector3d.new(0.01, 0.04, 0.09),
           "Vector3 v * v should euqal [0.01, 0.04, 0.09]")
  end

  def test_not_equal
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)
    vec2 = Ignition::Math::Vector3d.new(0.2, 0.2, 0.3)
    vec3 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)

    assert(vec1 != vec2, "Vector3 vec1 should not equal vec2")
    assert(!(vec1 != vec3), "Vector3 vec1 should not equal vec3" )
  end

  def test_equal
    assert(!Ignition::Math::Vector3d.Zero.Equal(
      Ignition::Math::Vector3d.One, 1e-6),
      "Vector3 Zero should not equal 1 with tolerance of 1e-6")
    assert(!Ignition::Math::Vector3d.Zero.Equal(
      Ignition::Math::Vector3d.One, 1e-3),
      "Vector3 Zero should not equal 1 with tolerance of 1e-3")
    assert(!Ignition::Math::Vector3d.Zero.Equal(
      Ignition::Math::Vector3d.One, 1e-1),
      "Vector3 Zero should not equal 1 with tolerance of 1e-1")

    assert(Ignition::Math::Vector3d.Zero.Equal(
           Ignition::Math::Vector3d.One, 1),
           "Vector3 Zero should equal 1 with tolerance of 1")
    assert(Ignition::Math::Vector3d.Zero.Equal(
           Ignition::Math::Vector3d.One, 1.1),
           "Vector3 Zero should equal 1 with tolerance of 1.1")
  end

  def test_finite
    vec1 = Ignition::Math::Vector3d.new(0.1, 0.2, 0.3)

    assert(vec1.IsFinite(), "Vector3 vec1 should be be finite")
  end

end

exit Test::Unit::UI::Console::TestRunner.run(Vector3_TEST).passed? ? 0 : -1
