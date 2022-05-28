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


class Vector4_TEST < Test::Unit::TestCase
  def test_construction
    v = Gz::Math::Vector4d.new

    # ::operator== vector4
    assert(true,
           "Vector4d::Zero should equal [0, 0, 0, 0]")

    # ::Distance, ::Length()
    v.Set(1, 2, 3, 4)
    assert(v.Length() == v.Distance(Gz::Math::Vector4d.Zero),
           "Vector4d::Lenth() should equal Vector4d::Distance(zero)")

    # ::operator/ vector4
    v.Set(4, 4, 4, 4)
    v = v / Gz::Math::Vector4d.new(1, 2, 2, 4)
    assert(v == Gz::Math::Vector4d.new(4, 2, 2, 1),
           "v / Vector4d(1, 2, 2, 4) should equal Vector4d(4, 2, 2, 1)")

    # ::operator / double
    v = v / 2
    assert(v == Gz::Math::Vector4d.new(2, 1, 1, 0.5),
           "v / 2 should equal Vector4d(2, 1, 1, .5)")

    # ::operator * vector4
    v = v * Gz::Math::Vector4d.new(2, 3, 3, 4)
    assert(v == Gz::Math::Vector4d.new(4, 3, 3, 2),
           "v * Vector4d(2, 3, 3, 4) should equal Vector4d(4, 3, 3, 2)")

    # operator /=
    v.Set(1, 2, 2, 4)
    v /= Gz::Math::Vector4d.new(1, 4, 8, 4)
    assert(v == Gz::Math::Vector4d.new(1, 0.5, 0.25, 1))

    # operator *=
    v.Set(1, 2, 2, 4)
    v *= Gz::Math::Vector4d.new(2, 0.5, 0.25, 0.1)
    assert(v == Gz::Math::Vector4d.new(2, 1, 0.5, 0.4))

    # Test the static defines.
    assert(Gz::Math::Vector4d.Zero ==
           Gz::Math::Vector4d.new(0, 0, 0, 0),
           "Vector4d::Zero should equal [0, 0, 0, 0]")

    assert(Gz::Math::Vector4d.One ==
           Gz::Math::Vector4d.new(1, 1, 1, 1),
           "Vector4d::One should equal [1, 1, 1, 1]")
  end

  def test_distance
    vec1 = Gz::Math::Vector4d.new(0, 0, 0, 0)
    vec2 = Gz::Math::Vector4d.new(1, 2, 3, 4)

    dist = vec1.Distance(vec2)
    assert((dist - 5.47722557505).abs() < 1e-6,
           "Vector4 distance should be near 5,47722557505")
  end

  def test_squared_length
    vec1 = Gz::Math::Vector4d.new(0, 0, 0, 0)
    vec2 = Gz::Math::Vector4d.new(1, 2, 3, 4)

    sum1 = vec1.SquaredLength()
    sum2 = vec2.SquaredLength()

    assert(sum1 == 0, "Vector4 sum1 should equal 0")
    assert(sum2 == 30, "Vector4 sum2 should equal 30")
  end

  def test_length
    # Zero vector
    assert(Gz::Math::Vector4d.Zero.Length() == 0.0,
           "Vector4 length of [0, 0, 0, 0] should equal 0")
    assert(Gz::Math::Vector4d.Zero.SquaredLength() == 0.0,
           "Vector4 squared length of [0, 0, 0, 0] should equal 0")

    # One vector
    assert((Gz::Math::Vector4d.One.Length() -
            Math.sqrt(4.0)).abs() < 1e-10,
           "Vector4 length of [1, 1, 1, 1] should equal sqrt(4.0)")

    assert(Gz::Math::Vector4d.One.SquaredLength() == 4.0,
           "Vector4 squared lenght of [1, 1, 1, 1] should equal 4.0")

    # Arbitrary vector
    v = Gz::Math::Vector4d.new(0.1, -4.2, 2.5, -1.2)
    assert((v.Length() - 5.03388517946).abs() < 1e-10,
           "Vector4 v length should equal 5.03388517946")

    assert((v.SquaredLength() - 25.34).abs() < 1e-10 ,
           "Vector4 v squared length should equal 25.34")
  end

  def test_normalize
    vec1 = Gz::Math::Vector4d.new(0, 0, 0, 0)
    vec2 = Gz::Math::Vector4d.new(1, 2, 3, 4)

    vec3 = vec1
    vec3.Normalize()
    assert(vec3 == vec1, "Vector4 vec3 should equal vec1")
    assert(vec1 == Gz::Math::Vector4d.Zero,
           "Vector4 should equal [0, 0, 0, 0]")

    vec3 = vec2
    vec2.Normalize()
    assert(vec2.Equal(Gz::Math::Vector4d.new(0.182575, 0.365150, 0.547725, 0.730300), 1e-5),
           "Vector4 vec3 should equal [0.182575, 0.365150, 0.547725, 0.730300]")
  end

  def test_add
    vec1 = Gz::Math::Vector4d.new(0.1, 0.2, 0.4, 0.8)
    vec2 = Gz::Math::Vector4d.new(1.1, 2.2, 3.4, 4.3)

    vec3 = vec1
    vec3 += vec2

    assert(vec1 + vec2 == Gz::Math::Vector4d.new(1.2, 2.4, 3.8, 5.1),
           "Vector4 vec1 + vec2 should equal [1.2, 2.4, 3.8, 4.9]")
    assert(vec3 == Gz::Math::Vector4d.new(1.2, 2.4, 3.8, 5.1),
           "Vector4 vec3 should equal [1.2, 2.4, 3.8, 4.9]")

    # Addition with zeros

    # Scalar left and right
    assert(vec1 + 0 == vec1, "Vector4 vec1+0 should equal vec1")

    # Vector left and right
    assert(Gz::Math::Vector4d.Zero + vec1 == vec1,
           "Vector4 Zero + vec1 should equal vec1")
    assert(vec1 + Gz::Math::Vector4d.Zero == vec1,
           "Vector4 vec1 + Zero should equal vec1")

    # Addition assignment
    vec4 = vec1
    vec4 += 0
    assert(vec4 == vec1, "Vector4 vec4 should equal vec1")
    vec4 += Gz::Math::Vector4d.Zero
    assert(vec4 == vec1, "Vector4 vec4 should equal vec1")

    # Add non-trivial scalar values left and right
    assert(vec1 + 2.5 == Gz::Math::Vector4d.new(2.6, 2.7, 2.9, 3.3),
           "Vector4 vec1 + 2.5 should equal [2.6, 2.7, 2.9, 3.3]")

    vec1 = vec4
    vec4 += 2.5
    assert(vec4 == Gz::Math::Vector4d.new(2.6, 2.7, 2.9, 3.3),
           "Vector4 vec4 should equal [2.6, 2.7, 2.9, 3.3]")
  end

  def test_sub
    vec1 = Gz::Math::Vector4d.new(0.1, 0.2, 0.4, 0.8)
    vec2 = Gz::Math::Vector4d.new(1.1, 2.2, 3.4, 4.3)

    vec3 = vec2
    vec3 -= vec1

    assert(vec2 - vec1 === Gz::Math::Vector4d.new(1.0, 2.0, 3.0, 3.5),
           "Vector4 vec2 - vec1 should equal [1.0, 2.0, 3.0, 3.5]")
    assert(vec3 == Gz::Math::Vector4d.new(1.0, 2.0, 3.0, 3.5),
           "Vector4 vec3 should equal [1.0, 2.0, 3.0, 3.5]")

    # Subtraction with zeros

    # Scalar left and right
    assert(vec1 - 0 == vec1, "Vector4 vec1 - 0 should equal vec1")

    # Vector left and right
    assert(Gz::Math::Vector4d.Zero - vec1 == -vec1,
           "Vector4 Zero - vec1 should equal -vec1")
    assert(vec1 - Gz::Math::Vector4d.Zero == vec1,
           "Vector4 vec1 - Zero should equal vec1")

    # Subtraction assignment
    vec4 = vec1
    vec4 -= 0
    assert(vec4 == vec1, "Vector4 vec4 should equal vec1")
    vec4 -= Gz::Math::Vector4d.Zero
    assert(vec4 == vec1, "Vector4 vec4 should equal vec1")

    # Subtract non-trivial scalar values left and right
    assert(vec1 - 2.5 == -Gz::Math::Vector4d.new(2.4, 2.3, 2.1, 1.7),
           "Vecetor3 vec1 - 2.5 should equal [2.4, 2.3, 2.1, 1.7]")

    vec4 = vec1
    vec4 -= 2.5
    assert(vec4 == -Gz::Math::Vector4d.new(2.4, 2.3, 2.1, 1.7),
           "Vector4 vec4 - 2.5 should equal [2.4, 2.3, 2.1, 1.7]")
  end

  def test_divide
    vec1 = Gz::Math::Vector4d.new(0.1, 0.2, 0.4, 0.8)

    vec3 = vec1 / 2.0
    assert(vec3 == Gz::Math::Vector4d.new(0.05, 0.1, 0.2, 0.4),
           "Vector4 vec3 should equal [0.05, 0.1, 0.2, 0.4]")

    vec3 /= 4.0
    assert(vec3 == Gz::Math::Vector4d.new(0.0125, 0.025, 0.05, 0.1),
           "Vector4 vec3 should qual [0.0125, 0.025, 0.05, 0.1]")
  end

  def test_multiply
    v = Gz::Math::Vector4d.new(0.1, 0.2, 0.3, 0.4)

    vec3 = v * 2.0
    assert(vec3 == Gz::Math::Vector4d.new(0.2, 0.4, 0.6, 0.8),
           "Vector4 vec3 should equal[0.2, 0.4, 0.6, 0.8]")

    vec3 *= 4.0
    assert(vec3 == Gz::Math::Vector4d.new(0.8, 1.6, 2.4, 3.2),
           "Vector4 vec3 should equal [0.8, 1.6, 2.4, 3.2]")

    # Multiply by zero

    # Scalar left and right
    assert(v * 0 == Gz::Math::Vector4d.Zero,
           "Vector4 v * 0 should equal Zero")

    # Element-wise vector multiplication
    assert(v * Gz::Math::Vector4d.Zero == Gz::Math::Vector4d.Zero,
           "Vector4 v * Zero should equal Zero")

    # Multiply by one

    # Scalar left and right
    assert(v * 1 == v, "Vector4 v * 1 should equal v")

    # Element-wise vector multiplication
    assert(v * Gz::Math::Vector4d.One == v,
           "Vector4 v * One should equal v")

    # Multiply by non-trivial scalar value

    scalar = 2.5
    expect = Gz::Math::Vector4d.new(0.25, 0.5, 0.75, 1.0)
    assert(v * scalar == expect,
           "Vector4 v * scalar should equal [0.25, 0.5, 0.75, 1.0]")

    # Multiply by itself element-wise
    assert(v*v == Gz::Math::Vector4d.new(0.01, 0.04, 0.09, 0.16),
           "Vector4 v * v should euqal [0.01, 0.04, 0.09, 0.16]")
  end

  def test_not_equal
    vec1 = Gz::Math::Vector4d.new(0.1, 0.2, 0.3, 0.4)
    vec2 = Gz::Math::Vector4d.new(0.2, 0.2, 0.3, 0.4)
    vec3 = Gz::Math::Vector4d.new(0.1, 0.2, 0.3, 0.4)

    assert(vec1 != vec2, "Vector4 vec1 should not equal vec2")
    assert(!(vec1 != vec3), "Vector4 vec1 should equal vec3" )
  end

  def test_equal
    assert(!Gz::Math::Vector4d.Zero.Equal(
      Gz::Math::Vector4d.One, 1e-6),
      "Vector4 Zero should not equal 1 with tolerance of 1e-6")
    assert(!Gz::Math::Vector4d.Zero.Equal(
      Gz::Math::Vector4d.One, 1e-3),
      "Vector4 Zero should not equal 1 with tolerance of 1e-3")
    assert(!Gz::Math::Vector4d.Zero.Equal(
      Gz::Math::Vector4d.One, 1e-1),
      "Vector4 Zero should not equal 1 with tolerance of 1e-1")

    assert(Gz::Math::Vector4d.Zero.Equal(
           Gz::Math::Vector4d.One, 1),
           "Vector4 Zero should equal 1 with tolerance of 1")
    assert(Gz::Math::Vector4d.Zero.Equal(
           Gz::Math::Vector4d.One, 1.1),
           "Vector4 Zero should equal 1 with tolerance of 1.1")
  end

  def test_finite
    vec1 = Gz::Math::Vector4d.new(0.1, 0.2, 0.3, 0.4)

    assert(vec1.IsFinite(), "Vector4 vec1 should be be finite")
  end

  def test_nan
    nanVec = Gz::Math::Vector4d.NaN
    assert(!nanVec.IsFinite(),
           "NaN vector shouldn't be finite")
    assert(nanVec.X().nan?, "X should be NaN")
    assert(nanVec.Y().nan?, "Y should be NaN")
    assert(nanVec.Z().nan?, "Z should be NaN")
    assert(nanVec.W().nan?, "W should be NaN")

    nanVec.Correct()
    assert(Gz::Math::Vector4d.Zero == nanVec,
           "Corrected vector should equal zero")

    nanVecF = Gz::Math::Vector4f.NaN
    assert(!nanVecF.IsFinite(),
           "NaN vector shouldn't be finite")
    assert(nanVecF.X().nan?, "X should be NaN")
    assert(nanVecF.Y().nan?, "Y should be NaN")
    assert(nanVecF.Z().nan?, "Z should be NaN")
    assert(nanVecF.W().nan?, "W should be NaN")

    nanVecF.Correct()
    assert(Gz::Math::Vector4f.Zero == nanVecF,
           "Corrected vector should equal zero")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Vector4_TEST).passed? ? 0 : -1
