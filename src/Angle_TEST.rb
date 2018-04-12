# Copyright (C) 2016 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
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

class Angle_TEST < Test::Unit::TestCase
  def test_construction
    angle1 = Ignition::Math::Angle.new

    assert(angle1.Radian() == 0.0,
           "Angle::Radian() should equal zero")

    angle1.Degree(180.0)
    assert(angle1 == Ignition::Math::Angle.Pi,
           "180.0 degrees should equal PI")

    assert(angle1 != Ignition::Math::Angle.Pi + Ignition::Math::Angle.new(0.1),
           "180.0 degrees should not equal PI + 0.1")

    assert(angle1 == Ignition::Math::Angle.Pi +
           Ignition::Math::Angle.new(0.0001),
           "180.0 degrees should equal PI + 0.0001")

    assert(angle1 == Ignition::Math::Angle.Pi -
           Ignition::Math::Angle.new(0.0001),
           "180.0 degrees should equal PI - 0.0001")

    assert(Ignition::Math::Angle.new(0) == Ignition::Math::Angle.new(0),
           "Zero angle should equal zero angle")

    assert(Ignition::Math::Angle.new(0) == Ignition::Math::Angle.new(0.001),
           "Zero should equal 0.001")

    angle1 = Ignition::Math::Angle.new(0.1) - Ignition::Math::Angle.new(0.3)
    assert(angle1 == Ignition::Math::Angle.new(-0.2),
           "Angle1 should equal -0.2")

    angle = Ignition::Math::Angle.new(0.5)
    assert(0.5 == angle.Radian, "Angle should equal 0.5")

    angle.Radian(Math::PI)
    assert(Ignition::Math::Angle.Pi.Degree == angle.Degree,
           "Pi in degrees should equal Angle.Radian(Pi).Degree")

    angle.Normalize()
    assert(Ignition::Math::Angle.new(Math::PI).Degree == angle.Degree,
           "Normalized angle should equal PI")

    angle = Ignition::Math::Angle.new(0.1) + Ignition::Math::Angle.new(0.2)
    assert((angle.Radian - 0.3).abs < 1e-6, "Angle should equal 0.3")

    angle = Ignition::Math::Angle.new(0.1) * Ignition::Math::Angle.new(0.2)
    assert((angle.Radian - 0.02).abs < 1e-6, "Angle should equal 0.02")

    angle = Ignition::Math::Angle.new(0.1) / Ignition::Math::Angle.new(0.2)
    assert((angle.Radian - 0.5).abs < 1e-6, "Angle should equal 0.5")

    angle -= Ignition::Math::Angle.new(0.1)
    assert((angle.Radian - 0.4).abs < 1e-6, "Angle should equal 0.1")

    angle += Ignition::Math::Angle.new(0.2)
    assert((angle.Radian - 0.6).abs < 1e-6, "Angle should equal 0.6")

    angle *= Ignition::Math::Angle.new(0.5)
    assert((angle.Radian - 0.3).abs < 1e-6, "Angle should equal 0.5")

    angle /= Ignition::Math::Angle.new(0.1)
    assert((angle.Radian() - 3.0).abs < 1e-6, "Angle should equal 3.0")
    assert(angle == Ignition::Math::Angle.new(3), "Angle should equal Angle(3)")
    assert(angle != Ignition::Math::Angle.new(2),
           "Angle should not equal Angle(2)")
    assert(angle < Ignition::Math::Angle.new(4), "Angle should be < Angle(4)")
    assert(angle > Ignition::Math::Angle.new(2), "Angle should be > Angle(2)")
    assert(angle >= Ignition::Math::Angle.new(3), "Angle should be >= Angle(3)")
    assert(angle <= Ignition::Math::Angle.new(3), "Angle should be <= 3")

    angle = 1.2
    assert(angle <= 1.21, "Angle should be less than or equal to 1.21")
    assert(angle >= 1.19, "Angle should be greater than or equal to 1.19")
    assert(angle <= 1.2, "Angle should be less than or equal to 1.2")
    assert(!(angle <= -1.19), "Angle should not be less than or equal to -1.19")

    assert(Ignition::Math::Angle.new(1.2) <=
           Ignition::Math::Angle.new(1.2000000001),
           "1.2 should be less than or equal to 1.2000000001")
    assert(Ignition::Math::Angle.new(1.2000000001) <=
           Ignition::Math::Angle.new(1.2),
           "1.2000000001 should be less than or equal to 1.2")

    angle = 1.2
    assert(!(angle >= 1.21), "Angle should not be greater or equal to 1.21")
    assert(angle >= 1.19, "Angle should be greater or equal to 1.19")
    assert(angle >= 1.2, "Angle should be greater or equal to 1.2")
    assert(angle >= -1.19, "Angle should be greater or equal to -1.19")

    assert(Ignition::Math::Angle.new(1.2) >=
           Ignition::Math::Angle.new(1.2000000001),
           "1.2 should be greater than or equal to 1.2000000001")
    assert(Ignition::Math::Angle.new(1.2000000001) >=
           Ignition::Math::Angle.new(1.2),
           "1.2000000001 should be greater than or equal to 1.2")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Angle_TEST).passed? ? 0 : -1
