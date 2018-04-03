# Copyright (C) 2017 Open Source Robotics Foundation
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


class Ruby_TEST < Test::Unit::TestCase
  # This test just uses a few classes to make sure multiple ruby interfaces
  # work together.
  def test_construction
    angle1 = Ignition::Math::Angle.new

    assert(angle1.Radian() == 0.0,
           "Angle::Radian() should equal zero")

    v = Ignition::Math::Vector3d.new

    # ::Distance, ::Length()
    v.Set(1, 2, 3)
    assert(v.Length() == v.Distance(Ignition::Math::Vector3d.Zero),
           "Vector3d::Lenth() should equal Vector3d::Distance(zero)")

    v2 = Ignition::Math::Vector2d.new(1, 2)
    assert(v2.X == 1, "v2.X should equal 1")
    assert(v2.Y == 2, "v2.Y should equal 2")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Ruby_TEST).passed? ? 0 : -1
