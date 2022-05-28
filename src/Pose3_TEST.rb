# Copyright (C) 2019 Open Source Robotics Foundation
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

class Pose3_TEST < Test::Unit::TestCase
  def test_construction
    p = Gz::Math::Pose3d.new

    assert(p.Pos().X() == 0.0,
           "Gz::Math::Pose3d default constructor should have PX==0")
    assert(p.Pos().Y() == 0.0,
           "Gz::Math::Pose3d default constructor should have PY==0")
    assert(p.Pos().Z() == 0.0,
           "Gz::Math::Pose3d default constructor should have PZ==0")

    assert(p.Rot().W() == 1.0,
           "Gz::Math::Pose3d default constructor should have QW==1")
    assert(p.Rot().X() == 0.0,
           "Gz::Math::Pose3d default constructor should have QX==0")
    assert(p.Rot().Y() == 0.0,
           "Gz::Math::Pose3d default constructor should have QY==0")
    assert(p.Rot().Z() == 0.0,
           "Gz::Math::Pose3d default constructor should have QZ==0")

    p1 = Gz::Math::Pose3d.new(
      Gz::Math::Vector3d.new(1, 2, 3),
      Gz::Math::Quaterniond.new(0, 0, Math::PI/2.0))

    assert(p1.Pos().X() == 1.0,
           "Gz::Math::Pose3d default constructor should have PX==1")
    assert(p1.Pos().Y() == 2.0,
           "Gz::Math::Pose3d default constructor should have PY==2")
    assert(p1.Pos().Z() == 3.0,
           "Gz::Math::Pose3d default constructor should have PZ==3")

    assert(p1.Rot().Euler().X() == 0.0,
           "Gz::Math::Pose3d should have EulerX==0")
    assert(p1.Rot().Euler().Y() == 0.0,
           "Gz::Math::Pose3d should have EulerY==0")
    assert((p1.Rot().Euler().Z() - Math::PI/2.0).abs() < 1e-3,
           "Gz::Math::Pose3d should have EulerZ==PI/2")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Pose3_TEST).passed? ? 0 : -1

