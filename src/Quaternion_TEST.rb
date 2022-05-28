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

class Quaternion_TEST < Test::Unit::TestCase
  def test_construction
    q = Gz::Math::Quaterniond.new
    assert(q.W() == 1.0,
           "Gz::Math::Quaterniond default constructor should have W==1")
    assert(q.X() == 0.0,
           "Gz::Math::Quaterniond default constructor should have X==0")
    assert(q.Y() == 0.0,
           "Gz::Math::Quaterniond default constructor should have Y==0")
    assert(q.Z() == 0.0,
           "Gz::Math::Quaterniond default constructor should have Z==0")

    q1 = Gz::Math::Quaterniond.new(1, 2, 3, 4)
    assert(q1.W() == 1.0,
           "Gz::Math::Quaterniond(1, 2, 3, 4) should have W==1")
    assert(q1.X() == 2.0,
           "Gz::Math::Quaterniond(1, 2, 3, 4) should have X==2")
    assert(q1.Y() == 3.0,
           "Gz::Math::Quaterniond(1, 2, 3, 4) should have Y==3")
    assert(q1.Z() == 4.0,
           "Gz::Math::Quaterniond default constructor should have Z==4")

    q2 = Gz::Math::Quaterniond.new(0, 0, 0, 0);
    assert(q2.W() == 0.0,
           "Gz::Math::Quaterniond(0, 0, 0, 0) should have W==0")
    assert(q2.X() == 0.0,
           "Gz::Math::Quaterniond(0, 0, 0, 0) should have W==0")
    assert(q2.Y() == 0.0,
           "Gz::Math::Quaterniond(0, 0, 0, 0) should have W==0")
    assert(q2.Z() == 0.0,
           "Gz::Math::Quaterniond(0, 0, 0, 0) should have W==0")

    # Test inverse
    qI = q2.Inverse()
    assert(qI.W() == 1.0,
           "Inverse of q2 should have W==1")
    assert(qI.X() == 0.0,
           "Inverse of q2  should have W==0")
    assert(qI.Y() == 0.0,
           "Inverse of q2 should have W==0")
    assert(qI.Z() == 0.0,
           "Inverse of q2  should have W==0")

    # Test Euler angles
    for pitch in [-Math::PI*0.5, Math::PI*0.5] do
      for roll in (0..2 * Math::PI + 0.1).step(Math::PI*0.25) do
        for yaw in (0..2 * Math::PI + 0.1).step(Math::PI*0.25) do
          qOrig = Gz::Math::Quaterniond.new(roll, pitch, yaw)
          qDerived = Gz::Math::Quaterniond.new(qOrig.Euler())
          assert(qOrig == qDerived || qOrig == -qDerived,
                 "Singularities should be handled correctly")
        end 
      end
    end

    # Test construction from axis angle
    qA = Gz::Math::Quaterniond.new(Gz::Math::Vector3d.new(0, 0, 1),
                                     Math::PI)
    assert(qA.X() == 0.0, "X should equal 0")
    assert(qA.Y() == 0.0, "Y should equal 0")
    assert(qA.Z() == 1.0, "Z should equal 1")
    assert((qA.W() - 0).abs < 1e-3, "W should equal 0")

    # Test the defined identity quaternion
    qIdent = Gz::Math::Quaterniond.Identity
    assert(qIdent.W() == 1.0, "Identity W should equal 1")
    assert(qIdent.X() == 0.0, "Identity X should equal 0")
    assert(qIdent.Y() == 0.0, "Identity Y should equal 0")
    assert(qIdent.Z() == 0.0, "Identity Z should equal 0")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Quaternion_TEST).passed? ? 0 : -1

