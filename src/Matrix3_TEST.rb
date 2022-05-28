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

class Matrix3_TEST < Test::Unit::TestCase
  def test_construction
    m = Gz::Math::Matrix3d.new
    assert(m.(0, 0) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (0,0)==0")
    assert(m.(0, 1) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (0,1)==0")
    assert(m.(0, 2) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (0,2)==0")
    assert(m.(1, 0) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (1,0)==0")
    assert(m.(1, 1) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (1,1)==0")
    assert(m.(1, 2) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (1,2)==0")
    assert(m.(2, 0) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (2,0)==0")
    assert(m.(2, 1) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (2,1)==0")
    assert(m.(2, 2) == 0.0,
           "Gz::Math::Matrix3d default constructor should have (2,2)==0")

    m1 = Gz::Math::Matrix3d.new(1, 2, 3, 4, 5, 6 , 7, 8, 9)
    assert(m1.(0, 0) == 1.0,
           "m1 should have (0,0)==1")
    assert(m1.(0, 1) == 2.0,
           "m1 should have (0,1)==2")
    assert(m1.(0, 2) == 3.0,
           "m1 should have (0,2)==3")
    assert(m1.(1, 0) == 4.0,
           "m1 should have (1,0)==4")
    assert(m1.(1, 1) == 5.0,
           "m1 should have (1,1)==5")
    assert(m1.(1, 2) == 6.0,
           "m1 should have (1,2)==6")
    assert(m1.(2, 0) == 7.0,
           "m1 should have (2,0)==7")
    assert(m1.(2, 1) == 8.0,
           "m1 should have (2,1)==8")
    assert(m1.(2, 2) == 9.0,
           "m1 should have (2,2)==9")

    mT = m1.Transposed()
    puts mT.(0,0)
    puts mT.(0,1)
    puts mT.(0,2)
    puts mT.(1,0)
    puts mT.(1,1)
    puts mT.(1,2)
    puts mT.(2,0)
    puts mT.(2,1)
    puts mT.(2,2)

    assert(mT.(0, 0) == 1.0,
           "mT should have (0,0)==1")
    assert(mT.(0, 1) == 4.0,
           "mT should have (0,1)==4")
    assert(mT.(0, 2) == 7.0,
           "mT should have (0,2)==7")
    assert(mT.(1, 0) == 2.0,
           "mT should have (1,0)==2")
    assert(mT.(1, 1) == 5.0,
           "mT should have (1,1)==5")
    assert(mT.(1, 2) == 8.0,
           "mT should have (1,2)==8")
    assert(mT.(2, 0) == 3.0,
           "mT should have (2,0)==3")
    assert(mT.(2, 1) == 6.0,
           "mT should have (2,1)==6")
    assert(mT.(2, 2) == 9.0,
           "mT should have (2,2)==9")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Matrix3_TEST).passed? ? 0 : -1 
