# Copyright (C) 2020 Open Source Robotics Foundation
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

class GaussMarkovProcess_TEST < Test::Unit::TestCase
  def test_construction
    gmp = Gz::Math::GaussMarkovProcess.new

    assert(gmp.Start() == 0.0, "Start value should equal zero")
    assert(gmp.Value() == 0.0, "Initial value should equal zero")
    assert(gmp.Theta() == 0.0, "Theta should equal zero")
    assert(gmp.Mu() == 0.0, "Mu should equal zero")
    assert(gmp.Sigma() == 0.0, "Sigma should equal zero")
  end

  def test_no_noise
    gmp = Gz::Math::GaussMarkovProcess.new(-1.2, 1.0, 2.5, 0)
    assert(gmp.Start() == -1.2, "Start value should equal zero")
    assert(gmp.Value() == -1.2, "Initial value should equal zero")
    assert(gmp.Theta() == 1.0, "Theta should equal zero")
    assert(gmp.Mu() == 2.5, "Mu should equal zero")
    assert(gmp.Sigma() == 0.0, "Sigma should equal zero")

    for i in 0..200 do
      value = gmp.Update(0.1);
      assert(value > -1.2, "Value should be greater than -1.2")
    end

    assert(gmp.Value() >= 2.4999, "The final value should be near 2.5")
    assert(gmp.Value() <= 2.5, "The final value should be near 2.5")
  end

  def test_noise
    gmp = Gz::Math::GaussMarkovProcess.new(20.2, 0.1, 0, 0.5)
    assert(gmp.Start() == 20.2, "Start value should equal zero")
    assert(gmp.Value() == 20.2, "Initial value should equal zero")
    assert(gmp.Theta() == 0.1, "Theta should equal zero")
    assert(gmp.Mu() == 0, "Mu should equal zero")
    assert(gmp.Sigma() == 0.5, "Sigma should equal zero")
    Gz::Math::Rand::Seed(1001);

    for i in 0..1000 do
      value = gmp.Update(0.1);
      assert(value > -11, "Value should be greater than -10")
      assert(value < 22, "Value should be less than 25")
    end
    # Hand-tuned values that are repeatable given the seed set above.
    assert((gmp.Value() + 3.99148).abs <= 1e-4);
  end

end

exit Test::Unit::UI::Console::TestRunner.run(GaussMarkovProcess_TEST).passed? ? 0 : -1
