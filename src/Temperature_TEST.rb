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

class Temperature_TEST < Test::Unit::TestCase
  def test_construction
    tmp = Gz::Math::Temperature.new

    assert(tmp.Kelvin() == 0.0,
           "Temperature::Kelvin() should equal zero")

    tmp.SetKelvin(123.5)
    assert(tmp.Kelvin() == 123.5,
           "Temperature::Kelvin() should equal 123.5")

    tmp2 = Gz::Math::Temperature.new(123.5)
    assert(tmp == tmp2, "Both temperatures should be the same.")

    tmp2.SetCelsius(123.6)
    assert(tmp != tmp2, "Both temperatures should not be the same.")
    assert(tmp < tmp2, "123.6K should be less than 123.6C")

    assert(Gz::Math::Temperature::CelsiusToKelvin(123.6) == tmp2.Kelvin(),
           "123.6C convert to Kelvin should equal tmp2" )
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Temperature_TEST).passed? ? 0 : -1
