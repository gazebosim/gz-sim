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

class Helpers_TEST < Test::Unit::TestCase
  def Helpers
    assert(12345 == Gz::Math::parseInt("12345"),
          "The string '12345' should parse to an integer")
    assert(-12345 == Gz::Math::parseInt("-12345"),
          "The string '-12345' should parse to an integer")
    assert(-12345 == Gz::Math::parseInt("    -12345"))
    assert(0 == Gz::Math::parseInt("    "),
          "The string '    ' should parse to 0")
    assert(23 == Gz::Math::parseInt("23ab67"),
          "The string '23ab67' should parse to 23")
  end
end

exit Test::Unit::UI::Console::TestRunner.run(Helpers_TEST).passed? ? 0 : -1
