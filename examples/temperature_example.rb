# Copyright (C) 2020 Open Source Robotics Foundation
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

# This example will only work if the Ruby interface library was compiled and
# installed.
#
# Modify the RUBYLIB environment variable to include the ignition math
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

celsius = Ignition::Math::Temperature::KelvinToCelsius(2.5);
printf("2.5Kelvin to Celsius is %f\n", celsius)

temp = Ignition::Math::Temperature.new(123.5)
printf("Constructed a Temperature object with %f Kelvin\n",
       temp.Kelvin())

printf("Same temperature in Celsius %f\n", temp.Celsius())

temp += 100.0
printf("Temperature + 100.0 is %fK", temp.Kelvin())

newTemp = Ignition::Math::Temperature.new(temp.Kelvin())
newTemp += temp + 23.5;
printf("Copied temp and added 23.5K. The new tempurature is %fF\n",
    newTemp.Fahrenheit());
