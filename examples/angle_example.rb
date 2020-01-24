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

# This example will only work if the Ruby interface library was compiled and
# installed.
#
# Modify the RUBYLIB environment variable to include the ignition math
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

#! [constant]
printf("PI in degrees = %f\n", Ignition::Math::Angle.Pi.Degree)
#! [constant]

a1 = Ignition::Math::Angle.new(1.5707)
a2 = Ignition::Math::Angle.new(0.7854)
printf("a1 = %f radians, %f degrees\n", a1.Radian, a1.Degree)
printf("a2 = %f radians, %f degrees\n", a2.Radian, a2.Degree)
printf("a1 * a2 = %f radians, %f degrees\n", (a1 * a2).Radian, (a1 * a2).Degree)
printf("a1 + a2 = %f radians, %f degrees\n", (a1 + a2).Radian, (a1 + a2).Degree)
printf("a1 - a2 = %f radians, %f degrees\n", (a1 - a2).Radian, (a1 - a2).Degree)

a3 = Ignition::Math::Angle.new(15.707)
printf("a3 = %f radians, %f degrees\n", a3.Radian, a3.Degree)
a3.Normalize
printf("a3.Normalize = %f radians, %f degrees\n", a3.Radian, a3.Degree)
